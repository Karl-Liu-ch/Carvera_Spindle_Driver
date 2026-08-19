/*
  Carvera PWM -> Arduino Nano R4 -> ODrive CANSimple

  Wiring:
    D2  <- Carvera spindle PWM command, approximately 1 kHz
    D3  -> Carvera Blue speed feedback, 12 pulses/revolution
    D4  -> CAN transceiver TXD (Nano R4 native CAN TX)
    D5  <- CAN transceiver RXD (Nano R4 native CAN RX)
    D6  -> Carvera Red alarm, LOW on fault / high impedance when normal

  CANH/CANL connect the transceiver to the ODrive. ODrive, Nano R4,
  transceiver and Carvera must share GND. Terminate both physical ends of the
  CAN bus with 120 ohms.

  ODrive S1 firmware 0.6.11 must already be commissioned and saved for
  velocity control, CANSimple and Hall sensor feedback:
    hall_encoder0.config.enabled = True
    axis0.config.load_encoder = EncoderId.HALL_ENCODER0
    axis0.config.commutation_encoder = EncoderId.HALL_ENCODER0
  Connect the Hall signals to the S1 HALL_A, HALL_B and HALL_C inputs, then
  complete Hall polarity/phase calibration in odrivetool before saving the
  configuration. Hall feedback does not use an encoder index search.
  Save this configuration before using the sketch. Nano R4's Arduino_CAN API
  cannot send RTR frames, so configure these ODrive cyclic messages:
    heartbeat_msg_rate_ms <= 100
    encoder_msg_rate_ms   = 10
    iq_msg_rate_ms        = 10
    bus_voltage_msg_rate_ms <= 100
    error_msg_rate_ms     <= 100
  Get_Error telemetry is required to capture the disarm reason and confirm an
  automatic or manual clear before releasing the local fault latch.
*/

#include <Arduino.h>
#include <Arduino_CAN.h>
#include "pwm.h"
#include <math.h>
#include <string.h>

// Hardware
static constexpr uint8_t PWM_INPUT_PIN = 2;
static constexpr uint8_t BLUE_SPEED_FEEDBACK_PIN = 3;
static constexpr uint8_t RED_ALARM_SIGNAL_PIN = 6;

// Spindle and motor. Positive Carvera commands are sent in the selected ODrive
// direction; change DIRECTION_SIGN only after a low-speed direction test.
static constexpr float GEAR_RATIO = 1.635f;
static constexpr float MAX_SPINDLE_RPM = 12500.0f;
static constexpr float MAX_MOTOR_RPM = 8400.0f;
static constexpr float DIRECTION_SIGN = 1.0f;
static constexpr float SPEED_FEEDBACK_PPR = 12.0f;
static constexpr float START_DUTY_THRESHOLD = 0.002f;
static constexpr float FILTER_ALPHA = 0.20f;
static constexpr float IQ_FILTER_ALPHA = 0.10f;
static constexpr float BLUE_FEEDBACK_FILTER_ALPHA = 0.25f;

// Expected PWM input: approximately 1 kHz.
static constexpr uint32_t MIN_PWM_PERIOD_US = 700;
static constexpr uint32_t MAX_PWM_PERIOD_US = 1500;
static constexpr uint32_t PWM_EDGE_TIMEOUT_US = 100000;

// Timing and protection values mirror SOLO_NANO_R4_CAN_v1.
static constexpr uint32_t COMMUNICATION_SLOT_MS = 25;
static constexpr uint32_t SCHEDULED_ERROR_INTERVAL_MS = 500;
static constexpr uint32_t FIRST_ERROR_SLOT_OFFSET_MS = 525;
static constexpr uint32_t PRINT_INTERVAL_MS = 500;
static constexpr uint32_t ERROR_CLEAR_RETRY_MS = 1000;
// Keep velocity control active at a 0 rpm reference for this long after the
// PWM run command is removed, then release the motor by switching to IDLE.
static constexpr uint32_t DRIVE_DISABLE_DELAY_MS = 4000;
static constexpr uint32_t COMMUNICATION_SAFETY_RETRY_MS = 500;
static constexpr uint32_t STARTUP_RETRY_INTERVAL_MS = 100;
static constexpr uint32_t TELEMETRY_TIMEOUT_MS = 350;
static constexpr uint8_t COMMUNICATION_FAILURE_ALARM_THRESHOLD = 8;
static constexpr float SPEED_DROP_ALARM_RATIO = 0.20f;
static constexpr float SPEED_DROP_MONITOR_MIN_RPM = 500.0f;
static constexpr float SPEED_TARGET_STEP_MIN_RPM = 200.0f;
static constexpr uint32_t SPEED_DROP_CONFIRMATION_MS = 2000;
static constexpr uint32_t POWER_INDEX_CONFIRMATION_MS = 1000;
static constexpr float POWER_INDEX_LIMIT = 58000.0f;

static constexpr unsigned long PC_BAUDRATE = 115200;
static constexpr uint8_t ODRIVE_NODE_ID = 0;
static constexpr CanBitRate CAN_BITRATE = CanBitRate::BR_1000k;

PwmOut blueFeedbackPwm(BLUE_SPEED_FEEDBACK_PIN);

// ODrive CANSimple IDs: standard ID = (node ID << 5) | command ID.
enum class ODriveCommand : uint8_t {
  HEARTBEAT = 0x01,
  GET_ERROR = 0x03,
  SET_AXIS_STATE = 0x07,
  GET_ENCODER_ESTIMATES = 0x09,
  SET_CONTROLLER_MODE = 0x0B,
  SET_INPUT_VEL = 0x0D,
  GET_IQ = 0x14,
  GET_BUS_VOLTAGE_CURRENT = 0x17,
  CLEAR_ERRORS = 0x18
};

static constexpr uint32_t AXIS_STATE_IDLE = 1;
static constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
static constexpr uint32_t CONTROL_MODE_VELOCITY_CONTROL = 2;
static constexpr uint32_t INPUT_MODE_VEL_RAMP = 2;

// PWM capture, shared with interrupt.
volatile uint32_t lastRiseTimeUs = 0;
volatile uint32_t pendingHighTimeUs = 0;
volatile uint32_t measuredHighTimeUs = 0;
volatile uint32_t measuredPeriodUs = 0;
volatile uint32_t lastEdgeSampleTimeUs = 0;
volatile bool pendingHighTimeReady = false;
volatile bool measurementReady = false;

enum class PWMReadResult : uint8_t {
  NEW_SAMPLE,
  NO_NEW_SAMPLE
};

// Runtime state.
float filteredDuty = 0.0f;
float requestedMotorRpm = 0.0f;
float targetMotorRpm = 0.0f;
float encoderPositionTurns = 0.0f;
float encoderVelocityTurnsPerSecond = 0.0f;
float measuredMotorRpm = 0.0f;
float motorIqSetpointA = 0.0f;
float motorIqMeasuredSignedA = 0.0f;
float filteredMotorIqSignedA = 0.0f;
float measuredMotorIqA = 0.0f;
float currentPowerIndex = 0.0f;
float speedFeedbackFrequencyHz = 0.0f;
float blueOutputFrequencyHz = 0.0f;
float filteredBlueFeedbackFrequencyHz = 0.0f;
float odriveBusVoltageV = 0.0f;
float odriveBusCurrentA = 0.0f;
bool pwmSignalValid = false;
bool canDriverReady = false;
bool odriveCommunicationValid = false;
bool odriveDriveEnabled = false;
bool odriveHasError = false;
bool odriveFaultLatched = false;
bool communicationFaultLatched = false;
bool speedDropFaultLatched = false;
bool speedDropMonitorArmed = false;
bool motorIqValid = false;
bool motorIqFilterInitialized = false;
bool busTelemetryValid = false;
bool powerIndexFaultLatched = false;
bool odriveStartupConfigured = false;
bool alarmActive = true;
bool blueFeedbackActive = false;
bool blueFeedbackPwmInitialized = false;
bool blueFeedbackFilterInitialized = false;
uint8_t consecutiveCommunicationFailures = 0;

uint8_t odriveState = 0;
uint8_t odriveProcedureResult = 0;
uint32_t odriveHeartbeatErrors = 0;
uint32_t odriveActiveErrors = 0;
uint32_t odriveDisarmReason = 0;
uint32_t latchedODriveHeartbeatErrors = 0;
uint32_t latchedODriveActiveErrors = 0;
uint32_t latchedODriveDisarmReason = 0;
uint32_t rejectedErrorFrameCount = 0;
bool errorClearPending = false;
bool startupErrorClearRequested = false;
bool errorIncidentReported = false;
uint32_t errorClearRequestTimeMs = 0;
uint32_t lastErrorClearAttemptTimeMs = 0;

uint32_t lastHeartbeatTimeMs = 0;
uint32_t lastEncoderTimeMs = 0;
uint32_t lastIqTimeMs = 0;
uint32_t lastBusTelemetryTimeMs = 0;
uint32_t lastErrorMessageTimeMs = 0;
uint32_t nextCommunicationSlotTimeMs = 0;
uint32_t communicationSlotIndex = 0;
uint32_t nextScheduledErrorTimeMs = 0;
uint32_t lastPrintTimeMs = 0;
uint32_t lastSuccessfulCommunicationMs = 0;
uint32_t blueFeedbackRestartCount = 0;
uint32_t blueFeedbackStopCount = 0;
uint32_t alarmActivationCount = 0;
uint32_t lastAlarmActivationTimeMs = 0;
uint32_t lastRunRequestedTimeMs = 0;
uint32_t lastCommunicationSafetyRetryMs = 0;
uint32_t nextStartupRetryTimeMs = 0;
uint32_t speedDropStartTimeMs = 0;
uint32_t powerIndexStartTimeMs = 0;
float lastSpeedReferenceSentRpm = -1.0f;
float lastSpeedDropMonitorTargetRpm = 0.0f;

enum class AlarmReason : uint8_t {
  NONE,
  COMMUNICATION,
  ODRIVE_ACTIVE_ERROR,
  ODRIVE_FAULT_LATCHED,
  SPEED_DROP,
  POWER_INDEX
};

AlarmReason lastAlarmReason = AlarmReason::NONE;
bool nextTelemetryReadIsIq = true;
uint8_t startupCommandStage = 0;
bool startupSequenceComplete = false;
bool nextSafetyCommandIsZero = true;
bool nextEmergencyCommandIsZero = true;
uint32_t lastEmergencyCommandTimeMs = 0;

// ---------------------------------------------------------------------------
// Byte packing and CANSimple transport
// ---------------------------------------------------------------------------

uint32_t makeCanId(ODriveCommand command)
{
  return (static_cast<uint32_t>(ODRIVE_NODE_ID) << 5) |
         static_cast<uint8_t>(command);
}

void writeUint32LE(uint8_t *destination, uint32_t value)
{
  destination[0] = static_cast<uint8_t>(value);
  destination[1] = static_cast<uint8_t>(value >> 8);
  destination[2] = static_cast<uint8_t>(value >> 16);
  destination[3] = static_cast<uint8_t>(value >> 24);
}

uint32_t readUint32LE(const uint8_t *source)
{
  return static_cast<uint32_t>(source[0]) |
         (static_cast<uint32_t>(source[1]) << 8) |
         (static_cast<uint32_t>(source[2]) << 16) |
         (static_cast<uint32_t>(source[3]) << 24);
}

void writeFloatLE(uint8_t *destination, float value)
{
  static_assert(sizeof(float) == 4, "CANSimple requires 32-bit float");
  uint32_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));
  writeUint32LE(destination, bits);
}

float readFloatLE(const uint8_t *source)
{
  const uint32_t bits = readUint32LE(source);
  float value = 0.0f;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

bool transmitCan(ODriveCommand command, const uint8_t *data, uint8_t length)
{
  if (!canDriverReady) {
    return false;
  }

  const CanMsg message(
    CanStandardId(makeCanId(command)), length, data
  );
  return CAN.write(message) > 0;
}

bool sendAxisState(uint32_t requestedState)
{
  uint8_t payload[4] = {};
  writeUint32LE(payload, requestedState);
  return transmitCan(ODriveCommand::SET_AXIS_STATE, payload, sizeof(payload));
}

bool sendControllerMode()
{
  uint8_t payload[8] = {};
  writeUint32LE(payload, CONTROL_MODE_VELOCITY_CONTROL);
  writeUint32LE(payload + 4, INPUT_MODE_VEL_RAMP);
  return transmitCan(
    ODriveCommand::SET_CONTROLLER_MODE, payload, sizeof(payload)
  );
}

bool sendVelocityRpm(float rpm)
{
  const float commandRpm = constrain(rpm, 0.0f, MAX_MOTOR_RPM);
  uint8_t payload[8] = {};
  writeFloatLE(payload, DIRECTION_SIGN * commandRpm / 60.0f);
  writeFloatLE(payload + 4, 0.0f);

  const bool accepted = transmitCan(
    ODriveCommand::SET_INPUT_VEL, payload, sizeof(payload)
  );
  if (accepted) {
    lastSpeedReferenceSentRpm = commandRpm;
  }
  return accepted;
}

bool clearODriveErrors()
{
  const uint8_t identify = 0;
  return transmitCan(ODriveCommand::CLEAR_ERRORS, &identify, 1);
}

// ---------------------------------------------------------------------------
// PWM input
// ---------------------------------------------------------------------------

void pwmEdgeISR()
{
  const uint32_t nowUs = micros();
  const bool inputHigh = digitalRead(PWM_INPUT_PIN);

  if (inputHigh) {
    if (lastRiseTimeUs != 0) {
      const uint32_t periodUs = nowUs - lastRiseTimeUs;
      if (pendingHighTimeReady &&
          periodUs >= MIN_PWM_PERIOD_US &&
          periodUs <= MAX_PWM_PERIOD_US &&
          pendingHighTimeUs <= periodUs) {
        measuredPeriodUs = periodUs;
        measuredHighTimeUs = pendingHighTimeUs;
        lastEdgeSampleTimeUs = nowUs;
        measurementReady = true;
      }
    }
    lastRiseTimeUs = nowUs;
    pendingHighTimeReady = false;
  }
  else if (lastRiseTimeUs != 0) {
    const uint32_t highTimeUs = nowUs - lastRiseTimeUs;
    if (highTimeUs <= MAX_PWM_PERIOD_US) {
      pendingHighTimeUs = highTimeUs;
      pendingHighTimeReady = true;
    }
  }
}

PWMReadResult readPWMDuty(float &duty)
{
  uint32_t highTimeUs;
  uint32_t periodUs;
  uint32_t sampleTimeUs;
  bool newMeasurement;

  noInterrupts();
  highTimeUs = measuredHighTimeUs;
  periodUs = measuredPeriodUs;
  sampleTimeUs = lastEdgeSampleTimeUs;
  newMeasurement = measurementReady;
  measurementReady = false;
  interrupts();

  const bool edgeTimedOut = sampleTimeUs == 0 ||
    static_cast<uint32_t>(micros() - sampleTimeUs) > PWM_EDGE_TIMEOUT_US;

  if (edgeTimedOut) {
    // INPUT_PULLDOWN makes a disconnected wire read as 0%; steady HIGH is a
    // valid 100% command, so exact endpoint duties remain usable.
    duty = digitalRead(PWM_INPUT_PIN) ? 1.0f : 0.0f;
    return PWMReadResult::NEW_SAMPLE;
  }

  if (!newMeasurement || periodUs == 0) {
    return PWMReadResult::NO_NEW_SAMPLE;
  }

  duty = constrain(
    static_cast<float>(highTimeUs) / static_cast<float>(periodUs),
    0.0f, 1.0f
  );
  return PWMReadResult::NEW_SAMPLE;
}

// ---------------------------------------------------------------------------
// Carvera outputs and received ODrive telemetry
// ---------------------------------------------------------------------------

void updateBlueSpeedFeedback(float motorRpm, bool valid)
{
  // Carvera applies the transmission-ratio conversion itself.
  speedFeedbackFrequencyHz =
    fabsf(motorRpm) / 60.0f * SPEED_FEEDBACK_PPR;

  if (!valid || !isfinite(speedFeedbackFrequencyHz) ||
      speedFeedbackFrequencyHz < 0.5f) {
    if (blueFeedbackActive) {
      blueFeedbackPwm.suspend();
      blueFeedbackActive = false;
      blueOutputFrequencyHz = 0.0f;
      ++blueFeedbackStopCount;
    }
    blueFeedbackFilterInitialized = false;
    speedFeedbackFrequencyHz = 0.0f;
    return;
  }

  if (!blueFeedbackFilterInitialized) {
    filteredBlueFeedbackFrequencyHz = speedFeedbackFrequencyHz;
    blueFeedbackFilterInitialized = true;
  }
  else {
    filteredBlueFeedbackFrequencyHz += BLUE_FEEDBACK_FILTER_ALPHA *
      (speedFeedbackFrequencyHz - filteredBlueFeedbackFrequencyHz);
  }

  if (!blueFeedbackPwmInitialized) {
    if (!blueFeedbackPwm.begin(filteredBlueFeedbackFrequencyHz, 50.0f)) {
      speedFeedbackFrequencyHz = 0.0f;
      return;
    }
    blueFeedbackPwm.get_timer()->set_period_buffer(true);
    blueFeedbackPwmInitialized = true;
    blueFeedbackActive = true;
    ++blueFeedbackRestartCount;
  }
  else {
    // GPT remains running. A buffered period update takes effect at a PWM
    // boundary, so every encoder sample changes frequency without truncating
    // a pulse and without a frequency deadband.
    blueFeedbackPwm.get_timer()->set_frequency(
      filteredBlueFeedbackFrequencyHz);
    blueFeedbackPwm.pulse_perc(50.0f);
    if (!blueFeedbackActive) {
      blueFeedbackPwm.resume();
      blueFeedbackActive = true;
      ++blueFeedbackRestartCount;
    }
  }

  blueOutputFrequencyHz = filteredBlueFeedbackFrequencyHz;
}

const char *alarmReasonName(AlarmReason reason)
{
  switch (reason) {
    case AlarmReason::COMMUNICATION: return "COMMUNICATION";
    case AlarmReason::ODRIVE_ACTIVE_ERROR: return "ODRIVE_ACTIVE_ERROR";
    case AlarmReason::ODRIVE_FAULT_LATCHED: return "ODRIVE_FAULT_LATCHED";
    case AlarmReason::SPEED_DROP: return "SPEED_DROP";
    case AlarmReason::POWER_INDEX: return "POWER_INDEX";
    default: return "NONE";
  }
}

AlarmReason currentAlarmReason()
{
  if (communicationFaultLatched) return AlarmReason::COMMUNICATION;
  if (odriveHasError) return AlarmReason::ODRIVE_ACTIVE_ERROR;
  if (odriveFaultLatched) return AlarmReason::ODRIVE_FAULT_LATCHED;
  if (speedDropFaultLatched) return AlarmReason::SPEED_DROP;
  if (powerIndexFaultLatched) return AlarmReason::POWER_INDEX;
  return AlarmReason::NONE;
}

void setCarveraAlarm(bool active)
{
  const AlarmReason reason = active ? currentAlarmReason() : AlarmReason::NONE;
  if (active && !alarmActive) {
    ++alarmActivationCount;
    lastAlarmActivationTimeMs = millis();
    lastAlarmReason = reason;
    Serial.print("Carvera alarm activated: ");
    Serial.println(alarmReasonName(reason));
  }
  else if (active && reason != AlarmReason::NONE &&
           reason != lastAlarmReason) {
    lastAlarmReason = reason;
  }

  alarmActive = active;
  // Emulate an open-drain/contact-closure alarm output.  The Carvera input
  // supplies its own pull-up: float the line when normal and only sink it to
  // GND while an alarm is active.  Never actively drive this line HIGH.
  digitalWrite(RED_ALARM_SIGNAL_PIN, LOW);
  pinMode(RED_ALARM_SIGNAL_PIN, active ? OUTPUT : INPUT);
}

void recordCommunicationSuccess()
{
  odriveCommunicationValid = true;
  lastSuccessfulCommunicationMs = millis();
  consecutiveCommunicationFailures = 0;
}

void recordCommunicationFailure()
{
  odriveCommunicationValid = false;
  if (consecutiveCommunicationFailures < UINT8_MAX) {
    ++consecutiveCommunicationFailures;
  }
}

bool communicationFailureLimitExceeded()
{
  return consecutiveCommunicationFailures >=
         COMMUNICATION_FAILURE_ALARM_THRESHOLD;
}

bool telemetryIsFresh(uint32_t sampleTimeMs, uint32_t nowMs)
{
  return sampleTimeMs != 0 && nowMs - sampleTimeMs <= TELEMETRY_TIMEOUT_MS;
}

void refreshODriveErrorState(uint32_t nowMs)
{
  (void)nowMs;
  const bool hasError = odriveHeartbeatErrors != 0 ||
                        odriveActiveErrors != 0 ||
                        odriveDisarmReason != 0;
  odriveHasError = hasError;
}

void captureODriveError(uint32_t activeErrors, uint32_t disarmReason)
{
  // Preserve the complete most-recent nonzero error pair across successful
  // auto-clears. Assign both fields together so codes from two incidents are
  // never mixed.
  if (activeErrors != 0 || disarmReason != 0) {
    latchedODriveActiveErrors = activeErrors;
    latchedODriveDisarmReason = disarmReason;
  }
}

void processReceivedCan()
{
  while (CAN.available()) {
    const CanMsg message = CAN.read();
    if (!message.isStandardId()) {
      continue;
    }

    const uint32_t identifier = message.getStandardId();
    if ((identifier >> 5) != ODRIVE_NODE_ID) {
      continue;
    }

    const uint8_t commandId = identifier & 0x1F;
    const uint32_t nowMs = millis();

    if (commandId == static_cast<uint8_t>(ODriveCommand::HEARTBEAT) &&
        message.data_length >= 6) {
      odriveHeartbeatErrors = readUint32LE(message.data);
      if (odriveHeartbeatErrors != 0) {
        latchedODriveHeartbeatErrors = odriveHeartbeatErrors;
      }
      odriveState = message.data[4];
      odriveProcedureResult = message.data[5];
      odriveDriveEnabled =
        odriveState == AXIS_STATE_CLOSED_LOOP_CONTROL;
      lastHeartbeatTimeMs = nowMs;
      refreshODriveErrorState(nowMs);
    }
    else if (commandId == static_cast<uint8_t>(ODriveCommand::GET_ERROR) &&
             message.data_length >= 8) {
      const uint32_t receivedActiveErrors = readUint32LE(message.data);
      const uint32_t receivedDisarmReason = readUint32LE(message.data + 4);

      // Heartbeat reports active_errors | disarm_reason. Reject any Get_Error
      // payload that is inconsistent with the latest fresh Heartbeat. This
      // prevents a corrupted/misidentified float payload (often containing a
      // NaN bit pattern such as 0x7FC00000) from becoming a false fault.
      if (telemetryIsFresh(lastHeartbeatTimeMs, nowMs) &&
          (receivedActiveErrors | receivedDisarmReason) ==
            odriveHeartbeatErrors) {
        odriveActiveErrors = receivedActiveErrors;
        odriveDisarmReason = receivedDisarmReason;
        captureODriveError(odriveActiveErrors, odriveDisarmReason);
        lastErrorMessageTimeMs = nowMs;
        refreshODriveErrorState(nowMs);
      }
      else {
        ++rejectedErrorFrameCount;
      }
    }
    else if (commandId ==
               static_cast<uint8_t>(ODriveCommand::GET_ENCODER_ESTIMATES) &&
             message.data_length >= 8) {
      const float velocityTurnsPerSecond = readFloatLE(message.data + 4);
      const float positionTurns = readFloatLE(message.data);
      if (isfinite(positionTurns) && isfinite(velocityTurnsPerSecond)) {
        encoderPositionTurns = positionTurns;
        encoderVelocityTurnsPerSecond = velocityTurnsPerSecond;
        measuredMotorRpm = fabsf(velocityTurnsPerSecond * 60.0f);
        lastEncoderTimeMs = nowMs;
      }
    }
    else if (commandId == static_cast<uint8_t>(ODriveCommand::GET_IQ) &&
             message.data_length >= 8) {
      const float iqSetpoint = readFloatLE(message.data);
      const float iqMeasured = readFloatLE(message.data + 4);
      if (isfinite(iqSetpoint) && isfinite(iqMeasured)) {
        motorIqSetpointA = iqSetpoint;
        motorIqMeasuredSignedA = iqMeasured;
        if (!motorIqFilterInitialized) {
          filteredMotorIqSignedA = iqMeasured;
          motorIqFilterInitialized = true;
        }
        else {
          filteredMotorIqSignedA += IQ_FILTER_ALPHA *
            (iqMeasured - filteredMotorIqSignedA);
        }
        // Filter the signed current before taking its magnitude. Taking the
        // absolute value first would rectify the current ripple and bias the
        // load estimate upward.
        measuredMotorIqA = fabsf(filteredMotorIqSignedA);
        motorIqValid = true;
        lastIqTimeMs = nowMs;
      }
    }
    else if (commandId ==
               static_cast<uint8_t>(ODriveCommand::GET_BUS_VOLTAGE_CURRENT) &&
             message.data_length >= 8) {
      const float busVoltage = readFloatLE(message.data);
      const float busCurrent = readFloatLE(message.data + 4);
      if (isfinite(busVoltage) && isfinite(busCurrent)) {
        odriveBusVoltageV = busVoltage;
        odriveBusCurrentA = busCurrent;
        busTelemetryValid = true;
        lastBusTelemetryTimeMs = nowMs;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// ODrive state, command schedule and safety logic
// ---------------------------------------------------------------------------

bool deadlineReached(uint32_t nowMs, uint32_t deadlineMs)
{
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

bool setODriveEnabled(bool enabled)
{
  return sendAxisState(
    enabled ? AXIS_STATE_CLOSED_LOOP_CONTROL : AXIS_STATE_IDLE
  );
}

void disableODriveSafely()
{
  const uint32_t nowMs = millis();
  if (lastEmergencyCommandTimeMs != 0 &&
      nowMs - lastEmergencyCommandTimeMs < COMMUNICATION_SLOT_MS) {
    return;
  }

  lastEmergencyCommandTimeMs = nowMs;
  if (nextEmergencyCommandIsZero) sendVelocityRpm(0.0f);
  else setODriveEnabled(false);
  nextEmergencyCommandIsZero = !nextEmergencyCommandIsZero;
}

bool initializeODrive(uint32_t nowMs)
{
  if (!canDriverReady) {
    return false;
  }

  // Gains, motor/encoder calibration, current limits, CAN node ID/rate and
  // cyclic telemetry rates remain commissioned settings in the ODrive. Send
  // only one frame per retry because the RA4M1 has one transmit mailbox.
  bool accepted = false;
  if (startupCommandStage == 0) accepted = sendVelocityRpm(0.0f);
  else if (startupCommandStage == 1) accepted = setODriveEnabled(false);
  else accepted = sendControllerMode();

  if (!accepted) {
    return false;
  }
  if (startupCommandStage == 2) startupSequenceComplete = true;
  startupCommandStage = (startupCommandStage + 1U) % 3U;

  return startupSequenceComplete &&
         telemetryIsFresh(lastHeartbeatTimeMs, nowMs) &&
         telemetryIsFresh(lastEncoderTimeMs, nowMs) &&
         telemetryIsFresh(lastIqTimeMs, nowMs) &&
         odriveState == AXIS_STATE_IDLE && !odriveHasError;
}

void pollODriveErrors(uint32_t nowMs)
{
  if (telemetryIsFresh(lastHeartbeatTimeMs, nowMs)) {
    refreshODriveErrorState(nowMs);
    recordCommunicationSuccess();
  }
  else {
    recordCommunicationFailure();
  }
}

void pollODriveSpeed(uint32_t nowMs)
{
  const bool valid = telemetryIsFresh(lastEncoderTimeMs, nowMs);
  if (valid) {
    recordCommunicationSuccess();
  }
  else {
    recordCommunicationFailure();
  }

  updateBlueSpeedFeedback(
    measuredMotorRpm,
    valid && !communicationFailureLimitExceeded() &&
      !odriveHasError && !odriveFaultLatched
  );
}

void pollODriveIq(uint32_t nowMs)
{
  motorIqValid = telemetryIsFresh(lastIqTimeMs, nowMs);
  if (motorIqValid) {
    recordCommunicationSuccess();
  }
  else {
    recordCommunicationFailure();
  }
}

void executeScheduledSpeedCommand(uint32_t nowMs)
{
  const bool safeToRun =
    pwmSignalValid &&
    !odriveHasError &&
    !odriveFaultLatched &&
    !speedDropFaultLatched &&
    !powerIndexFaultLatched &&
    odriveStartupConfigured &&
    !communicationFaultLatched;
  const bool nonzeroCommand = targetMotorRpm > 0.0f;

  if (safeToRun && nonzeroCommand) {
    if (!odriveDriveEnabled) {
      setODriveEnabled(true);
    }

    if (odriveDriveEnabled) {
      sendVelocityRpm(targetMotorRpm);
    }
    lastRunRequestedTimeMs = nowMs;
  }
  else {
    // Keep velocity control active while ODrive's VEL_RAMP decelerates to
    // zero. Start the release delay only after the motor has nearly stopped.
    if (odriveDriveEnabled && measuredMotorRpm >= 50.0f) {
      lastRunRequestedTimeMs = nowMs;
    }
    if (odriveDriveEnabled &&
        nowMs - lastRunRequestedTimeMs >= DRIVE_DISABLE_DELAY_MS) {
      setODriveEnabled(false);
    }
    else {
      sendVelocityRpm(0.0f);
    }
  }
}

void runCommunicationSchedule(uint32_t nowMs)
{
  if (!deadlineReached(nowMs, nextCommunicationSlotTimeMs)) {
    return;
  }

  const uint32_t slotsLate =
    (nowMs - nextCommunicationSlotTimeMs) / COMMUNICATION_SLOT_MS;
  communicationSlotIndex += slotsLate;
  nextCommunicationSlotTimeMs +=
    (slotsLate + 1U) * COMMUNICATION_SLOT_MS;
  const uint32_t slotIndex = communicationSlotIndex++;

  if (communicationFaultLatched) {
    return;
  }

  if (!odriveStartupConfigured) {
    if (deadlineReached(nowMs, nextStartupRetryTimeMs)) {
      nextStartupRetryTimeMs = nowMs + STARTUP_RETRY_INTERVAL_MS;
      if (initializeODrive(nowMs)) {
        odriveStartupConfigured = true;
        Serial.println("ODrive startup configuration verified");
      }
      else {
        recordCommunicationFailure();
      }
    }
    return;
  }

  if ((slotIndex & 1U) == 0U) {
    executeScheduledSpeedCommand(nowMs);
    return;
  }

  if (deadlineReached(nowMs, nextScheduledErrorTimeMs)) {
    pollODriveErrors(nowMs);
    const uint32_t elapsedIntervals =
      (nowMs - nextScheduledErrorTimeMs) /
        SCHEDULED_ERROR_INTERVAL_MS + 1U;
    nextScheduledErrorTimeMs +=
      elapsedIntervals * SCHEDULED_ERROR_INTERVAL_MS;
    return;
  }

  if (nextTelemetryReadIsIq) {
    pollODriveIq(nowMs);
  }
  else {
    pollODriveSpeed(nowMs);
  }
  nextTelemetryReadIsIq = !nextTelemetryReadIsIq;
}

void updateSpeedDropMonitor(uint32_t nowMs)
{
  if (speedDropFaultLatched) {
    return;
  }

  if (targetMotorRpm < SPEED_DROP_MONITOR_MIN_RPM ||
      filteredDuty < START_DUTY_THRESHOLD) {
    speedDropMonitorArmed = false;
    speedDropStartTimeMs = 0;
    lastSpeedDropMonitorTargetRpm = targetMotorRpm;
    return;
  }

  if (!odriveDriveEnabled || !odriveCommunicationValid ||
      odriveHasError || odriveFaultLatched || communicationFaultLatched) {
    speedDropStartTimeMs = 0;
    return;
  }

  const float upwardStepThreshold = fmaxf(
    SPEED_TARGET_STEP_MIN_RPM,
    lastSpeedDropMonitorTargetRpm * SPEED_DROP_ALARM_RATIO
  );
  if (targetMotorRpm - lastSpeedDropMonitorTargetRpm >
      upwardStepThreshold) {
    speedDropMonitorArmed = false;
    speedDropStartTimeMs = 0;
  }
  lastSpeedDropMonitorTargetRpm = targetMotorRpm;

  const float minimumAllowedRpm =
    targetMotorRpm * (1.0f - SPEED_DROP_ALARM_RATIO);
  if (measuredMotorRpm >= minimumAllowedRpm) {
    speedDropMonitorArmed = true;
    speedDropStartTimeMs = 0;
    return;
  }

  if (!speedDropMonitorArmed) {
    return;
  }

  if (speedDropStartTimeMs == 0) {
    speedDropStartTimeMs = nowMs;
  }
  else if (nowMs - speedDropStartTimeMs >= SPEED_DROP_CONFIRMATION_MS) {
    speedDropFaultLatched = true;
    Serial.println(
      "Motor speed stayed below 80% of target for 2 s: fault latched");
  }
}

void updatePowerIndexMonitor(uint32_t nowMs)
{
  if (powerIndexFaultLatched) {
    return;
  }

  if (!motorIqValid || !odriveDriveEnabled ||
      targetMotorRpm < SPEED_DROP_MONITOR_MIN_RPM ||
      filteredDuty < START_DUTY_THRESHOLD ||
      odriveHasError || odriveFaultLatched || communicationFaultLatched) {
    currentPowerIndex = 0.0f;
    powerIndexStartTimeMs = 0;
    return;
  }

  currentPowerIndex = measuredMotorIqA * measuredMotorRpm;
  if (currentPowerIndex <= POWER_INDEX_LIMIT) {
    powerIndexStartTimeMs = 0;
    return;
  }

  if (powerIndexStartTimeMs == 0) {
    powerIndexStartTimeMs = nowMs;
  }
  else if (nowMs - powerIndexStartTimeMs >=
           POWER_INDEX_CONFIRMATION_MS) {
    powerIndexFaultLatched = true;
    Serial.println("Iq * motor RPM exceeded limit for 1 s: fault latched");
  }
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

void printStatus()
{
  const float requestedSpindleRpm = requestedMotorRpm * GEAR_RATIO;
  const float targetSpindleRpm = targetMotorRpm * GEAR_RATIO;
  const float measuredSpindleRpm = measuredMotorRpm * GEAR_RATIO;

  Serial.print("PWM: ");
  Serial.print(pwmSignalValid ? "VALID" : "NO");
  Serial.print(" | duty: ");
  Serial.print(filteredDuty * 100.0f, 1);
  Serial.print("% | requested motor: ");
  Serial.print(requestedMotorRpm, 0);
  Serial.print(" rpm | ramp target motor: ");
  Serial.print(targetMotorRpm, 0);
  Serial.print(" rpm | requested spindle: ");
  Serial.print(requestedSpindleRpm, 0);
  Serial.print(" rpm | ramp target spindle: ");
  Serial.print(targetSpindleRpm, 0);
  Serial.print(" rpm | measured motor: ");
  Serial.print(measuredMotorRpm, 0);
  Serial.print(" rpm | measured spindle: ");
  Serial.print(measuredSpindleRpm, 0);
  Serial.print(" rpm | Iq setpoint: ");
  if (motorIqValid) {
    Serial.print(motorIqSetpointA, 3);
    Serial.print(" A | Iq measured: ");
    Serial.print(motorIqMeasuredSignedA, 3);
    Serial.print(" A | Iq filtered: ");
    Serial.print(filteredMotorIqSignedA, 3);
    Serial.print(" A | power index: ");
    Serial.print(currentPowerIndex, 0);
  }
  else {
    Serial.print(
      "INVALID | Iq measured: INVALID | Iq filtered: INVALID"
      " | power index: INVALID");
  }
  Serial.print(" | Vbus: ");
  if (busTelemetryValid) {
    Serial.print(odriveBusVoltageV, 2);
    Serial.print(" V | Ibus: ");
    Serial.print(odriveBusCurrentA, 2);
    Serial.print(" A");
  }
  else {
    Serial.print("INVALID | Ibus: INVALID");
  }
  Serial.print(" | Hall position: ");
  if (telemetryIsFresh(lastEncoderTimeMs, millis())) {
    Serial.print(encoderPositionTurns, 5);
    Serial.print(" turn | Hall velocity: ");
    Serial.print(encoderVelocityTurnsPerSecond, 3);
    Serial.print(" turn/s");
  }
  else {
    Serial.print("INVALID | Hall velocity: INVALID");
  }
  Serial.print(" | Blue: ");
  Serial.print(speedFeedbackFrequencyHz, 1);
  Serial.print(" Hz requested / ");
  Serial.print(blueOutputFrequencyHz);
  Serial.print(" Hz output | Blue restarts: ");
  Serial.print(blueFeedbackRestartCount);
  Serial.print(" | Blue stops: ");
  Serial.print(blueFeedbackStopCount);
  Serial.print(" | drive: ");
  Serial.print(odriveDriveEnabled ? "ENABLED" : "DISABLED");
  Serial.print(" | speed monitor: ");
  if (speedDropFaultLatched) Serial.print("FAULT");
  else if (speedDropStartTimeMs != 0) Serial.print("TIMING");
  else Serial.print(speedDropMonitorArmed ? "ARMED" : "WAIT");
  Serial.print(" | power monitor: ");
  if (powerIndexFaultLatched) Serial.print("FAULT");
  else if (powerIndexStartTimeMs != 0) Serial.print("TIMING");
  else Serial.print("NORMAL");
  Serial.print(" | state: ");
  Serial.print(odriveState);
  Serial.print(" | procedure result: ");
  Serial.print(odriveProcedureResult);
  Serial.print(" | heartbeat errors: 0x");
  Serial.print(odriveHeartbeatErrors, HEX);
  Serial.print(" | active errors: 0x");
  Serial.print(odriveActiveErrors, HEX);
  Serial.print(" | disarm: 0x");
  Serial.print(odriveDisarmReason, HEX);
  Serial.print(" | latched heartbeat errors: 0x");
  Serial.print(latchedODriveHeartbeatErrors, HEX);
  Serial.print(" | latched active errors: 0x");
  Serial.print(latchedODriveActiveErrors, HEX);
  Serial.print(" | latched disarm: 0x");
  Serial.print(latchedODriveDisarmReason, HEX);
  Serial.print(" | rejected error frames: ");
  Serial.print(rejectedErrorFrameCount);
  Serial.print(" | auto clear: ");
  Serial.print(errorClearPending ? "PENDING" : "IDLE");
  Serial.print(" | CANSimple: ");
  Serial.print(odriveCommunicationValid ? "CONNECTED" : "LOST");
  Serial.print(" (");
  Serial.print(consecutiveCommunicationFailures);
  Serial.print(" consecutive failures) | Red: ");
  Serial.print(alarmActive ? "ALARM" : "NORMAL");
  Serial.print(" | alarm count: ");
  Serial.print(alarmActivationCount);
  Serial.print(" | last alarm: ");
  Serial.print(alarmReasonName(lastAlarmReason));
  Serial.print(" @ ");
  Serial.print(lastAlarmActivationTimeMs);
  Serial.println(" ms");
}

void handleUSBSerial()
{
  if (!Serial.available()) return;
  String command = Serial.readStringUntil('\n');
  command.trim();
  String compactCommand = command;
  compactCommand.replace(" ", "");
  compactCommand.replace("\t", "");

  if (compactCommand.equalsIgnoreCase("status")) printStatus();
  else if (compactCommand.equalsIgnoreCase("clear_errors")) {
    if (filteredDuty >= START_DUTY_THRESHOLD) {
      Serial.println(
        "Clear rejected: set the Carvera spindle command to zero first"
      );
    }
    else if (!canDriverReady ||
             !telemetryIsFresh(lastHeartbeatTimeMs, millis())) {
      Serial.println("Clear rejected: ODrive CAN heartbeat is not available");
    }
    else if (clearODriveErrors()) {
      errorClearPending = true;
      errorClearRequestTimeMs = millis();
      lastErrorClearAttemptTimeMs = errorClearRequestTimeMs;
      disableODriveSafely();
      Serial.println(
        "ODrive CLEAR_ERRORS sent; waiting for zero error telemetry"
      );
    }
    else {
      Serial.println("CLEAR_ERRORS CAN frame could not be queued");
    }
  }
  else if (compactCommand.equalsIgnoreCase("help")) {
    Serial.println("Commands: status, clear_errors");
  }
  else if (command.length() != 0) {
    Serial.println("Unknown command. Enter 'help'");
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup()
{
  pinMode(PWM_INPUT_PIN, INPUT_PULLDOWN);
  pinMode(BLUE_SPEED_FEEDBACK_PIN, OUTPUT);
  digitalWrite(BLUE_SPEED_FEEDBACK_PIN, LOW);
  setCarveraAlarm(false);

  Serial.begin(PC_BAUDRATE);
  Serial.setTimeout(20);
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmEdgeISR, CHANGE);
  canDriverReady = CAN.begin(CAN_BITRATE);
  Serial.println("Nano R4 Carvera / ODrive CANSimple interface startup");

  if (!canDriverReady) {
    Serial.println("CAN.begin failed");
  }
  else {
    // A safe zero frame also serves as an autobaud beacon. Remaining startup
    // frames are issued one at a time by the scheduler.
    sendVelocityRpm(0.0f);
  }

  const uint32_t schedulerStartTimeMs = millis();
  lastRunRequestedTimeMs = schedulerStartTimeMs;
  nextCommunicationSlotTimeMs = schedulerStartTimeMs;
  nextScheduledErrorTimeMs =
    schedulerStartTimeMs + FIRST_ERROR_SLOT_OFFSET_MS;
  nextStartupRetryTimeMs =
    schedulerStartTimeMs + STARTUP_RETRY_INTERVAL_MS;
}

void loop()
{
  processReceivedCan();
  handleUSBSerial();

  float newDuty = 0.0f;
  if (readPWMDuty(newDuty) == PWMReadResult::NEW_SAMPLE) {
    pwmSignalValid = true;
    if (newDuty < START_DUTY_THRESHOLD) newDuty = 0.0f;
    filteredDuty += FILTER_ALPHA * (newDuty - filteredDuty);
    if (filteredDuty < START_DUTY_THRESHOLD) filteredDuty = 0.0f;
  }

  const float pwmRequestedSpindleRpm = filteredDuty * MAX_SPINDLE_RPM;
  requestedMotorRpm = constrain(
    pwmRequestedSpindleRpm / GEAR_RATIO, 0.0f, MAX_MOTOR_RPM
  );
  targetMotorRpm = requestedMotorRpm;

  const uint32_t nowMs = millis();
  busTelemetryValid = telemetryIsFresh(lastBusTelemetryTimeMs, nowMs);
  runCommunicationSchedule(nowMs);
  updateSpeedDropMonitor(nowMs);
  updatePowerIndexMonitor(nowMs);

  if (odriveHasError) {
    odriveFaultLatched = true;
    errorClearPending = true;

    if (!errorIncidentReported) {
      errorIncidentReported = true;
      Serial.print("ODrive error detected | heartbeat: 0x");
      Serial.print(odriveHeartbeatErrors, HEX);
      Serial.print(" | active: 0x");
      Serial.print(odriveActiveErrors, HEX);
      Serial.print(" | disarm: 0x");
      Serial.println(odriveDisarmReason, HEX);
    }
  }

  // Clear once after every restart, and retry automatically while an ODrive
  // error is present. The saved last-error fields are intentionally retained
  // after recovery for later diagnosis.
  const bool heartbeatAvailable =
    telemetryIsFresh(lastHeartbeatTimeMs, nowMs);
  const bool startupClearDue =
    heartbeatAvailable && !startupErrorClearRequested;
  const bool retryClearDue =
    errorClearPending && heartbeatAvailable &&
    (lastErrorClearAttemptTimeMs == 0 ||
     nowMs - lastErrorClearAttemptTimeMs >= ERROR_CLEAR_RETRY_MS);

  if (startupClearDue || retryClearDue) {
    if (clearODriveErrors()) {
      startupErrorClearRequested = true;
      errorClearPending = true;
      errorClearRequestTimeMs = nowMs;
      lastErrorClearAttemptTimeMs = nowMs;
      Serial.println(
        startupClearDue
          ? "Startup ODrive CLEAR_ERRORS sent"
          : "Automatic ODrive CLEAR_ERRORS retry sent"
      );
    }
  }

  // A newer, cross-checked Get_Error frame must confirm zero before the alarm
  // and local fault latch are released. Do not erase the saved last error.
  if (errorClearPending &&
      lastErrorMessageTimeMs > errorClearRequestTimeMs &&
      !odriveHasError) {
    odriveFaultLatched = false;
    errorClearPending = false;
    errorClearRequestTimeMs = 0;
    lastErrorClearAttemptTimeMs = 0;
    errorIncidentReported = false;
    Serial.print("ODrive error clear confirmed; returning to NORMAL");
    Serial.print(" | last heartbeat: 0x");
    Serial.print(latchedODriveHeartbeatErrors, HEX);
    Serial.print(" | last active: 0x");
    Serial.print(latchedODriveActiveErrors, HEX);
    Serial.print(" | last disarm: 0x");
    Serial.println(latchedODriveDisarmReason, HEX);
  }

  if (speedDropFaultLatched &&
      filteredDuty < START_DUTY_THRESHOLD) {
    disableODriveSafely();
    speedDropFaultLatched = false;
    speedDropMonitorArmed = false;
    speedDropStartTimeMs = 0;
  }

  if (powerIndexFaultLatched &&
      filteredDuty < START_DUTY_THRESHOLD) {
    disableODriveSafely();
    powerIndexFaultLatched = false;
    powerIndexStartTimeMs = 0;
    currentPowerIndex = 0.0f;
  }

  if (communicationFailureLimitExceeded()) {
    communicationFaultLatched = true;
  }

  if (communicationFaultLatched &&
      nowMs - lastCommunicationSafetyRetryMs >=
        COMMUNICATION_SAFETY_RETRY_MS) {
    lastCommunicationSafetyRetryMs = nowMs;
    if (nextSafetyCommandIsZero) sendVelocityRpm(0.0f);
    else setODriveEnabled(false);
    nextSafetyCommandIsZero = !nextSafetyCommandIsZero;
  }

  // CAN.write only confirms local enqueue. Require fresh ODrive telemetry and
  // an observed IDLE state before releasing a communication fault, and only
  // release it after the Carvera run command has returned to zero.
  if (communicationFaultLatched &&
      telemetryIsFresh(lastHeartbeatTimeMs, nowMs) &&
      telemetryIsFresh(lastEncoderTimeMs, nowMs) &&
      telemetryIsFresh(lastIqTimeMs, nowMs) &&
      odriveState == AXIS_STATE_IDLE &&
      lastSpeedReferenceSentRpm == 0.0f &&
      filteredDuty < START_DUTY_THRESHOLD) {
    communicationFaultLatched = false;
    consecutiveCommunicationFailures = 0;
    odriveCommunicationValid = true;
  }

  const bool driveFault =
    odriveHasError || odriveFaultLatched || speedDropFaultLatched ||
    powerIndexFaultLatched;
  const bool seriousFault = communicationFaultLatched || driveFault;

  if (seriousFault) updateBlueSpeedFeedback(0.0f, false);
  if (driveFault && odriveDriveEnabled) disableODriveSafely();
  setCarveraAlarm(seriousFault);

  if (nowMs - lastPrintTimeMs >= PRINT_INTERVAL_MS) {
    lastPrintTimeMs = nowMs;
    printStatus();
  }
}
