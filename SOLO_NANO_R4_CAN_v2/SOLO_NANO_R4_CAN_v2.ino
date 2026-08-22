/*
  Carvera PWM -> Arduino Nano R4 -> SOLO PICO CANopen

  Wiring:
    D2  <- Carvera spindle PWM command, approximately 1 kHz
    D3  -> Carvera Blue speed feedback, 12 pulses/revolution
    D4  -> CAN transceiver TXD (Nano R4 native CAN TX)
    D5  <- CAN transceiver RXD (Nano R4 native CAN RX)
    D6  -> Carvera Red alarm, LOW on fault / high impedance when normal

  CAN transceiver CANH/CANL connect to SOLO CANH/CANL. SOLO, Nano R4,
  transceiver and Carvera must share GND. Terminate both physical ends of the
  CAN bus with 120 ohms.

  SOLO must already be commissioned and saved in Motion Monitor with CANopen
  node ID 1 and CAN bitrate 1 Mbit/s. The sketch restores the verified motor,
  Hall-feedback, controller-gain and acceleration settings after every Nano or
  SOLO restart.
*/

#include <Arduino.h>
// SOLOMotorControllers 5.5.0 omits Nano R4 from its native-CAN compile guard.
// The companion compatibility source file supplies the RA4M1 backend.
#if defined(ARDUINO_NANO_R4) && !defined(ARDUINO_MINIMA)
#define ARDUINO_MINIMA
#endif

#include <SOLOMotorControllersCanopenNative.h>
#include "pwm.h"
#include <math.h>

// Hardware.
static constexpr uint8_t PWM_INPUT_PIN = 2;
static constexpr uint8_t BLUE_SPEED_FEEDBACK_PIN = 3;
static constexpr uint8_t RED_ALARM_SIGNAL_PIN = 6;

// Spindle and motor values mirror ODRIVE_NANO_R4_CAN_v3_HALL.
static constexpr float GEAR_RATIO = 1.635f;
static constexpr float MAX_SPINDLE_RPM = 18000.0f;
static constexpr float MAX_MOTOR_RPM = 11500.0f;
static constexpr float SPEED_FEEDBACK_PPR = 12.0f;
static constexpr float START_DUTY_THRESHOLD = 0.002f;
static constexpr float FILTER_ALPHA = 0.20f;
static constexpr float IQ_FILTER_ALPHA = 0.10f;
static constexpr float BLUE_FEEDBACK_FILTER_ALPHA = 0.25f;

// SOLO-specific runtime configuration restored after every restart. These
// values are retained from SOLO_NANO_R4_CAN_v1's verified Motion Monitor setup.
static constexpr float SOLO_CURRENT_LIMIT_A = 8.2f;
static constexpr float SOLO_SPEED_KP = 0.15f;
static constexpr float SOLO_SPEED_KI = 0.005f;
static constexpr float SOLO_CURRENT_KP = 0.3105163f;
static constexpr float SOLO_CURRENT_KI = 0.02108f;
static constexpr float SOLO_ACCELERATION_RPS2 = 100.0f;
static constexpr float SOLO_DECELERATION_RPS2 = 20.0f;
// Limit regenerative current to 0.5 A. Confirm that the DC supply can absorb
// this current and verify bus voltage during worst-case deceleration.
static constexpr float SOLO_REGENERATION_CURRENT_LIMIT_A = 0.5f;

// Expected PWM input: approximately 1 kHz.
static constexpr uint32_t MIN_PWM_PERIOD_US = 700;
static constexpr uint32_t MAX_PWM_PERIOD_US = 1500;
static constexpr uint32_t PWM_EDGE_TIMEOUT_US = 100000;

// Timing and protection values mirror ODRIVE_NANO_R4_CAN_v3_HALL.
static constexpr uint32_t COMMUNICATION_SLOT_MS = 25;
static constexpr uint32_t SCHEDULED_ERROR_INTERVAL_MS = 500;
static constexpr uint32_t FIRST_ERROR_SLOT_OFFSET_MS = 525;
static constexpr uint32_t PRINT_INTERVAL_MS = 500;
static constexpr uint32_t ERROR_CLEAR_RETRY_MS = 1000;
// Match SOLO_NANO_R4_CAN_v1-MarsLab: keep sending a zero-speed reference and
// leave the drive ENABLED for 11 seconds before switching it to DISABLE.
static constexpr uint32_t DRIVE_DISABLE_DELAY_MS = 11000;
static constexpr uint32_t STARTUP_RETRY_INTERVAL_MS = 100;
static constexpr uint32_t TELEMETRY_TIMEOUT_MS = 350;
static constexpr uint8_t COMMUNICATION_FAILURE_ALARM_THRESHOLD = 8;
// Detect a failed start separately from SPEED_DROP, which is armed only after
// the motor has already reached its requested speed. A 5% PWM command equals
// approximately 900 spindle rpm. Once that threshold is crossed, the motor
// must leave the near-zero speed band within 2 seconds.
static constexpr float START_FAILURE_PWM_DUTY_THRESHOLD = 0.05f;
static constexpr float START_FAILURE_ZERO_SPEED_RPM = 20.0f;
static constexpr uint32_t START_FAILURE_CONFIRMATION_MS = 2000;
static constexpr float SPEED_DROP_ALARM_RATIO = 0.20f;
static constexpr float SPEED_DROP_MONITOR_MIN_RPM = 500.0f;
static constexpr float SPEED_TARGET_STEP_MIN_RPM = 200.0f;
static constexpr uint32_t SPEED_DROP_CONFIRMATION_MS = 2000;
static constexpr uint32_t POWER_INDEX_CONFIRMATION_MS = 1000;
static constexpr float POWER_INDEX_LIMIT = 52000.0f;

// USB serial and SOLO CANopen.
static constexpr unsigned long PC_BAUDRATE = 115200;
static constexpr uint8_t SOLO_CANOPEN_NODE_ID = 0;
static constexpr SOLOMotorControllers::CanbusBaudrate CANOPEN_BITRATE =
  SOLOMotorControllers::CanbusBaudrate::RATE_1000;
static constexpr long CANOPEN_RESPONSE_TIMEOUT_MS = 8;

SOLOMotorControllersCanopenNative *solo = nullptr;
PwmOut blueFeedbackPwm(BLUE_SPEED_FEEDBACK_PIN);

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
float measuredMotorRpm = 0.0f;
float motorIqMeasuredSignedA = 0.0f;
float filteredMotorIqSignedA = 0.0f;
float measuredMotorIqA = 0.0f;
float currentPowerIndex = 0.0f;
float speedFeedbackFrequencyHz = 0.0f;
float blueOutputFrequencyHz = 0.0f;
float filteredBlueFeedbackFrequencyHz = 0.0f;
float soloBusVoltageV = 0.0f;
float soloRegenerationCurrentLimitA = -1.0f;

bool pwmSignalValid = false;
bool soloCommunicationValid = false;
bool soloDriveEnabled = false;
bool soloHasError = false;
bool soloFaultLatched = false;
bool communicationFaultLatched = false;
bool startFailureFaultLatched = false;
bool startFailureMonitorArmed = false;
bool startFailureCommandAboveThreshold = false;
bool speedDropFaultLatched = false;
bool speedDropMonitorArmed = false;
bool motorIqValid = false;
bool motorIqFilterInitialized = false;
bool busTelemetryValid = false;
bool powerIndexFaultLatched = false;
bool soloStartupConfigured = false;
bool alarmActive = true;
bool blueFeedbackActive = false;
bool blueFeedbackPwmInitialized = false;
bool blueFeedbackFilterInitialized = false;
uint8_t consecutiveCommunicationFailures = 0;

long soloErrorRegister = 0;
long latchedSoloErrorRegister = 0;
bool soloErrorIncidentActive = false;
int soloLibraryError = 0;
int soloMotorDirection = -1;
bool errorClearPending = false;
uint32_t soloErrorReadCount = 0;
uint32_t errorClearRequestReadCount = 0;
uint32_t lastErrorClearAttemptTimeMs = 0;

uint32_t lastSpeedTimeMs = 0;
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
uint32_t nextStartupRetryTimeMs = 0;
uint32_t startFailureStartTimeMs = 0;
uint32_t speedDropStartTimeMs = 0;
uint32_t powerIndexStartTimeMs = 0;
float lastSpeedReferenceSentRpm = -1.0f;
float lastSpeedDropMonitorTargetRpm = 0.0f;

enum class AlarmReason : uint8_t {
  NONE,
  COMMUNICATION,
  SOLO_ACTIVE_ERROR,
  SOLO_FAULT_LATCHED,
  START_FAILURE,
  SPEED_DROP,
  POWER_INDEX
};

AlarmReason lastAlarmReason = AlarmReason::NONE;

enum class MotorSequenceState : uint8_t {
  STOPPED,
  RUNNING,
  STOPPING
};

MotorSequenceState motorSequenceState = MotorSequenceState::STOPPED;
uint8_t nextTelemetryRead = 0;
uint8_t startupCommandStage = 0;
bool startupSequenceComplete = false;
bool recoveryZeroConfirmed = false;
bool recoveryDisableConfirmed = false;
bool recoverySpeedConfirmed = false;
bool recoveryIqConfirmed = false;
bool recoveryErrorConfirmed = false;

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
// Carvera outputs
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
    case AlarmReason::SOLO_ACTIVE_ERROR: return "SOLO_ACTIVE_ERROR";
    case AlarmReason::SOLO_FAULT_LATCHED: return "SOLO_FAULT_LATCHED";
    case AlarmReason::START_FAILURE: return "START_FAILURE";
    case AlarmReason::SPEED_DROP: return "SPEED_DROP";
    case AlarmReason::POWER_INDEX: return "POWER_INDEX";
    default: return "NONE";
  }
}

const char *motorSequenceStateName(MotorSequenceState state)
{
  switch (state) {
    case MotorSequenceState::STOPPED: return "STOPPED";
    case MotorSequenceState::RUNNING: return "RUNNING";
    case MotorSequenceState::STOPPING: return "STOPPING";
    default: return "UNKNOWN";
  }
}

AlarmReason currentAlarmReason()
{
  if (communicationFaultLatched) return AlarmReason::COMMUNICATION;
  if (soloHasError) return AlarmReason::SOLO_ACTIVE_ERROR;
  if (soloFaultLatched) return AlarmReason::SOLO_FAULT_LATCHED;
  if (startFailureFaultLatched) return AlarmReason::START_FAILURE;
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
  // Open-drain/contact-closure behavior: float when normal and only sink the
  // Carvera alarm input to GND while a fault is active.
  digitalWrite(RED_ALARM_SIGNAL_PIN, LOW);
  pinMode(RED_ALARM_SIGNAL_PIN, active ? OUTPUT : INPUT);
}

// ---------------------------------------------------------------------------
// SOLO communication
// ---------------------------------------------------------------------------

void recordCommunicationSuccess()
{
  soloCommunicationValid = true;
  lastSuccessfulCommunicationMs = millis();
  consecutiveCommunicationFailures = 0;
}

void recordCommunicationFailure()
{
  soloCommunicationValid = false;
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

bool finishSoloTransaction(bool ok)
{
  // A valid CANopen response can still contain a value that does not match a
  // requested configuration.  That is a configuration/verification failure,
  // not a lost CAN transaction, so keep the link status healthy when the
  // library reports no communication error.
  if (soloLibraryError == 0) {
    recordCommunicationSuccess();
    return ok;
  }
  recordCommunicationFailure();
  return false;
}

bool sendSoloSpeed(float rpm)
{
  const long commandRpm = static_cast<long>(
    lroundf(constrain(rpm, 0.0f, MAX_MOTOR_RPM))
  );
  soloLibraryError = 0;
  const bool ok = solo->SetSpeedReference(commandRpm, soloLibraryError);
  if (finishSoloTransaction(ok)) {
    lastSpeedReferenceSentRpm = static_cast<float>(commandRpm);
    if (communicationFaultLatched && commandRpm == 0) {
      recoveryZeroConfirmed = true;
    }
    return true;
  }
  return false;
}

bool setSoloClockwiseDirection();
bool verifySoloClockwiseDirection();

bool setSoloDriveEnabled(bool enabled)
{
  if (enabled && (soloHasError || soloFaultLatched)) {
    return false;
  }

  if (enabled) {
    // Match the MarsLab start order: before every DISABLED -> ENABLED
    // transition, write CLOCKWISE and verify it by readback. Active errors
    // remain manually cleared; do not copy MarsLab's automatic error clear.
    if (!setSoloClockwiseDirection() ||
        !verifySoloClockwiseDirection()) {
      return false;
    }
  }

  soloLibraryError = 0;
  const bool ok = solo->SetDriveDisableEnable(
    enabled ? SOLOMotorControllers::DisableEnable::ENABLE
            : SOLOMotorControllers::DisableEnable::DISABLE,
    soloLibraryError
  );
  if (finishSoloTransaction(ok)) {
    soloDriveEnabled = enabled;
    if (communicationFaultLatched && !enabled) {
      recoveryDisableConfirmed = true;
    }
    return true;
  }
  return false;
}

bool setSoloClockwiseDirection()
{
  soloLibraryError = 0;
  return finishSoloTransaction(
    solo->SetMotorDirection(
      SOLOMotorControllers::Direction::CLOCKWISE, soloLibraryError)
  );
}

bool verifySoloClockwiseDirection()
{
  soloLibraryError = 0;
  const SOLOMotorControllers::Direction direction =
    solo->GetMotorDirection(soloLibraryError);
  soloMotorDirection = static_cast<int>(direction);
  return finishSoloTransaction(
    soloLibraryError == 0 &&
    direction == SOLOMotorControllers::Direction::CLOCKWISE
  );
}

bool pollSoloSpeed(uint32_t nowMs)
{
  soloLibraryError = 0;
  const long rpm = solo->GetSpeedFeedback(soloLibraryError);
  const bool valid = soloLibraryError == 0;
  if (valid) {
    // SOLO feedback is signed according to its direction convention.
    measuredMotorRpm = fabsf(static_cast<float>(rpm));
    lastSpeedTimeMs = nowMs;
    if (communicationFaultLatched) recoverySpeedConfirmed = true;
    recordCommunicationSuccess();
  }
  else {
    recordCommunicationFailure();
  }

  updateBlueSpeedFeedback(
    measuredMotorRpm,
    valid && !communicationFailureLimitExceeded() &&
      !soloHasError && !soloFaultLatched
  );
  return valid;
}

bool pollSoloIq(uint32_t nowMs)
{
  soloLibraryError = 0;
  const float iqA = solo->GetQuadratureCurrentIqFeedback(soloLibraryError);
  const bool valid = soloLibraryError == 0 && isfinite(iqA);
  if (valid) {
    motorIqMeasuredSignedA = iqA;
    if (!motorIqFilterInitialized) {
      filteredMotorIqSignedA = iqA;
      motorIqFilterInitialized = true;
    }
    else {
      filteredMotorIqSignedA += IQ_FILTER_ALPHA *
        (iqA - filteredMotorIqSignedA);
    }
    // Filter signed Iq before taking its magnitude to avoid rectifying ripple.
    measuredMotorIqA = fabsf(filteredMotorIqSignedA);
    motorIqValid = true;
    lastIqTimeMs = nowMs;
    if (communicationFaultLatched) recoveryIqConfirmed = true;
    recordCommunicationSuccess();
  }
  else {
    motorIqValid = false;
    recordCommunicationFailure();
  }
  return valid;
}

bool pollSoloBusVoltage(uint32_t nowMs)
{
  soloLibraryError = 0;
  const float busVoltage = solo->GetBusVoltage(soloLibraryError);
  const bool valid = soloLibraryError == 0 && isfinite(busVoltage);
  if (valid) {
    soloBusVoltageV = busVoltage;
    busTelemetryValid = true;
    lastBusTelemetryTimeMs = nowMs;
    recordCommunicationSuccess();
  }
  else {
    busTelemetryValid = false;
    recordCommunicationFailure();
  }
  return valid;
}

bool pollSoloErrors(uint32_t nowMs)
{
  soloLibraryError = 0;
  const long errorRegister = solo->GetErrorRegister(soloLibraryError);
  if (soloLibraryError != 0) {
    recordCommunicationFailure();
    return false;
  }

  soloErrorRegister = errorRegister;
  soloHasError = errorRegister != 0;
  if (errorRegister != 0) {
    if (!soloErrorIncidentActive) {
      latchedSoloErrorRegister = errorRegister;
      soloErrorIncidentActive = true;
      errorClearPending = true;
      lastErrorClearAttemptTimeMs = 0;
      Serial.print("SOLO error recorded; automatic clear pending: 0x");
      Serial.println(
        static_cast<unsigned long>(latchedSoloErrorRegister), HEX);
    }
    else {
      // Preserve every error bit observed during the current incident.
      latchedSoloErrorRegister |= errorRegister;
    }
  }
  lastErrorMessageTimeMs = nowMs;
  ++soloErrorReadCount;
  if (communicationFaultLatched) recoveryErrorConfirmed = true;
  recordCommunicationSuccess();
  return true;
}

bool requestSoloErrorClear()
{
  soloLibraryError = 0;
  return finishSoloTransaction(
    solo->OverwriteErrorRegister(soloLibraryError)
  );
}

void requestControlledStop(uint32_t nowMs)
{
  (void)nowMs;
  if (motorSequenceState == MotorSequenceState::STOPPING ||
      motorSequenceState == MotorSequenceState::STOPPED) {
    return;
  }

  if (!soloDriveEnabled) {
    motorSequenceState = MotorSequenceState::STOPPED;
    return;
  }

  motorSequenceState = MotorSequenceState::STOPPING;
}

void executeControlledStop(uint32_t nowMs)
{
  requestControlledStop(nowMs);

  if (motorSequenceState == MotorSequenceState::STOPPED) {
    // With the power stage already disabled, a zero reference cannot create
    // regenerative braking and leaves the next start in a known state.
    if (lastSpeedReferenceSentRpm != 0.0f ||
        (communicationFaultLatched && !recoveryZeroConfirmed)) {
      sendSoloSpeed(0.0f);
    }
    else if (communicationFaultLatched && !recoveryDisableConfirmed) {
      setSoloDriveEnabled(false);
    }
    return;
  }

  // MarsLab stop order: command zero on every command slot, then release the
  // drive only after 11 seconds have elapsed since the last run request.
  sendSoloSpeed(0.0f);
  if (soloDriveEnabled &&
      nowMs - lastRunRequestedTimeMs >= DRIVE_DISABLE_DELAY_MS) {
    if (setSoloDriveEnabled(false)) {
      motorSequenceState = MotorSequenceState::STOPPED;
    }
  }
  else if (!soloDriveEnabled) {
    motorSequenceState = MotorSequenceState::STOPPED;
  }
}

bool runSoloStartupStage(uint32_t nowMs)
{
  if (solo == nullptr) return false;

  soloLibraryError = 0;
  bool ok = false;

  switch (startupCommandStage) {
    case 0:
      ok = solo->CommunicationIsWorking(soloLibraryError);
      break;
    case 1:
      ok = solo->SetDriveDisableEnable(
        SOLOMotorControllers::DisableEnable::DISABLE, soloLibraryError);
      if (ok && soloLibraryError == 0) soloDriveEnabled = false;
      break;
    case 2:
      ok = solo->GetDriveDisableEnable(soloLibraryError) ==
        SOLOMotorControllers::DisableEnable::DISABLE;
      break;
    case 3:
      ok = solo->SetSpeedReference(0, soloLibraryError);
      if (ok && soloLibraryError == 0) lastSpeedReferenceSentRpm = 0.0f;
      break;
    case 4:
      ok = solo->GetSpeedReference(soloLibraryError) == 0;
      break;
    case 5:
      ok = solo->SetCommandMode(
        SOLOMotorControllers::CommandMode::DIGITAL, soloLibraryError);
      break;
    case 6:
      ok = solo->SetMotorType(
        SOLOMotorControllers::MotorType::BLDC_PMSM, soloLibraryError);
      break;
    case 7:
      ok = solo->SetFeedbackControlMode(
        SOLOMotorControllers::FeedbackControlMode::HALL_SENSORS,
        soloLibraryError);
      break;
    case 8:
      ok = solo->SetControlMode(
        SOLOMotorControllers::ControlMode::SPEED_MODE, soloLibraryError);
      break;
    case 9:
      ok = solo->SetCurrentLimit(SOLO_CURRENT_LIMIT_A, soloLibraryError);
      break;
    case 10:
      ok = solo->SetCurrentControllerKp(
        SOLO_CURRENT_KP, soloLibraryError);
      break;
    case 11:
      ok = solo->SetCurrentControllerKi(
        SOLO_CURRENT_KI, soloLibraryError);
      break;
    case 12:
      ok = solo->SetSpeedControllerKp(SOLO_SPEED_KP, soloLibraryError);
      break;
    case 13:
      ok = solo->SetSpeedControllerKi(SOLO_SPEED_KI, soloLibraryError);
      break;
    case 14:
      ok = solo->SetSpeedAccelerationValue(
        SOLO_ACCELERATION_RPS2, soloLibraryError);
      break;
    case 15:
      ok = solo->SetSpeedDecelerationValue(
        SOLO_DECELERATION_RPS2, soloLibraryError);
      break;
    case 16:
      ok = solo->SetRegenerationCurrentLimit(
        SOLO_REGENERATION_CURRENT_LIMIT_A, soloLibraryError);
      if (ok && soloLibraryError == 0) {
        soloRegenerationCurrentLimitA = SOLO_REGENERATION_CURRENT_LIMIT_A;
      }
      break;
    case 17:
      ok = solo->SetMotorDirection(
        SOLOMotorControllers::Direction::CLOCKWISE, soloLibraryError);
      break;
    case 18: {
      const SOLOMotorControllers::Direction direction =
        solo->GetMotorDirection(soloLibraryError);
      soloMotorDirection = static_cast<int>(direction);
      ok = soloLibraryError == 0 &&
        direction == SOLOMotorControllers::Direction::CLOCKWISE;
      break;
    }
    default:
      ok = pollSoloErrors(nowMs);
      if (ok) startupSequenceComplete = true;
      return startupSequenceComplete && !soloHasError;
  }

  if (!finishSoloTransaction(ok)) {
    static uint32_t lastStartupFailurePrintMs = 0;
    if (lastStartupFailurePrintMs == 0 ||
        nowMs - lastStartupFailurePrintMs >= 1000) {
      lastStartupFailurePrintMs = nowMs;
      Serial.print("SOLO startup stage ");
      Serial.print(startupCommandStage);
      Serial.print(soloLibraryError == 0
        ? " verification mismatch"
        : " communication error: ");
      if (soloLibraryError != 0) Serial.print(soloLibraryError);
      Serial.println();
    }
    return false;
  }

  ++startupCommandStage;
  return false;
}

bool deadlineReached(uint32_t nowMs, uint32_t deadlineMs)
{
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

void executeScheduledSpeedCommand(uint32_t nowMs)
{
  const bool safeToRun =
    pwmSignalValid &&
    !soloHasError &&
    !soloFaultLatched &&
    !startFailureFaultLatched &&
    !speedDropFaultLatched &&
    !powerIndexFaultLatched &&
    soloStartupConfigured &&
    !communicationFaultLatched;
  const bool nonzeroCommand = targetMotorRpm > 0.0f;

  if (!safeToRun || !nonzeroCommand) {
    executeControlledStop(nowMs);
    return;
  }

  // Match MarsLab behavior: a returning normal PWM request during the
  // 11-second stop delay resumes speed immediately while the drive is still
  // ENABLED. Faults cannot reach this branch because safeToRun remains false.
  if (motorSequenceState == MotorSequenceState::STOPPING) {
    motorSequenceState = MotorSequenceState::RUNNING;
    sendSoloSpeed(targetMotorRpm);
    lastRunRequestedTimeMs = nowMs;
    return;
  }

  switch (motorSequenceState) {
    case MotorSequenceState::STOPPED:
      // setSoloDriveEnabled(true) follows MarsLab's start order internally:
      // set direction -> verify direction -> ENABLE. The first nonzero speed
      // reference is intentionally deferred to the next command slot.
      if (setSoloDriveEnabled(true)) {
        motorSequenceState = MotorSequenceState::RUNNING;
      }
      break;

    case MotorSequenceState::RUNNING:
      sendSoloSpeed(targetMotorRpm);
      lastRunRequestedTimeMs = nowMs;
      break;

    default:
      break;
  }
}

void runCommunicationSchedule(uint32_t nowMs)
{
  if (!deadlineReached(nowMs, nextCommunicationSlotTimeMs)) return;

  // Run at most one SOLO CANopen transaction per slot. Skip expired slots
  // instead of issuing a burst when the loop is late.
  const uint32_t slotsLate =
    (nowMs - nextCommunicationSlotTimeMs) / COMMUNICATION_SLOT_MS;
  communicationSlotIndex += slotsLate;
  nextCommunicationSlotTimeMs +=
    (slotsLate + 1U) * COMMUNICATION_SLOT_MS;
  const uint32_t slotIndex = communicationSlotIndex++;

  // Preserve the error register in pollSoloErrors() before attempting a clear.
  // Retry no faster than once per second and require a newer zero register read
  // before releasing the local fault latch.
  if (soloHasError && errorClearPending &&
      (lastErrorClearAttemptTimeMs == 0 ||
       nowMs - lastErrorClearAttemptTimeMs >= ERROR_CLEAR_RETRY_MS)) {
    lastErrorClearAttemptTimeMs = nowMs;
    if (requestSoloErrorClear()) {
      errorClearRequestReadCount = soloErrorReadCount;
      Serial.print("Automatic SOLO error clear sent | saved errors: 0x");
      Serial.println(
        static_cast<unsigned long>(latchedSoloErrorRegister), HEX);
    }
    else {
      Serial.println("Automatic SOLO error-clear request failed; will retry");
    }
    return;
  }

  if (!soloStartupConfigured) {
    if (deadlineReached(nowMs, nextStartupRetryTimeMs)) {
      nextStartupRetryTimeMs = nowMs + STARTUP_RETRY_INTERVAL_MS;
      if (runSoloStartupStage(nowMs)) {
        soloStartupConfigured = true;
        Serial.println("SOLO startup configuration verified");
      }
    }
    return;
  }

  if ((slotIndex & 1U) == 0U) {
    executeScheduledSpeedCommand(nowMs);
    return;
  }

  if (deadlineReached(nowMs, nextScheduledErrorTimeMs)) {
    pollSoloErrors(nowMs);
    const uint32_t elapsedIntervals =
      (nowMs - nextScheduledErrorTimeMs) /
        SCHEDULED_ERROR_INTERVAL_MS + 1U;
    nextScheduledErrorTimeMs +=
      elapsedIntervals * SCHEDULED_ERROR_INTERVAL_MS;
    return;
  }

  if (nextTelemetryRead == 0) pollSoloIq(nowMs);
  else if (nextTelemetryRead == 1) pollSoloSpeed(nowMs);
  else pollSoloBusVoltage(nowMs);
  nextTelemetryRead = (nextTelemetryRead + 1U) % 3U;
}

// ---------------------------------------------------------------------------
// Protection logic
// ---------------------------------------------------------------------------

void updateStartFailureMonitor(uint32_t nowMs)
{
  const bool commandAboveThreshold =
    pwmSignalValid && filteredDuty >= START_FAILURE_PWM_DUTY_THRESHOLD;

  // Arm only on a low-to-high command transition. Once real motion has been
  // observed, this startup-only monitor is finished until PWM returns low.
  if (commandAboveThreshold && !startFailureCommandAboveThreshold) {
    startFailureMonitorArmed = true;
    startFailureStartTimeMs = 0;
  }
  startFailureCommandAboveThreshold = commandAboveThreshold;

  if (!commandAboveThreshold) {
    startFailureMonitorArmed = false;
    startFailureStartTimeMs = 0;
    return;
  }

  if (startFailureFaultLatched || !startFailureMonitorArmed) return;

  const bool speedTelemetryValid = telemetryIsFresh(lastSpeedTimeMs, nowMs);
  if (!soloDriveEnabled || !soloCommunicationValid ||
      !speedTelemetryValid || soloHasError || soloFaultLatched ||
      communicationFaultLatched) {
    // Do not charge SOLO startup or invalid telemetry time against the motor,
    // but keep the start attempt armed until conditions are valid.
    startFailureStartTimeMs = 0;
    return;
  }

  if (measuredMotorRpm > START_FAILURE_ZERO_SPEED_RPM) {
    startFailureMonitorArmed = false;
    startFailureStartTimeMs = 0;
    return;
  }

  if (startFailureStartTimeMs == 0) {
    startFailureStartTimeMs = nowMs;
  }
  else if (nowMs - startFailureStartTimeMs >=
           START_FAILURE_CONFIRMATION_MS) {
    startFailureFaultLatched = true;
    startFailureMonitorArmed = false;
    Serial.println(
      "PWM start command present but motor stayed at 0 rpm for 2 s: "
      "fault latched");
  }
}

void updateSpeedDropMonitor(uint32_t nowMs)
{
  if (speedDropFaultLatched) return;

  if (targetMotorRpm < SPEED_DROP_MONITOR_MIN_RPM ||
      filteredDuty < START_DUTY_THRESHOLD) {
    speedDropMonitorArmed = false;
    speedDropStartTimeMs = 0;
    lastSpeedDropMonitorTargetRpm = targetMotorRpm;
    return;
  }

  if (!soloDriveEnabled || !soloCommunicationValid ||
      soloHasError || soloFaultLatched || communicationFaultLatched) {
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

  if (!speedDropMonitorArmed) return;

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
  if (powerIndexFaultLatched) return;

  if (!motorIqValid || !soloDriveEnabled ||
      targetMotorRpm < SPEED_DROP_MONITOR_MIN_RPM ||
      filteredDuty < START_DUTY_THRESHOLD ||
      soloHasError || soloFaultLatched || communicationFaultLatched) {
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
  Serial.print(" rpm | Iq measured: ");
  if (motorIqValid) {
    Serial.print(motorIqMeasuredSignedA, 3);
    Serial.print(" A | Iq filtered: ");
    Serial.print(filteredMotorIqSignedA, 3);
    Serial.print(" A | power index: ");
    Serial.print(currentPowerIndex, 0);
  }
  else {
    Serial.print(
      "INVALID | Iq filtered: INVALID | power index: INVALID");
  }
  Serial.print(" | Vbus: ");
  if (busTelemetryValid) {
    Serial.print(soloBusVoltageV, 2);
    Serial.print(" V");
  }
  else {
    Serial.print("INVALID");
  }
  Serial.print(" | Blue: ");
  Serial.print(speedFeedbackFrequencyHz, 1);
  Serial.print(" Hz requested / ");
  Serial.print(blueOutputFrequencyHz, 1);
  Serial.print(" Hz output | Blue restarts: ");
  Serial.print(blueFeedbackRestartCount);
  Serial.print(" | Blue stops: ");
  Serial.print(blueFeedbackStopCount);
  Serial.print(" | drive: ");
  Serial.print(soloDriveEnabled ? "ENABLED" : "DISABLED");
  Serial.print(" | motor sequence: ");
  Serial.print(motorSequenceStateName(motorSequenceState));
  Serial.print(" | regen limit: ");
  Serial.print(soloRegenerationCurrentLimitA, 2);
  Serial.print(" A");
  Serial.print(" | start monitor: ");
  if (startFailureFaultLatched) Serial.print("FAULT");
  else if (startFailureStartTimeMs != 0) Serial.print("TIMING");
  else Serial.print(startFailureMonitorArmed ? "ARMED" : "WAIT");
  Serial.print(" | speed monitor: ");
  if (speedDropFaultLatched) Serial.print("FAULT");
  else if (speedDropStartTimeMs != 0) Serial.print("TIMING");
  else Serial.print(speedDropMonitorArmed ? "ARMED" : "WAIT");
  Serial.print(" | power monitor: ");
  if (powerIndexFaultLatched) Serial.print("FAULT");
  else if (powerIndexStartTimeMs != 0) Serial.print("TIMING");
  else Serial.print("NORMAL");
  Serial.print(" | SOLO errors: 0x");
  Serial.print(static_cast<unsigned long>(soloErrorRegister), HEX);
  Serial.print(" | last SOLO errors: 0x");
  Serial.print(static_cast<unsigned long>(latchedSoloErrorRegister), HEX);
  Serial.print(" | library error: ");
  Serial.print(soloLibraryError);
  Serial.print(" | direction: ");
  if (soloMotorDirection ==
      static_cast<int>(SOLOMotorControllers::Direction::COUNTERCLOCKWISE)) {
    Serial.print("CCW");
  }
  else if (soloMotorDirection ==
           static_cast<int>(SOLOMotorControllers::Direction::CLOCKWISE)) {
    Serial.print("CW");
  }
  else {
    Serial.print("UNKNOWN");
  }
  Serial.print(" | error clear: ");
  Serial.print(errorClearPending ? "PENDING" : "IDLE");
  Serial.print(" | CANopen: ");
  Serial.print(soloCommunicationValid ? "CONNECTED" : "LOST");
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

  if (compactCommand.equalsIgnoreCase("status")) {
    printStatus();
  }
  else if (compactCommand.equalsIgnoreCase("clear_errors")) {
    const uint32_t nowMs = millis();
    if (filteredDuty >= START_DUTY_THRESHOLD) {
      Serial.println(
        "Clear rejected: set the Carvera spindle command to zero first");
    }
    else if (solo == nullptr ||
             !telemetryIsFresh(lastSuccessfulCommunicationMs, nowMs)) {
      Serial.println("Clear rejected: SOLO CANopen is not available");
    }
    else if (requestSoloErrorClear()) {
      errorClearPending = true;
      errorClearRequestReadCount = soloErrorReadCount;
      lastErrorClearAttemptTimeMs = nowMs;
      requestControlledStop(nowMs);
      Serial.println(
        "SOLO error clear sent; waiting for zero error-register feedback");
    }
    else {
      Serial.println("SOLO error-clear request failed");
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

  solo = new SOLOMotorControllersCanopenNative(
    SOLO_CANOPEN_NODE_ID,
    CANOPEN_BITRATE,
    CAN,
    CANOPEN_RESPONSE_TIMEOUT_MS
  );

  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmEdgeISR, CHANGE);
  Serial.println("Nano R4 Carvera / SOLO CANopen v2 interface startup");

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
  motorIqValid = telemetryIsFresh(lastIqTimeMs, nowMs);
  busTelemetryValid = telemetryIsFresh(lastBusTelemetryTimeMs, nowMs);

  runCommunicationSchedule(nowMs);
  updateStartFailureMonitor(nowMs);
  updateSpeedDropMonitor(nowMs);
  updatePowerIndexMonitor(nowMs);

  if (soloHasError) soloFaultLatched = true;

  // Automatic clearing never permits an automatic restart. Require a newer
  // zero error-register read, zero PWM, zero command and DISABLE before the
  // local fault latch is released.
  if (errorClearPending &&
      soloErrorReadCount > errorClearRequestReadCount &&
      !soloHasError && filteredDuty < START_DUTY_THRESHOLD) {
    if (motorSequenceState == MotorSequenceState::STOPPED &&
        !soloDriveEnabled && lastSpeedReferenceSentRpm == 0.0f) {
      soloFaultLatched = false;
      errorClearPending = false;
      errorClearRequestReadCount = 0;
      lastErrorClearAttemptTimeMs = 0;
      soloErrorIncidentActive = false;
      Serial.print("SOLO error clear confirmed; fault latch released");
      Serial.print(" | last SOLO errors: 0x");
      Serial.println(
        static_cast<unsigned long>(latchedSoloErrorRegister), HEX);
    }
  }

  if (startFailureFaultLatched &&
      filteredDuty < START_DUTY_THRESHOLD) {
    if (motorSequenceState == MotorSequenceState::STOPPED &&
        !soloDriveEnabled && lastSpeedReferenceSentRpm == 0.0f) {
      startFailureFaultLatched = false;
      startFailureMonitorArmed = false;
      startFailureCommandAboveThreshold = false;
      startFailureStartTimeMs = 0;
    }
  }

  if (speedDropFaultLatched && filteredDuty < START_DUTY_THRESHOLD) {
    if (motorSequenceState == MotorSequenceState::STOPPED &&
        !soloDriveEnabled && lastSpeedReferenceSentRpm == 0.0f) {
      speedDropFaultLatched = false;
      speedDropMonitorArmed = false;
      speedDropStartTimeMs = 0;
    }
  }

  if (powerIndexFaultLatched && filteredDuty < START_DUTY_THRESHOLD) {
    if (motorSequenceState == MotorSequenceState::STOPPED &&
        !soloDriveEnabled && lastSpeedReferenceSentRpm == 0.0f) {
      powerIndexFaultLatched = false;
      powerIndexStartTimeMs = 0;
      currentPowerIndex = 0.0f;
    }
  }

  if (communicationFailureLimitExceeded() && !communicationFaultLatched) {
    communicationFaultLatched = true;
    recoveryZeroConfirmed = false;
    recoveryDisableConfirmed = false;
    recoverySpeedConfirmed = false;
    recoveryIqConfirmed = false;
    recoveryErrorConfirmed = false;
  }

  if (communicationFaultLatched &&
      recoveryZeroConfirmed && recoveryDisableConfirmed &&
      recoverySpeedConfirmed && recoveryIqConfirmed &&
      recoveryErrorConfirmed &&
      motorSequenceState == MotorSequenceState::STOPPED &&
      !soloDriveEnabled &&
      lastSpeedReferenceSentRpm == 0.0f &&
      filteredDuty < START_DUTY_THRESHOLD) {
    communicationFaultLatched = false;
    consecutiveCommunicationFailures = 0;
    soloCommunicationValid = true;
  }

  const bool driveFault =
    soloHasError || soloFaultLatched || startFailureFaultLatched ||
    speedDropFaultLatched || powerIndexFaultLatched;
  const bool seriousFault = communicationFaultLatched || driveFault;

  if (seriousFault) updateBlueSpeedFeedback(0.0f, false);
  if (driveFault) requestControlledStop(nowMs);
  setCarveraAlarm(seriousFault);

  if (nowMs - lastPrintTimeMs >= PRINT_INTERVAL_MS) {
    lastPrintTimeMs = nowMs;
    printStatus();
  }
}
