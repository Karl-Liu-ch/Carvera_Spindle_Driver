/*
  Carvera PWM -> ESP32 -> SN65HVD230 -> ODrive CANSimple

  Target board: ESP32 Dev Module (classic ESP32)

  ESP32 wiring:
    GPIO2  <- Carvera spindle PWM through a 5 V to 3.3 V level shifter
    GPIO17 -> SN65HVD230 D (TXD)
    GPIO16 <- SN65HVD230 R (RXD)
    GPIO18 -> Carvera Blue speed feedback through suitable level shifting
    GPIO19 -> Carvera Red alarm through suitable level shifting
    3V3    -> SN65HVD230 VCC
    GND    -> SN65HVD230 GND and common ODrive/Carvera ground
    CANH/CANL connect the transceiver to the ODrive CAN bus.

  Fit 120 ohm termination between CANH and CANL at each physical end of the
  bus (two terminators total). ESP32 GPIO is not 5 V tolerant.

  ODrive requirements:
    - CANSimple protocol
    - node ID equal to ODRIVE_NODE_ID below
    - classic CAN at CAN_BITRATE (500 kbit/s by default)
    - velocity control commissioned and saved on the ODrive

  The sketch uses the ESP32's built-in TWAI (classic CAN) controller. The
  SN65HVD230 is the required 3.3 V physical-layer transceiver.
*/

#include <Arduino.h>
#include <driver/twai.h>
#include <math.h>
#include <string.h>

// Hardware
static constexpr uint8_t PWM_INPUT_PIN = 2;
static constexpr uint8_t CAN_RX_PIN = 16;
static constexpr uint8_t CAN_TX_PIN = 17;
static constexpr uint8_t BLUE_SPEED_FEEDBACK_PIN = 18;
static constexpr uint8_t RED_ALARM_SIGNAL_PIN = 19;

// ODrive CANSimple configuration. CAN_BITRATE currently supports 125000,
// 250000, 500000 or 1000000 in configureCAN().
static constexpr uint8_t ODRIVE_NODE_ID = 0;
static constexpr uint32_t CAN_BITRATE = 500000;

// spindle RPM = motor RPM * GEAR_RATIO. The Carvera speed feedback in the
// original sketch reports motor speed directly, so GEAR_RATIO is retained for
// documentation/possible future use but not applied to the feedback output.
static constexpr float GEAR_RATIO = 1.635f;
static constexpr float MAX_SPINDLE_RPM = 13000.0f;
static constexpr float MAX_MOTOR_RPM = 12500.0f;
static constexpr float DIRECTION_SIGN = -1.0f;
static constexpr float SPEED_FEEDBACK_PPR = 12.0f;
static constexpr float START_DUTY_THRESHOLD = 0.02f;
static constexpr float FILTER_ALPHA = 0.20f;

static constexpr uint32_t MIN_PWM_PERIOD_US = 700;
static constexpr uint32_t MAX_PWM_PERIOD_US = 1500;
static constexpr uint32_t PWM_TIMEOUT_US = 100000;
static constexpr uint32_t COMMAND_INTERVAL_MS = 20;
static constexpr uint32_t STATUS_REQUEST_INTERVAL_MS = 100;
static constexpr uint32_t PRINT_INTERVAL_MS = 500;
static constexpr uint32_t IDLE_DELAY_MS = 500;
static constexpr uint32_t ERROR_CLEAR_RETRY_MS = 1000;
static constexpr uint32_t ODRIVE_TIMEOUT_MS = 500;
static constexpr unsigned long PC_BAUDRATE = 115200;

// ODrive CANSimple command IDs (11-bit identifier = node_id << 5 | cmd_id).
enum class ODriveCommand : uint8_t {
  HEARTBEAT = 0x01,
  GET_ERROR = 0x03,
  SET_AXIS_STATE = 0x07,
  GET_ENCODER_ESTIMATES = 0x09,
  SET_CONTROLLER_MODE = 0x0B,
  SET_INPUT_VEL = 0x0D,
  CLEAR_ERRORS = 0x18
};

static constexpr uint32_t AXIS_STATE_IDLE = 1;
static constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
static constexpr uint32_t CONTROL_MODE_VELOCITY_CONTROL = 2;
static constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1;

volatile uint32_t lastRiseTimeUs = 0;
volatile uint32_t measuredPeriodUs = 0;
volatile uint32_t measuredHighTimeUs = 0;
volatile uint32_t pendingHighTimeUs = 0;
volatile uint32_t lastValidSampleTimeUs = 0;
volatile bool pendingHighTimeReady = false;
volatile bool measurementReady = false;

enum class PWMReadResult : uint8_t {
  NEW_SAMPLE,
  NO_NEW_SAMPLE,
  TIMEOUT
};

float filteredDuty = 0.0f;
float targetMotorRpm = 0.0f;
float targetSpindleRpm = 0.0f;
float measuredMotorRpm = 0.0f;
float measuredSpindleRpm = 0.0f;
float measuredSpeedPercent = 0.0f;
float speedFeedbackFrequencyHz = 0.0f;

bool pwmSignalValid = false;
bool canDriverReady = false;
bool odriveConnected = false;
bool odriveInClosedLoop = false;
bool odriveHasError = false;
bool odriveFaultLatched = false;
bool alarmActive = false;

uint8_t odriveState = 0;
uint32_t odriveAxisError = 0;
uint32_t odriveActiveErrors = 0;
uint32_t odriveDisarmReason = 0;

uint32_t lastHeartbeatTimeMs = 0;
uint32_t lastEncoderTimeMs = 0;
uint32_t lastCommandTimeMs = 0;
uint32_t lastStatusRequestTimeMs = 0;
uint32_t lastPrintTimeMs = 0;
uint32_t spindleOffStartTimeMs = 0;
uint32_t odriveErrorFirstSeenMs = 0;

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

bool transmitCan(ODriveCommand command, const uint8_t *data,
                 uint8_t length, bool remoteRequest = false)
{
  if (!canDriverReady) return false;

  twai_message_t message = {};
  message.identifier = makeCanId(command);
  message.extd = 0;
  message.rtr = remoteRequest ? 1 : 0;
  message.data_length_code = length;
  if (data != nullptr && length != 0) {
    memcpy(message.data, data, length);
  }

  return twai_transmit(&message, pdMS_TO_TICKS(2)) == ESP_OK;
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
  writeUint32LE(payload + 4, INPUT_MODE_PASSTHROUGH);
  return transmitCan(ODriveCommand::SET_CONTROLLER_MODE,
                     payload, sizeof(payload));
}

bool sendVelocity(float turnsPerSecond)
{
  uint8_t payload[8] = {};
  writeFloatLE(payload, turnsPerSecond);
  writeFloatLE(payload + 4, 0.0f);  // torque feed-forward
  return transmitCan(ODriveCommand::SET_INPUT_VEL, payload, sizeof(payload));
}

void requestODriveStatus()
{
  // RTR requests work whether or not these messages are configured as cyclic.
  transmitCan(ODriveCommand::HEARTBEAT, nullptr, 0, true);
  transmitCan(ODriveCommand::GET_ERROR, nullptr, 0, true);
  transmitCan(ODriveCommand::GET_ENCODER_ESTIMATES, nullptr, 0, true);
}

void clearODriveErrors()
{
  // Clear_Errors has one byte (Identify); zero means do not identify.
  const uint8_t identify = 0;
  transmitCan(ODriveCommand::CLEAR_ERRORS, &identify, 1);
}

void processReceivedCan()
{
  twai_message_t message = {};

  while (twai_receive(&message, 0) == ESP_OK) {
    if (message.extd || message.rtr) continue;
    if ((message.identifier >> 5) != ODRIVE_NODE_ID) continue;

    const uint8_t commandId = message.identifier & 0x1F;
    const uint32_t nowMs = millis();

    if (commandId == static_cast<uint8_t>(ODriveCommand::HEARTBEAT) &&
        message.data_length_code >= 5) {
      odriveAxisError = readUint32LE(message.data);
      odriveState = message.data[4];
      odriveInClosedLoop =
        odriveState == AXIS_STATE_CLOSED_LOOP_CONTROL;
      lastHeartbeatTimeMs = nowMs;
    }
    else if (commandId == static_cast<uint8_t>(ODriveCommand::GET_ERROR) &&
             message.data_length_code >= 8) {
      odriveActiveErrors = readUint32LE(message.data);
      odriveDisarmReason = readUint32LE(message.data + 4);
    }
    else if (commandId ==
               static_cast<uint8_t>(ODriveCommand::GET_ENCODER_ESTIMATES) &&
             message.data_length_code >= 8) {
      const float velocityTurnsPerSecond = readFloatLE(message.data + 4);
      if (isfinite(velocityTurnsPerSecond)) {
        measuredMotorRpm = velocityTurnsPerSecond * 60.0f;
        measuredSpindleRpm = measuredMotorRpm;
        measuredSpeedPercent =
          fabsf(measuredMotorRpm) / MAX_MOTOR_RPM * 100.0f;
        lastEncoderTimeMs = nowMs;
      }
    }
  }
}

void setCarveraAlarm(bool active)
{
  alarmActive = active;
  // Carvera alarm is active LOW.
  digitalWrite(RED_ALARM_SIGNAL_PIN, active ? LOW : HIGH);
}

void updateBlueSpeedFeedback(float motorVelocityTurnsPerSecond, bool valid)
{
  speedFeedbackFrequencyHz =
    fabsf(motorVelocityTurnsPerSecond) * SPEED_FEEDBACK_PPR;

  if (!valid || !isfinite(speedFeedbackFrequencyHz) ||
      speedFeedbackFrequencyHz < 0.5f) {
    noTone(BLUE_SPEED_FEEDBACK_PIN);
    digitalWrite(BLUE_SPEED_FEEDBACK_PIN, LOW);
    speedFeedbackFrequencyHz = 0.0f;
    return;
  }

  tone(BLUE_SPEED_FEEDBACK_PIN,
       static_cast<unsigned int>(lroundf(speedFeedbackFrequencyHz)));
}

void updateODriveStatus(uint32_t nowMs)
{
  odriveConnected = lastHeartbeatTimeMs != 0 &&
                    nowMs - lastHeartbeatTimeMs <= ODRIVE_TIMEOUT_MS;

  if (!odriveConnected || lastEncoderTimeMs == 0 ||
      nowMs - lastEncoderTimeMs > ODRIVE_TIMEOUT_MS) {
    measuredMotorRpm = 0.0f;
    measuredSpindleRpm = 0.0f;
    measuredSpeedPercent = 0.0f;
  }

  odriveHasError = odriveConnected &&
    (odriveAxisError != 0 || odriveActiveErrors != 0 ||
     odriveDisarmReason != 0);

  updateBlueSpeedFeedback(
    measuredMotorRpm / 60.0f,
    odriveConnected && lastEncoderTimeMs != 0 &&
      nowMs - lastEncoderTimeMs <= ODRIVE_TIMEOUT_MS
  );

  const bool normalState = odriveState == AXIS_STATE_IDLE ||
                           odriveState == AXIS_STATE_CLOSED_LOOP_CONTROL;
  setCarveraAlarm(odriveHasError || odriveFaultLatched ||
                  !odriveConnected || !normalState);
}

void IRAM_ATTR pwmEdgeISR()
{
  const uint32_t nowUs = micros();
  const bool inputHigh = digitalRead(PWM_INPUT_PIN);

  if (inputHigh) {
    if (lastRiseTimeUs != 0) {
      const uint32_t completedPeriodUs = nowUs - lastRiseTimeUs;
      if (pendingHighTimeReady &&
          completedPeriodUs >= MIN_PWM_PERIOD_US &&
          completedPeriodUs <= MAX_PWM_PERIOD_US &&
          pendingHighTimeUs <= completedPeriodUs) {
        measuredPeriodUs = completedPeriodUs;
        measuredHighTimeUs = pendingHighTimeUs;
        lastValidSampleTimeUs = nowUs;
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
  uint32_t validSampleTimeUs;
  bool newMeasurement;

  noInterrupts();
  highTimeUs = measuredHighTimeUs;
  periodUs = measuredPeriodUs;
  validSampleTimeUs = lastValidSampleTimeUs;
  newMeasurement = measurementReady;
  measurementReady = false;
  interrupts();

  if (validSampleTimeUs == 0 ||
      static_cast<uint32_t>(micros() - validSampleTimeUs) > PWM_TIMEOUT_US) {
    duty = 0.0f;
    return PWMReadResult::TIMEOUT;
  }

  if (!newMeasurement) return PWMReadResult::NO_NEW_SAMPLE;

  duty = constrain(static_cast<float>(highTimeUs) /
                   static_cast<float>(periodUs), 0.0f, 1.0f);
  return PWMReadResult::NEW_SAMPLE;
}

bool configureCAN()
{
  const twai_general_config_t generalConfig =
    TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(CAN_TX_PIN),
      static_cast<gpio_num_t>(CAN_RX_PIN),
      TWAI_MODE_NORMAL
    );

  twai_timing_config_t timingConfig;
  if (CAN_BITRATE == 125000) {
    timingConfig = TWAI_TIMING_CONFIG_125KBITS();
  }
  else if (CAN_BITRATE == 250000) {
    timingConfig = TWAI_TIMING_CONFIG_250KBITS();
  }
  else if (CAN_BITRATE == 500000) {
    timingConfig = TWAI_TIMING_CONFIG_500KBITS();
  }
  else if (CAN_BITRATE == 1000000) {
    timingConfig = TWAI_TIMING_CONFIG_1MBITS();
  }
  else {
    Serial.println("ERROR: unsupported CAN_BITRATE");
    return false;
  }

  const twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&generalConfig, &timingConfig, &filterConfig) !=
      ESP_OK) {
    Serial.println("ERROR: TWAI driver installation failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("ERROR: TWAI driver start failed");
    return false;
  }
  return true;
}

void printStatus()
{
  Serial.print("PWM: ");
  Serial.print(pwmSignalValid ? "VALID" : "NO");
  Serial.print(" | duty: ");
  Serial.print(filteredDuty * 100.0f, 1);
  Serial.print("% | target motor: ");
  Serial.print(targetMotorRpm, 1);
  Serial.print(" rpm | target spindle: ");
  Serial.print(targetSpindleRpm, 1);
  Serial.print(" rpm | motor: ");
  Serial.print(measuredMotorRpm, 1);
  Serial.print(" rpm | spindle: ");
  Serial.print(measuredSpindleRpm, 1);
  Serial.print(" rpm | speed: ");
  Serial.print(measuredSpeedPercent, 1);
  Serial.print("% | Blue: ");
  Serial.print(speedFeedbackFrequencyHz, 1);
  Serial.print(" Hz | state: ");
  Serial.print(odriveState);
  Serial.print(" | axis error: 0x");
  Serial.print(odriveAxisError, HEX);
  Serial.print(" | active: 0x");
  Serial.print(odriveActiveErrors, HEX);
  Serial.print(" | disarm: 0x");
  Serial.print(odriveDisarmReason, HEX);
  Serial.print(" | CAN: ");
  Serial.print(odriveConnected ? "CONNECTED" : "LOST");
  Serial.print(" | Red: ");
  Serial.println(alarmActive ? "ALARM" : "NORMAL");
}

void setup()
{
  pinMode(PWM_INPUT_PIN, INPUT_PULLDOWN);
  pinMode(BLUE_SPEED_FEEDBACK_PIN, OUTPUT);
  pinMode(RED_ALARM_SIGNAL_PIN, OUTPUT);
  digitalWrite(BLUE_SPEED_FEEDBACK_PIN, LOW);
  setCarveraAlarm(false);

  Serial.begin(PC_BAUDRATE);
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmEdgeISR, CHANGE);
  canDriverReady = configureCAN();

  if (canDriverReady) {
    // These frames also act as beacons if ODrive CAN autobaud is enabled.
    sendVelocity(0.0f);
    sendControllerMode();
    sendAxisState(AXIS_STATE_IDLE);
    requestODriveStatus();
  }
  else {
    setCarveraAlarm(true);
  }

  spindleOffStartTimeMs = millis();
  Serial.println("ESP32 Carvera / ODrive CAN interface started");
}

void loop()
{
  processReceivedCan();

  float newDuty = 0.0f;
  const PWMReadResult pwmResult = readPWMDuty(newDuty);

  if (pwmResult == PWMReadResult::NEW_SAMPLE) {
    pwmSignalValid = true;
    if (newDuty < START_DUTY_THRESHOLD) newDuty = 0.0f;
    filteredDuty += FILTER_ALPHA * (newDuty - filteredDuty);
    if (filteredDuty < START_DUTY_THRESHOLD) filteredDuty = 0.0f;
  }
  else if (pwmResult == PWMReadResult::TIMEOUT) {
    pwmSignalValid = false;
    filteredDuty = 0.0f;
  }

  targetSpindleRpm = filteredDuty * MAX_MOTOR_RPM;
  targetMotorRpm = constrain(targetSpindleRpm, 0.0f, MAX_MOTOR_RPM);
  const float targetVelocity =
    DIRECTION_SIGN * targetMotorRpm / 60.0f;
  const uint32_t nowMs = millis();

  if (canDriverReady &&
      nowMs - lastStatusRequestTimeMs >= STATUS_REQUEST_INTERVAL_MS) {
    lastStatusRequestTimeMs = nowMs;
    requestODriveStatus();
  }

  updateODriveStatus(nowMs);

  if (canDriverReady &&
      nowMs - lastCommandTimeMs >= COMMAND_INTERVAL_MS) {
    lastCommandTimeMs = nowMs;

    const bool runRequested =
      pwmSignalValid && filteredDuty >= START_DUTY_THRESHOLD &&
      odriveConnected && !odriveFaultLatched;

    if (runRequested) {
      if (!odriveInClosedLoop) {
        sendControllerMode();
        sendAxisState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        sendVelocity(0.0f);
      }
      else {
        sendVelocity(targetVelocity);
      }
      spindleOffStartTimeMs = nowMs;
    }
    else {
      sendVelocity(0.0f);
      if (odriveInClosedLoop &&
          nowMs - spindleOffStartTimeMs >= IDLE_DELAY_MS) {
        sendAxisState(AXIS_STATE_IDLE);
      }
    }
  }

  if (odriveHasError) {
    odriveFaultLatched = true;
    setCarveraAlarm(true);

    if (odriveErrorFirstSeenMs == 0) {
      odriveErrorFirstSeenMs = nowMs;
    }
    else if (nowMs - odriveErrorFirstSeenMs >= ERROR_CLEAR_RETRY_MS) {
      clearODriveErrors();
      sendVelocity(0.0f);
      sendAxisState(AXIS_STATE_IDLE);
      odriveErrorFirstSeenMs = nowMs;
    }
  }
  else {
    odriveErrorFirstSeenMs = 0;
    if (odriveFaultLatched &&
        (!pwmSignalValid || filteredDuty < START_DUTY_THRESHOLD)) {
      sendVelocity(0.0f);
      sendAxisState(AXIS_STATE_IDLE);
      odriveFaultLatched = false;
    }
  }

  // Re-evaluate the alarm after the fault-latch logic above.
  updateODriveStatus(nowMs);

  if (nowMs - lastPrintTimeMs >= PRINT_INTERVAL_MS) {
    lastPrintTimeMs = nowMs;
    printStatus();
  }
}
