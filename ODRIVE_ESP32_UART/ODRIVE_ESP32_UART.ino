/*
  Carvera PWM -> ESP32 -> ODrive UART

  Target board: ESP32 Dev Module (classic ESP32)

  ESP32 connections:
    GPIO2  <- Carvera spindle PWM command (~1 kHz), through a 5 V to
              3.3 V level shifter
    GPIO16 <- ODrive TX (UART2 RX)
    GPIO17 -> ODrive RX (UART2 TX)
    GPIO18 -> Blue speed feedback, 12 pulses/revolution
    GPIO19 -> Red alarm, LOW when alarm / HIGH when normal
    GND    -> Common ground between Carvera, ESP32 and ODrive

  Configuration:
    - Spindle speed is 1.653 times motor speed
    - PWM input controls spindle RPM
    - Maximum commanded spindle RPM: 13000
    - Maximum allowed motor RPM: 12500
    - Direction: CCW (DIRECTION_SIGN = -1.0)

  ESP32 GPIO is not 5 V tolerant. Level-shift every 5 V Carvera signal.
  ODrive UART uses 3.3 V logic; do not insert an RS-232 level converter.
  Use a suitable output level shifter if Carvera requires a 5 V HIGH level.
*/

#include <Arduino.h>
#include <ODriveUART.h>
#include <math.h>

static constexpr uint8_t PWM_INPUT_PIN = 2;
static constexpr uint8_t ODRIVE_RX_PIN = 16;
static constexpr uint8_t ODRIVE_TX_PIN = 17;
static constexpr uint8_t BLUE_SPEED_FEEDBACK_PIN = 18;
static constexpr uint8_t RED_ALARM_SIGNAL_PIN = 19;

// spindle RPM = motor RPM * GEAR_RATIO
static constexpr float GEAR_RATIO = 1.635f;

// These are independent command and safety limits. Neither is derived from
// the other; GEAR_RATIO is used only for motor/spindle speed conversion.
static constexpr float MAX_SPINDLE_RPM = 13000.0f;
static constexpr float MAX_MOTOR_RPM = 12500.0f;

static constexpr float DIRECTION_SIGN = -1.0f;  // CCW direction
static constexpr float SPEED_FEEDBACK_PPR = 12.0f;
static constexpr float START_DUTY_THRESHOLD = 0.02f;
// Light filtering reduces micros()/ISR timing jitter without making the
// spindle command noticeably slow.
static constexpr float FILTER_ALPHA = 0.20f;

static constexpr uint32_t MIN_PWM_PERIOD_US = 700;
static constexpr uint32_t MAX_PWM_PERIOD_US = 1500;
static constexpr uint32_t PWM_TIMEOUT_US = 100000;
static constexpr uint32_t COMMAND_INTERVAL_MS = 20;
static constexpr uint32_t STATUS_INTERVAL_MS = 100;
static constexpr uint32_t PRINT_INTERVAL_MS = 500;
static constexpr uint32_t IDLE_DELAY_MS = 500;
static constexpr unsigned long ODRIVE_BAUDRATE = 115200;
static constexpr unsigned long PC_BAUDRATE = 115200;

HardwareSerial odriveSerial(2);
ODriveUART odrive(odriveSerial);

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
bool odriveInClosedLoop = false;
bool odriveHasError = false;
bool odriveFaultLatched = false;
bool odriveUartConnected = false;
bool redAlarmHigh = false;

int odriveState = AXIS_STATE_UNDEFINED;
uint32_t odriveActiveErrors = 0;
uint32_t odriveDisarmReason = 0;

uint32_t lastCommandTimeMs = 0;
uint32_t lastStatusTimeMs = 0;
uint32_t spindleOffStartTimeMs = 0;
uint32_t odriveErrorFirstSeenMs = 0;

static constexpr uint32_t ERROR_CLEAR_RETRY_MS = 1000;

void IRAM_ATTR pwmEdgeISR()
{
  const uint32_t nowUs = micros();
  const bool inputHigh = digitalRead(PWM_INPUT_PIN);

  if (inputHigh) {
    if (lastRiseTimeUs != 0) {
      const uint32_t completedPeriodUs = nowUs - lastRiseTimeUs;

      // Commit period and high time together only when both belong to the
      // same completed PWM cycle.
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
  } else if (lastRiseTimeUs != 0) {
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

  if (!newMeasurement) {
    return PWMReadResult::NO_NEW_SAMPLE;
  }

  duty = constrain(
    static_cast<float>(highTimeUs) / static_cast<float>(periodUs),
    0.0f,
    1.0f
  );
  return PWMReadResult::NEW_SAMPLE;
}

void updateBlueSpeedFeedback(float motorVelocityTurnsPerSecond, bool valid)
{
  // Convert motor velocity to spindle velocity using gear ratio
  // const float spindleVelocityTurnsPerSecond = motorVelocityTurnsPerSecond * GEAR_RATIO;
  const float spindleVelocityTurnsPerSecond = motorVelocityTurnsPerSecond;

  speedFeedbackFrequencyHz =
    fabsf(spindleVelocityTurnsPerSecond) * SPEED_FEEDBACK_PPR;

  if (!valid || speedFeedbackFrequencyHz < 0.5f) {
    noTone(BLUE_SPEED_FEEDBACK_PIN);
    digitalWrite(BLUE_SPEED_FEEDBACK_PIN, LOW);
    speedFeedbackFrequencyHz = 0.0f;
    return;
  }

  tone(
    BLUE_SPEED_FEEDBACK_PIN,
    static_cast<unsigned int>(lroundf(speedFeedbackFrequencyHz))
  );
}

void updateODriveStatus()
{
  odriveState = odrive.getState();
  const float activeErrorsValue =
    odrive.getParameterAsFloat("axis0.active_errors");
  const float disarmReasonValue =
    odrive.getParameterAsFloat("axis0.disarm_reason");

  const bool activeErrorsValid =
    isfinite(activeErrorsValue) && activeErrorsValue >= 0.0f;
  const bool disarmReasonValid =
    isfinite(disarmReasonValue) && disarmReasonValue >= 0.0f;

  odriveActiveErrors = activeErrorsValid
    ? static_cast<uint32_t>(activeErrorsValue)
    : UINT32_MAX;
  odriveDisarmReason = disarmReasonValid
    ? static_cast<uint32_t>(disarmReasonValue)
    : UINT32_MAX;

  odriveUartConnected = odriveState != AXIS_STATE_UNDEFINED &&
                        activeErrorsValid &&
                        disarmReasonValid;
  odriveHasError = odriveUartConnected &&
                   (odriveActiveErrors != 0 || odriveDisarmReason != 0);

  if (odriveUartConnected) {
    const ODriveFeedback feedback = odrive.getFeedback();
    measuredMotorRpm = feedback.vel * 60.0f;
    // measuredSpindleRpm = measuredMotorRpm * GEAR_RATIO;
    measuredSpindleRpm = measuredMotorRpm;
    measuredSpeedPercent =
      fabsf(measuredMotorRpm) / MAX_MOTOR_RPM * 100.0f;
  } else {
    measuredMotorRpm = 0.0f;
    measuredSpindleRpm = 0.0f;
    measuredSpeedPercent = 0.0f;
  }

  updateBlueSpeedFeedback(
    measuredMotorRpm / 60.0f,
    odriveUartConnected
  );

  const bool normalState = odriveState == AXIS_STATE_IDLE ||
                           odriveState == AXIS_STATE_CLOSED_LOOP_CONTROL;
  redAlarmHigh = odriveHasError || odriveFaultLatched ||
                 !odriveUartConnected || !normalState;
  // Active (alarm) -> 0V (LOW). Normal -> 5V (HIGH).
  digitalWrite(RED_ALARM_SIGNAL_PIN, redAlarmHigh ? LOW : HIGH);
}

bool enterClosedLoop()
{
  const uint32_t startMs = millis();
  while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL &&
         millis() - startMs < 1000) {
    // Do not clear errors here.  Errors must first be observed by
    // updateODriveStatus(), which asserts the Carvera alarm, and are then
    // cleared by the dedicated retry logic in loop().
    odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
    delay(10);
  }

  odriveInClosedLoop =
    odrive.getState() == AXIS_STATE_CLOSED_LOOP_CONTROL;
  return odriveInClosedLoop;
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
  Serial.print(" | errors: ");
  Serial.print(odriveActiveErrors);
  Serial.print(" | disarm: ");
  Serial.print(odriveDisarmReason);
  Serial.print(" | UART: ");
  Serial.print(odriveUartConnected ? "CONNECTED" : "LOST");
  Serial.print(" | Red: ");
  Serial.println(redAlarmHigh ? "ALARM" : "NORMAL");
}

void setup()
{
  pinMode(PWM_INPUT_PIN, INPUT_PULLDOWN);
  pinMode(BLUE_SPEED_FEEDBACK_PIN, OUTPUT);
  pinMode(RED_ALARM_SIGNAL_PIN, OUTPUT);

  digitalWrite(BLUE_SPEED_FEEDBACK_PIN, LOW);
  // Keep alarm HIGH throughout startup to indicate normal state
  // and avoid a false Carvera alarm.
  digitalWrite(RED_ALARM_SIGNAL_PIN, HIGH);

  Serial.begin(PC_BAUDRATE);
  odriveSerial.begin(
    ODRIVE_BAUDRATE,
    SERIAL_8N1,
    ODRIVE_RX_PIN,
    ODRIVE_TX_PIN
  );
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmEdgeISR, CHANGE);

  odrive.setVelocity(0.0f);
  odrive.setState(AXIS_STATE_IDLE);
  spindleOffStartTimeMs = millis();

  Serial.println("ESP32 Carvera/ODrive UART interface started");
}

void loop()
{
  float newDuty = 0.0f;
  const PWMReadResult pwmResult = readPWMDuty(newDuty);

  if (pwmResult == PWMReadResult::NEW_SAMPLE) {
    pwmSignalValid = true;
    if (newDuty < START_DUTY_THRESHOLD) newDuty = 0.0f;
    filteredDuty += FILTER_ALPHA * (newDuty - filteredDuty);
    if (filteredDuty < START_DUTY_THRESHOLD) filteredDuty = 0.0f;
  } else if (pwmResult == PWMReadResult::TIMEOUT) {
    // Only a real signal timeout clears the last valid command. Merely having
    // no new 1 kHz sample during this pass through loop() does not.
    pwmSignalValid = false;
    filteredDuty = 0.0f;
  }

  // PWM duty represents spindle RPM. Convert it to motor RPM, then apply the
  // independent motor safety limit.
  targetSpindleRpm = filteredDuty * MAX_MOTOR_RPM;
  targetMotorRpm = constrain(
    targetSpindleRpm,
    0.0f,
    MAX_MOTOR_RPM
  );
  const float targetVelocity =
    DIRECTION_SIGN * targetMotorRpm / 60.0f;
  const uint32_t nowMs = millis();

  if (nowMs - lastCommandTimeMs >= COMMAND_INTERVAL_MS) {
    lastCommandTimeMs = nowMs;

    if (pwmSignalValid && filteredDuty >= START_DUTY_THRESHOLD &&
        !odriveFaultLatched) {
      if (odriveInClosedLoop || enterClosedLoop()) {
        odrive.setVelocity(targetVelocity);
      }
      spindleOffStartTimeMs = nowMs;
    } else {
      odrive.setVelocity(0.0f);
      if (odriveInClosedLoop &&
          nowMs - spindleOffStartTimeMs >= IDLE_DELAY_MS) {
        odrive.setState(AXIS_STATE_IDLE);
        odriveInClosedLoop = false;
      }
    }
  }

  if (nowMs - lastStatusTimeMs >= STATUS_INTERVAL_MS) {
    lastStatusTimeMs = nowMs;
    updateODriveStatus();
  }

  if (odriveHasError) {
    odriveFaultLatched = true;
    redAlarmHigh = true;
    digitalWrite(RED_ALARM_SIGNAL_PIN, LOW);
  }

  // If ODrive reports an error, wait 1 second then try to clear it.
  // Keep retrying every second while an error remains.
  if (odriveHasError) {
    if (odriveErrorFirstSeenMs == 0) {
      odriveErrorFirstSeenMs = nowMs;
    } else if (nowMs - odriveErrorFirstSeenMs >= ERROR_CLEAR_RETRY_MS) {
      odrive.clearErrors();
      odrive.setVelocity(0.0f);
      odrive.setState(AXIS_STATE_IDLE);
      odriveInClosedLoop = false;
      odriveErrorFirstSeenMs = nowMs;
    }
  } else {
    odriveErrorFirstSeenMs = 0;
    if (odriveFaultLatched &&
        (!pwmSignalValid || filteredDuty < START_DUTY_THRESHOLD)) {
      odrive.setVelocity(0.0f);
      odrive.setState(AXIS_STATE_IDLE);
      odriveInClosedLoop = false;
      odriveFaultLatched = false;
    }
  }

  static uint32_t lastPrintTimeMs = 0;
  if (nowMs - lastPrintTimeMs >= PRINT_INTERVAL_MS) {
    lastPrintTimeMs = nowMs;
    printStatus();
  }
}

