/*
  Carvera PWM -> Arduino Nano R4 -> SOLO PICO CANopen

  Wiring:
    D2  <- Carvera spindle PWM command, approximately 1 kHz
    D3  -> Carvera Blue speed feedback, 12 pulses/revolution
    D4  -> CAN transceiver TXD (Nano R4 native CAN TX)
    D5  <- CAN transceiver RXD (Nano R4 native CAN RX)
    D6  -> Carvera Red alarm, LOW on fault / HIGH when normal

  CAN transceiver CANH/CANL connect to SOLO CANH/CANL. SOLO, Nano R4,
  transceiver and Carvera must share GND. Terminate both physical ends of the
  CAN bus with 120 ohms.

  SOLO must already be commissioned and saved in Motion Monitor with CANopen
  node ID 1 and CAN bitrate 1 Mbit/s.
*/

#include <Arduino.h>
// SOLOMotorControllers 5.5.0 omits Nano R4 from its native-CAN compile guard.
// The companion compatibility source file supplies the same RA4M1 backend.
#if defined(ARDUINO_NANO_R4) && !defined(ARDUINO_MINIMA)
#define ARDUINO_MINIMA
#endif

#include <SOLOMotorControllersCanopenNative.h>
#include <math.h>

// Hardware
static constexpr uint8_t PWM_INPUT_PIN = 2;
static constexpr uint8_t BLUE_SPEED_FEEDBACK_PIN = 3;
static constexpr uint8_t RED_ALARM_SIGNAL_PIN = 6;

// Spindle and motor
static constexpr float GEAR_RATIO = 1.635f;
static constexpr float MAX_SPINDLE_RPM = 18000.0f;
static constexpr float MAX_MOTOR_RPM = 12500.0f;
static constexpr float SPEED_FEEDBACK_PPR = 12.0f;
static constexpr float START_DUTY_THRESHOLD = 0.002f;
static constexpr float FILTER_ALPHA = 0.20f;

// Runtime configuration restored after every Nano or SOLO restart. These
// values match the verified Motion Monitor setup.
static constexpr float SOLO_CURRENT_LIMIT_A = 12.6f;
static constexpr float SOLO_SPEED_KP = 0.2;
static constexpr float SOLO_SPEED_KI = 0.005;
static constexpr float SOLO_CURRENT_KP = 0.27764893f;
static constexpr float SOLO_CURRENT_KI = 0.01895905f;
static constexpr float SOLO_ACCELERATION_RPS2 = 100.708008f;
static constexpr float SOLO_DECELERATION_RPS2 = 20.354004f;

// Expected PWM input: approximately 1 kHz
static constexpr uint32_t MIN_PWM_PERIOD_US = 700;
static constexpr uint32_t MAX_PWM_PERIOD_US = 1500;
static constexpr uint32_t PWM_EDGE_TIMEOUT_US = 100000;

// Runtime timing
static constexpr uint32_t COMMAND_INTERVAL_MS = 50;
static constexpr uint32_t COMMAND_KEEPALIVE_MS = 500;
static constexpr float COMMAND_CHANGE_THRESHOLD_RPM = 50.0f;
static constexpr uint32_t SPEED_POLL_INTERVAL_MS = 200;
static constexpr uint32_t ERROR_POLL_INTERVAL_MS = 500;
static constexpr uint32_t PRINT_INTERVAL_MS = 500;
static constexpr uint32_t ERROR_CLEAR_RETRY_MS = 1000;
static constexpr uint32_t DRIVE_DISABLE_DELAY_MS = 11000;
static constexpr uint32_t COMMUNICATION_SAFETY_RETRY_MS = 500;
static constexpr uint8_t COMMUNICATION_FAILURE_ALARM_THRESHOLD = 5;
static constexpr float SPEED_DROP_ALARM_RATIO = 0.10f;
static constexpr float SPEED_DROP_MONITOR_MIN_RPM = 500.0f;
static constexpr float SPEED_TARGET_STEP_MIN_RPM = 200.0f;
static constexpr uint32_t SPEED_DROP_CONFIRMATION_MS = 500;
static constexpr uint32_t IQ_POLL_INTERVAL_MS = 200;
static constexpr uint32_t POWER_INDEX_CONFIRMATION_MS = 1000;
static constexpr float power_index = 52000.0f;

// USB serial and SOLO CANopen
static constexpr unsigned long PC_BAUDRATE = 115200;
static constexpr uint8_t SOLO_CANOPEN_NODE_ID = 1;
static constexpr SOLOMotorControllers::CanbusBaudrate CANOPEN_BITRATE =
  SOLOMotorControllers::CanbusBaudrate::RATE_1000;
static constexpr long CANOPEN_RESPONSE_TIMEOUT_MS = 30;

SOLOMotorControllersCanopenNative *solo = nullptr;

// PWM capture, shared with interrupt
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

// Runtime state
float filteredDuty = 0.0f;
float targetMotorRpm = 0.0f;
float measuredMotorRpm = 0.0f;
float measuredMotorIqA = 0.0f;
float currentPowerIndex = 0.0f;
float speedFeedbackFrequencyHz = 0.0f;

bool pwmSignalValid = false;
bool soloCommunicationValid = false;
bool soloDriveEnabled = false;
bool soloHasError = false;
bool soloFaultLatched = false;
bool communicationFaultLatched = false;
bool speedDropFaultLatched = false;
bool speedDropMonitorArmed = false;
bool motorIqValid = false;
bool powerIndexFaultLatched = false;
bool alarmActive = true;
uint8_t consecutiveCommunicationFailures = 0;

long soloErrorRegister = 0;
int soloLibraryError = 0;
int soloMotorDirection = -1;

uint32_t lastCommandTimeMs = 0;
uint32_t lastSpeedPollTimeMs = 0;
uint32_t lastIqPollTimeMs = 0;
uint32_t lastErrorPollTimeMs = 0;
uint32_t lastPrintTimeMs = 0;
uint32_t lastSuccessfulCommunicationMs = 0;
uint32_t lastRunRequestedTimeMs = 0;
uint32_t soloErrorStartTimeMs = 0;
uint32_t lastCommunicationSafetyRetryMs = 0;
uint32_t speedDropStartTimeMs = 0;
uint32_t powerIndexStartTimeMs = 0;
uint32_t lastSpeedReferenceSentTimeMs = 0;
float lastSpeedReferenceSentRpm = -1.0f;
float lastSpeedDropMonitorTargetRpm = 0.0f;

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

  const bool edgeTimedOut =
    sampleTimeUs == 0 ||
    static_cast<uint32_t>(micros() - sampleTimeUs) > PWM_EDGE_TIMEOUT_US;

  if (edgeTimedOut) {
    // Exact 0% and 100% PWM have no edges. INPUT_PULLDOWN makes a disconnected
    // wire safely read as 0%; a steady HIGH is a valid 100% command.
    duty = digitalRead(PWM_INPUT_PIN) ? 1.0f : 0.0f;
    return PWMReadResult::NEW_SAMPLE;
  }

  if (!newMeasurement || periodUs == 0) {
    return PWMReadResult::NO_NEW_SAMPLE;
  }

  duty = constrain(
    static_cast<float>(highTimeUs) / static_cast<float>(periodUs),
    0.0f,
    1.0f
  );
  return PWMReadResult::NEW_SAMPLE;
}

// ---------------------------------------------------------------------------
// Carvera outputs
// ---------------------------------------------------------------------------

void updateBlueSpeedFeedback(float motorRpm, bool valid)
{
  // Carvera's motherboard applies the transmission-ratio conversion itself,
  // so report motor RPM directly and do not apply GEAR_RATIO here.
  const float feedbackRpm = motorRpm;

  speedFeedbackFrequencyHz =
    fabsf(feedbackRpm) / 60.0f * SPEED_FEEDBACK_PPR;

  if (!valid || !isfinite(speedFeedbackFrequencyHz) ||
      speedFeedbackFrequencyHz < 0.5f) {
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

void setCarveraAlarm(bool active)
{
  alarmActive = active;
  digitalWrite(RED_ALARM_SIGNAL_PIN, active ? LOW : HIGH);
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
  return consecutiveCommunicationFailures >
         COMMUNICATION_FAILURE_ALARM_THRESHOLD;
}

bool setAndVerifySoloClockwiseDirection()
{
  soloLibraryError = 0;
  if (!solo->SetMotorDirection(
        SOLOMotorControllers::Direction::CLOCKWISE, soloLibraryError)) {
    return false;
  }

  const SOLOMotorControllers::Direction direction =
    solo->GetMotorDirection(soloLibraryError);
  soloMotorDirection = static_cast<int>(direction);

  return soloLibraryError == 0 &&
         direction == SOLOMotorControllers::Direction::CLOCKWISE;
}

bool clearAndVerifySoloErrors()
{
  soloLibraryError = 0;
  if (!solo->OverwriteErrorRegister(soloLibraryError)) {
    return false;
  }

  const long errorRegister = solo->GetErrorRegister(soloLibraryError);
  if (soloLibraryError != 0) {
    return false;
  }

  soloErrorRegister = errorRegister;
  soloHasError = errorRegister != 0;
  if (!soloHasError) {
    soloErrorStartTimeMs = 0;
  }
  return !soloHasError;
}

bool setSoloDriveEnabled(bool enabled)
{
  if (enabled) {
    // Never enable on a stale driver error. Clear and verify the error
    // register first, then restore and verify the spindle direction.
    if (!clearAndVerifySoloErrors()) {
      if (soloLibraryError != 0) {
        recordCommunicationFailure();
      }
      else {
        recordCommunicationSuccess();
      }
      return false;
    }
    if (!setAndVerifySoloClockwiseDirection()) {
      recordCommunicationFailure();
      return false;
    }
  }

  soloLibraryError = 0;
  const bool ok = solo->SetDriveDisableEnable(
    enabled ? SOLOMotorControllers::DisableEnable::ENABLE
            : SOLOMotorControllers::DisableEnable::DISABLE,
    soloLibraryError
  );

  if (ok) {
    soloDriveEnabled = enabled;
    recordCommunicationSuccess();
  }
  else {
    recordCommunicationFailure();
  }
  return ok;
}

bool applySoloStartupConfiguration()
{
  soloLibraryError = 0;

  bool ok = solo->SetCommandMode(
    SOLOMotorControllers::CommandMode::DIGITAL, soloLibraryError);
  ok &= solo->SetMotorType(
    SOLOMotorControllers::MotorType::BLDC_PMSM, soloLibraryError);
  ok &= solo->SetFeedbackControlMode(
    SOLOMotorControllers::FeedbackControlMode::HALL_SENSORS,
    soloLibraryError);
  ok &= solo->SetControlMode(
    SOLOMotorControllers::ControlMode::SPEED_MODE, soloLibraryError);
  ok &= solo->SetCurrentLimit(SOLO_CURRENT_LIMIT_A, soloLibraryError);
  ok &= solo->SetCurrentControllerKp(SOLO_CURRENT_KP, soloLibraryError);
  ok &= solo->SetCurrentControllerKi(SOLO_CURRENT_KI, soloLibraryError);
  ok &= solo->SetSpeedControllerKp(SOLO_SPEED_KP, soloLibraryError);
  ok &= solo->SetSpeedControllerKi(SOLO_SPEED_KI, soloLibraryError);
  ok &= solo->SetSpeedAccelerationValue(
    SOLO_ACCELERATION_RPS2, soloLibraryError);
  ok &= solo->SetSpeedDecelerationValue(
    SOLO_DECELERATION_RPS2, soloLibraryError);
  ok &= setAndVerifySoloClockwiseDirection();

  if (ok) {
    recordCommunicationSuccess();
    Serial.println(
      "SOLO runtime configuration restored; direction verified: CW");
  }
  else {
    recordCommunicationFailure();
  }
  return ok;
}

bool sendSoloSpeed(float rpm)
{
  const long commandRpm = static_cast<long>(
    lroundf(constrain(rpm, 0.0f, MAX_MOTOR_RPM))
  );

  soloLibraryError = 0;
  const bool ok = solo->SetSpeedReference(commandRpm, soloLibraryError);

  if (ok) {
    recordCommunicationSuccess();
    lastSpeedReferenceSentRpm = static_cast<float>(commandRpm);
    lastSpeedReferenceSentTimeMs = millis();
  }
  else {
    recordCommunicationFailure();
  }
  return ok;
}

void disableSoloSafely()
{
  sendSoloSpeed(0.0f);
  setSoloDriveEnabled(false);
}

void pollSoloSpeed()
{
  soloLibraryError = 0;
  const long rpm = solo->GetSpeedFeedback(soloLibraryError);

  if (soloLibraryError == 0) {
    // SOLO speed feedback is signed. A negative value means the configured
    // motor direction is opposite to SOLO's positive convention; it is not a
    // CANopen error. This interface commands one direction, so use its magnitude.
    measuredMotorRpm = fabsf(static_cast<float>(rpm));
    recordCommunicationSuccess();
  }
  else {
    recordCommunicationFailure();
  }

  // A failed read does not mean the motor stopped. Keep the last valid speed
  // feedback through short CANopen dropouts so Carvera does not see a false
  // spindle-speed loss.
  updateBlueSpeedFeedback(
    measuredMotorRpm,
    !communicationFailureLimitExceeded() &&
      !soloHasError && !soloFaultLatched
  );
}

void pollSoloIq()
{
  soloLibraryError = 0;
  const float iqA =
    solo->GetQuadratureCurrentIqFeedback(soloLibraryError);

  if (soloLibraryError == 0 && isfinite(iqA)) {
    // Iq can be signed according to SOLO's direction convention. Power-load
    // protection must work in either direction, so retain its magnitude.
    measuredMotorIqA = fabsf(iqA);
    motorIqValid = true;
    recordCommunicationSuccess();
  }
  else {
    motorIqValid = false;
    recordCommunicationFailure();
  }
}

void pollSoloErrors()
{
  soloLibraryError = 0;
  const long errorRegister = solo->GetErrorRegister(soloLibraryError);

  if (soloLibraryError != 0) {
    recordCommunicationFailure();
    return;
  }

  soloErrorRegister = errorRegister;
  const bool hasError = errorRegister != 0;

  if (hasError && !soloHasError) {
    soloErrorStartTimeMs = millis();
  }
  else if (!hasError) {
    soloErrorStartTimeMs = 0;
  }

  soloHasError = hasError;
  recordCommunicationSuccess();
}

void tryClearSoloError()
{
  soloLibraryError = 0;
  const bool ok = solo->OverwriteErrorRegister(soloLibraryError);

  if (ok) {
    recordCommunicationSuccess();
    Serial.println("Attempted to clear SOLO error");
  }
  else {
    recordCommunicationFailure();
  }
}

bool initializeSolo()
{
  soloLibraryError = 0;
  if (!solo->CommunicationIsWorking(soloLibraryError)) {
    recordCommunicationFailure();
    return false;
  }

  // Establish a safe state before restoring the verified runtime parameters.
  if (!setSoloDriveEnabled(false)) {
    return false;
  }

  if (!sendSoloSpeed(0.0f)) {
    return false;
  }

  return applySoloStartupConfiguration();
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
    // A substantially higher command is normal acceleration. Rearm after
    // measured speed reaches 90% of the new target.
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
  else if (nowMs - speedDropStartTimeMs >=
           SPEED_DROP_CONFIRMATION_MS) {
    speedDropFaultLatched = true;
    Serial.println("Motor speed dropped more than 10%: fault latched");
  }
}

void updatePowerIndexMonitor(uint32_t nowMs)
{
  if (powerIndexFaultLatched) {
    return;
  }

  if (!motorIqValid || !soloDriveEnabled ||
      targetMotorRpm < SPEED_DROP_MONITOR_MIN_RPM ||
      filteredDuty < START_DUTY_THRESHOLD ||
      soloHasError || soloFaultLatched || communicationFaultLatched) {
    currentPowerIndex = 0.0f;
    powerIndexStartTimeMs = 0;
    return;
  }

  currentPowerIndex = measuredMotorIqA * measuredMotorRpm;

  if (currentPowerIndex <= power_index) {
    powerIndexStartTimeMs = 0;
    return;
  }

  if (powerIndexStartTimeMs == 0) {
    powerIndexStartTimeMs = nowMs;
  }
  else if (nowMs - powerIndexStartTimeMs >=
           POWER_INDEX_CONFIRMATION_MS) {
    powerIndexFaultLatched = true;
    Serial.println("Iq * motor RPM exceeded power_index for 1 s: fault latched");
  }
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

void printStatus()
{
  const float targetSpindleRpm = targetMotorRpm * GEAR_RATIO;
  const float measuredSpindleRpm = measuredMotorRpm * GEAR_RATIO;

  Serial.print("PWM: ");
  Serial.print(pwmSignalValid ? "VALID" : "NO");
  Serial.print(" | duty: ");
  Serial.print(filteredDuty * 100.0f, 1);
  Serial.print("% | target motor: ");
  Serial.print(targetMotorRpm, 0);
  Serial.print(" rpm | target spindle: ");
  Serial.print(targetSpindleRpm, 0);
  Serial.print(" rpm | measured motor: ");
  Serial.print(measuredMotorRpm, 0);
  Serial.print(" rpm | measured spindle: ");
  Serial.print(measuredSpindleRpm, 0);
  Serial.print(" rpm | Iq: ");
  if (motorIqValid) {
    Serial.print(measuredMotorIqA, 2);
    Serial.print(" A | power index: ");
    Serial.print(currentPowerIndex, 0);
  }
  else {
    Serial.print("INVALID | power index: INVALID");
  }
  Serial.print(" rpm | Blue: ");
  Serial.print(speedFeedbackFrequencyHz, 1);
  Serial.print(" Hz | drive: ");
  Serial.print(soloDriveEnabled ? "ENABLED" : "DISABLED");
  Serial.print(" | speed monitor: ");
  Serial.print(speedDropMonitorArmed ? "ARMED" : "WAIT");
  if (speedDropFaultLatched) {
    Serial.print("/FAULT");
  }
  Serial.print(" | power monitor: ");
  if (powerIndexFaultLatched) {
    Serial.print("FAULT");
  }
  else if (powerIndexStartTimeMs != 0) {
    Serial.print("TIMING");
  }
  else {
    Serial.print("NORMAL");
  }
  Serial.print(" | SOLO errors: 0x");
  Serial.print(static_cast<unsigned long>(soloErrorRegister), HEX);
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
  Serial.print(" | CANopen: ");
  Serial.print(soloCommunicationValid ? "CONNECTED" : "LOST");
  Serial.print(" (");
  Serial.print(consecutiveCommunicationFailures);
  Serial.print(" consecutive failures)");
  Serial.print(" | Red: ");
  Serial.println(alarmActive ? "ALARM" : "NORMAL");
}

void handleUSBSerial()
{
  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n');
  command.trim();
  if (command.equalsIgnoreCase("status")) {
    printStatus();
  }
  else if (command.length() != 0) {
    Serial.println("Enter 'status' to print status");
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup()
{
  pinMode(PWM_INPUT_PIN, INPUT_PULLDOWN);
  pinMode(BLUE_SPEED_FEEDBACK_PIN, OUTPUT);
  pinMode(RED_ALARM_SIGNAL_PIN, OUTPUT);
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
  Serial.println("Nano R4 Carvera / SOLO CANopen interface startup");

  if (initializeSolo()) {
    Serial.println("SOLO communication established");
    setCarveraAlarm(false);
  }
  else {
    Serial.print("SOLO initialisation failed, error: ");
    Serial.println(soloLibraryError);
    setCarveraAlarm(true);
  }

  lastRunRequestedTimeMs = millis();
}

void loop()
{
  handleUSBSerial();

  float newDuty = 0.0f;
  if (readPWMDuty(newDuty) == PWMReadResult::NEW_SAMPLE) {
    pwmSignalValid = true;

    if (newDuty < START_DUTY_THRESHOLD) {
      newDuty = 0.0f;
    }

    filteredDuty += FILTER_ALPHA * (newDuty - filteredDuty);
    if (filteredDuty < START_DUTY_THRESHOLD) {
      filteredDuty = 0.0f;
    }
  }

  const float requestedSpindleRpm = filteredDuty * MAX_SPINDLE_RPM;
  targetMotorRpm = constrain(
    requestedSpindleRpm / GEAR_RATIO,
    0.0f,
    MAX_MOTOR_RPM
  );

  const uint32_t nowMs = millis();

  // Once a communication fault is latched, stop all normal CANopen traffic.
  // The dedicated safety-recovery block below is then the only CANopen caller.
  if (!communicationFaultLatched &&
      nowMs - lastCommandTimeMs >= COMMAND_INTERVAL_MS) {
    lastCommandTimeMs = nowMs;

    const bool runRequested =
      pwmSignalValid &&
      filteredDuty >= START_DUTY_THRESHOLD &&
      !soloHasError &&
      !soloFaultLatched &&
      !speedDropFaultLatched &&
      !powerIndexFaultLatched &&
      !communicationFaultLatched;

    if (runRequested) {
      if (!soloDriveEnabled) {
        setSoloDriveEnabled(true);
      }

      if (soloDriveEnabled) {
        const bool referenceChanged =
          lastSpeedReferenceSentRpm < 0.0f ||
          fabsf(targetMotorRpm - lastSpeedReferenceSentRpm) >=
            COMMAND_CHANGE_THRESHOLD_RPM;
        const bool keepaliveDue =
          nowMs - lastSpeedReferenceSentTimeMs >= COMMAND_KEEPALIVE_MS;

        if (referenceChanged || keepaliveDue) {
          sendSoloSpeed(targetMotorRpm);
        }
      }
      lastRunRequestedTimeMs = nowMs;
    }
    else {
      sendSoloSpeed(0.0f);

      if (soloDriveEnabled &&
          nowMs - lastRunRequestedTimeMs >= DRIVE_DISABLE_DELAY_MS) {
        setSoloDriveEnabled(false);
      }
    }
  }

  if (!communicationFaultLatched &&
      nowMs - lastSpeedPollTimeMs >= SPEED_POLL_INTERVAL_MS) {
    lastSpeedPollTimeMs = nowMs;
    pollSoloSpeed();
  }

  if (!communicationFaultLatched &&
      nowMs - lastIqPollTimeMs >= IQ_POLL_INTERVAL_MS) {
    lastIqPollTimeMs = nowMs;
    pollSoloIq();
  }

  if (!communicationFaultLatched &&
      nowMs - lastErrorPollTimeMs >= ERROR_POLL_INTERVAL_MS) {
    lastErrorPollTimeMs = nowMs;
    pollSoloErrors();
  }

  updateSpeedDropMonitor(nowMs);
  updatePowerIndexMonitor(nowMs);

  if (soloHasError) {
    soloFaultLatched = true;

    if (soloErrorStartTimeMs != 0 &&
        nowMs - soloErrorStartTimeMs >= ERROR_CLEAR_RETRY_MS) {
      tryClearSoloError();
      disableSoloSafely();
      soloErrorStartTimeMs = nowMs;
      pollSoloErrors();
    }
  }
  else if (soloFaultLatched && filteredDuty < START_DUTY_THRESHOLD) {
    disableSoloSafely();
    soloFaultLatched = false;
  }

  if (speedDropFaultLatched &&
      filteredDuty < START_DUTY_THRESHOLD) {
    disableSoloSafely();
    speedDropFaultLatched = false;
    speedDropMonitorArmed = false;
    speedDropStartTimeMs = 0;
  }

  if (powerIndexFaultLatched &&
      filteredDuty < START_DUTY_THRESHOLD) {
    disableSoloSafely();
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

    // During a real CANopen outage SOLO continues using its last received speed.
    // Keep retrying a zero-speed command; as soon as communication returns,
    // command zero and then put the drive into DISABLE (idle). Do not restart
    // automatically while the PWM run request is still present.
    const bool zeroAccepted = sendSoloSpeed(0.0f);
    if (zeroAccepted) {
      setSoloDriveEnabled(false);
    }
  }

  if (communicationFaultLatched &&
      soloCommunicationValid &&
      !soloDriveEnabled &&
      lastSpeedReferenceSentRpm == 0.0f &&
      filteredDuty < START_DUTY_THRESHOLD) {
    communicationFaultLatched = false;
  }

  const bool driveFault =
    soloHasError || soloFaultLatched || speedDropFaultLatched ||
    powerIndexFaultLatched;
  const bool communicationAlarm = communicationFaultLatched;
  const bool seriousFault = communicationAlarm || driveFault;

  if (driveFault || communicationAlarm) {
    updateBlueSpeedFeedback(0.0f, false);
  }

  if (driveFault) {
    if (soloDriveEnabled) {
      disableSoloSafely();
    }
  }
  setCarveraAlarm(seriousFault);

  if (nowMs - lastPrintTimeMs >= PRINT_INTERVAL_MS) {
    lastPrintTimeMs = nowMs;
    printStatus();
  }
}

