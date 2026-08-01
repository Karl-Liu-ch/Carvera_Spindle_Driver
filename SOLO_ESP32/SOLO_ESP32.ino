/*
  Carvera PWM -> ESP32 -> SOLO PICO UART

  ESP32 wiring:
    GPIO2  <- Carvera spindle PWM through a 5 V to 3.3 V level converter
    GPIO16 <- SOLO TX (UART2 RX)
    GPIO17 -> SOLO RX (UART2 TX)
    GPIO18 -> Carvera Blue speed feedback, 12 pulses/revolution
    GPIO19 -> Carvera Red alarm, LOW on fault / HIGH when normal

  SOLO, ESP32 and Carvera must share GND.
  ESP32 GPIO is not 5 V tolerant. Level-shift Carvera signals as required.
  SOLO must already be commissioned and saved in Motion Monitor.
*/

#include <Arduino.h>
#include <SOLOMotorControllersUart.h>
#include <math.h>

// Hardware
static constexpr uint8_t PWM_INPUT_PIN = 2;
static constexpr uint8_t BLUE_SPEED_FEEDBACK_PIN = 18;
static constexpr uint8_t RED_ALARM_SIGNAL_PIN = 19;
static constexpr int SOLO_RX_PIN = 16;
static constexpr int SOLO_TX_PIN = 17;

// Spindle and motor
static constexpr float GEAR_RATIO = 1.635f;
static constexpr float MAX_SPINDLE_RPM = 18000.0f;
static constexpr float MAX_MOTOR_RPM = 12500.0f;
static constexpr float SPEED_FEEDBACK_PPR = 12.0f;
static constexpr float START_DUTY_THRESHOLD = 0.002f;
static constexpr float FILTER_ALPHA = 0.20f;

// Runtime configuration restored after every ESP32 or SOLO restart. These
// values match the verified Motion Monitor setup.
static constexpr float SOLO_CURRENT_LIMIT_A = 4.19999695f;
static constexpr float SOLO_SPEED_KP = 0.11f;
static constexpr float SOLO_SPEED_KI = 0.003f;
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
static constexpr uint32_t COMMUNICATION_TIMEOUT_MS = 1000;
static constexpr uint32_t DRIVE_DISABLE_DELAY_MS = 11000;

// Serial and SOLO
static constexpr unsigned long PC_BAUDRATE = 115200;
static constexpr unsigned long SOLO_BAUDRATE = 115200;
static constexpr uint8_t SOLO_DEVICE_ADDRESS = 0;

HardwareSerial soloSerial(2);
SOLOMotorControllersUart *solo = nullptr;

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
float speedFeedbackFrequencyHz = 0.0f;

bool pwmSignalValid = false;
bool soloCommunicationValid = false;
bool soloDriveEnabled = false;
bool soloHasError = false;
bool soloFaultLatched = false;
bool alarmActive = true;
bool soloNeedsResynchronization = false;

long soloErrorRegister = 0;
int soloLibraryError = 0;

uint32_t lastCommandTimeMs = 0;
uint32_t lastSpeedPollTimeMs = 0;
uint32_t lastErrorPollTimeMs = 0;
uint32_t lastPrintTimeMs = 0;
uint32_t lastSuccessfulCommunicationMs = 0;
uint32_t communicationLostStartMs = 0;
uint32_t lastRunRequestedTimeMs = 0;
uint32_t soloErrorStartTimeMs = 0;
uint32_t lastSpeedReferenceSentTimeMs = 0;
float lastSpeedReferenceSentRpm = -1.0f;

// ---------------------------------------------------------------------------
// PWM input
// ---------------------------------------------------------------------------

void IRAM_ATTR pwmEdgeISR()
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
  communicationLostStartMs = 0;
}

void recordCommunicationFailure()
{
  soloCommunicationValid = false;
  soloNeedsResynchronization = true;
  if (communicationLostStartMs == 0) {
    communicationLostStartMs = millis();
  }
}

bool setSoloDriveEnabled(bool enabled)
{
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
  ok &= solo->SetMotorDirection(
    SOLOMotorControllers::Direction::CLOCKWISE, soloLibraryError);

  if (ok) {
    recordCommunicationSuccess();
    Serial.println("SOLO runtime configuration restored");
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
    // UART error. This interface commands one direction, so use its magnitude.
    measuredMotorRpm = fabsf(static_cast<float>(rpm));
    recordCommunicationSuccess();
  }
  else {
    measuredMotorRpm = 0.0f;
    recordCommunicationFailure();
  }

  updateBlueSpeedFeedback(
    measuredMotorRpm,
    soloCommunicationValid && !soloHasError && !soloFaultLatched
  );
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

bool communicationTimedOut(uint32_t nowMs)
{
  if (lastSuccessfulCommunicationMs == 0) {
    return true;
  }

  return !soloCommunicationValid &&
         communicationLostStartMs != 0 &&
         nowMs - communicationLostStartMs > COMMUNICATION_TIMEOUT_MS;
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

  const bool ok = applySoloStartupConfiguration();
  if (ok) {
    soloNeedsResynchronization = false;
  }
  return ok;
}

void resynchronizeSoloAfterReconnect()
{
  if (!soloNeedsResynchronization || !soloCommunicationValid) {
    return;
  }

  Serial.println("SOLO reconnected: resynchronizing runtime state");

  // A SOLO power cycle invalidates the ESP32's cached drive state. Establish a
  // known-safe runtime state without changing any commissioned configuration.
  const bool disableOk = setSoloDriveEnabled(false);
  const bool zeroOk = sendSoloSpeed(0.0f);
  const bool configurationOk = applySoloStartupConfiguration();

  if (disableOk && zeroOk && configurationOk) {
    soloDriveEnabled = false;
    lastSpeedReferenceSentRpm = 0.0f;
    soloNeedsResynchronization = false;
    lastRunRequestedTimeMs = millis();
    Serial.println("SOLO runtime state synchronized");
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
  Serial.print(" rpm | Blue: ");
  Serial.print(speedFeedbackFrequencyHz, 1);
  Serial.print(" Hz | drive: ");
  Serial.print(soloDriveEnabled ? "ENABLED" : "DISABLED");
  Serial.print(" | SOLO errors: 0x");
  Serial.print(static_cast<unsigned long>(soloErrorRegister), HEX);
  Serial.print(" | library error: ");
  Serial.print(soloLibraryError);
  Serial.print(" | UART: ");
  Serial.print(soloCommunicationValid ? "CONNECTED" : "LOST");
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

  soloSerial.begin(
    SOLO_BAUDRATE,
    SERIAL_8N1,
    SOLO_RX_PIN,
    SOLO_TX_PIN
  );
  solo = new SOLOMotorControllersUart(
    SOLO_DEVICE_ADDRESS,
    soloSerial,
    SOLOMotorControllers::UartBaudrate::RATE_115200,
    30,
    3
  );

  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN), pwmEdgeISR, CHANGE);
  Serial.println("ESP32 Carvera / SOLO interface startup");

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
  resynchronizeSoloAfterReconnect();

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

  if (nowMs - lastCommandTimeMs >= COMMAND_INTERVAL_MS) {
    lastCommandTimeMs = nowMs;

    const bool runRequested =
      pwmSignalValid &&
      filteredDuty >= START_DUTY_THRESHOLD &&
      !soloHasError &&
      !soloFaultLatched &&
      !communicationTimedOut(nowMs);

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

  if (nowMs - lastSpeedPollTimeMs >= SPEED_POLL_INTERVAL_MS) {
    lastSpeedPollTimeMs = nowMs;
    pollSoloSpeed();
  }

  if (nowMs - lastErrorPollTimeMs >= ERROR_POLL_INTERVAL_MS) {
    lastErrorPollTimeMs = nowMs;
    pollSoloErrors();
  }

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

  const bool seriousFault =
    communicationTimedOut(nowMs) || soloHasError || soloFaultLatched;

  if (seriousFault) {
    updateBlueSpeedFeedback(0.0f, false);
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
