/*
  ESP32-C6 Super Mini: USB Serial controlled PWM generator

  GPIO4 -> 1 kHz, 8-bit PWM output (3.3 V logic)

  Open USB Serial Monitor at 115200 baud with a newline ending, then enter a
  spindle speed from 0 to 15000 RPM. The output is scaled linearly:
  0 RPM = 0% PWM, 15000 RPM = 100% PWM.

  IMPORTANT: ESP32-C6 GPIO is not 5 V tolerant.
*/

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>

// On ESP32-C6, Serial reaches the USB connector only when this Arduino board
// option is enabled. Otherwise Serial is routed to the UART0 GPIO pins.
#if !ARDUINO_USB_CDC_ON_BOOT
#error "Set Tools > USB CDC On Boot > Enabled, then compile and upload again."
#endif

static constexpr uint8_t PWM_OUTPUT_PIN = 4;
static constexpr uint32_t PWM_FREQUENCY_HZ = 1000;
static constexpr uint8_t PWM_RESOLUTION_BITS = 8;
static constexpr uint16_t PWM_MAX_VALUE = 255;
static constexpr float MAX_SPINDLE_RPM = 15000.0f;
static constexpr unsigned long USB_BAUDRATE = 115200;
static constexpr uint32_t STATUS_PRINT_INTERVAL_MS = 500;
static constexpr float STARTUP_SPINDLE_RPM = 7500.0f;

float spindleReferenceRpm = 0.0f;
float pwmReferencePercent = 0.0f;
uint8_t pwmDutyValue = 0;
uint32_t lastStatusPrintTimeMs = 0;

void printPWMReference()
{
  Serial.print("Spindle reference: ");
  Serial.print(spindleReferenceRpm, 0);
  Serial.print(" RPM | PWM: ");
  Serial.print(pwmReferencePercent, 1);
  Serial.print("% | 8-bit duty: ");
  Serial.print(pwmDutyValue);
  Serial.print("/255 | frequency: ");
  Serial.print(PWM_FREQUENCY_HZ);
  Serial.println(" Hz");
}

void setSpindleReference(float rpm)
{
  spindleReferenceRpm = constrain(rpm, 0.0f, MAX_SPINDLE_RPM);
  pwmReferencePercent = spindleReferenceRpm / MAX_SPINDLE_RPM * 100.0f;
  pwmDutyValue = static_cast<uint8_t>(
    lroundf(pwmReferencePercent * PWM_MAX_VALUE / 100.0f)
  );

  ledcWrite(PWM_OUTPUT_PIN, pwmDutyValue);
  printPWMReference();
}

void processSerialCommand(String command)
{
  command.trim();

  if (command.length() == 0) {
    return;
  }

  if (command.equalsIgnoreCase("status")) {
    printPWMReference();
    return;
  }

  char *endPointer = nullptr;
  const float requestedRpm = strtof(command.c_str(), &endPointer);

  while (endPointer != nullptr && isspace(static_cast<unsigned char>(*endPointer))) {
    ++endPointer;
  }

  const bool validNumber =
    endPointer != command.c_str() &&
    endPointer != nullptr &&
    *endPointer == '\0' &&
    isfinite(requestedRpm) &&
    requestedRpm >= 0.0f &&
    requestedRpm <= MAX_SPINDLE_RPM;

  if (!validNumber) {
    Serial.println("Invalid input. Enter spindle RPM from 0 to 15000.");
    return;
  }

  setSpindleReference(requestedRpm);
}

void setup()
{
  Serial.begin(USB_BAUDRATE);
  Serial.setTimeout(50);

  if (!ledcAttach(
        PWM_OUTPUT_PIN,
        PWM_FREQUENCY_HZ,
        PWM_RESOLUTION_BITS)) {
    Serial.println("ERROR: Could not attach LEDC PWM to GPIO4");
    while (true) {
      delay(1000);
    }
  }

  // Start with a visible 1 kHz square wave so PWM can be verified even before
  // the USB Serial port is opened.
  setSpindleReference(STARTUP_SPINDLE_RPM);
  delay(500);

  Serial.println("ESP32-C6 PWM generator ready");
  Serial.println("Enter spindle RPM (0-15000), then press Enter.");
  Serial.println("Type 'status' to read the current RPM and PWM setting.");
  printPWMReference();
}

void loop()
{
  if (Serial.available()) {
    processSerialCommand(Serial.readStringUntil('\n'));
  }

  const uint32_t nowMs = millis();

  if (nowMs - lastStatusPrintTimeMs >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusPrintTimeMs = nowMs;
    printPWMReference();
  }
}
