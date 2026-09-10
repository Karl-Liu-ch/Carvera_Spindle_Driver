// Arduino Nano R4 Minima - DFR1036 PWM to Analog Converter Mimic
// Reads 8-bit PWM value (0-255) and outputs 0-5V analog via DAC
// Matches DFR1036 scaling: value/255 * output_range

const int pwmInputPin = 2;     // Pin to read PWM signal - FROM CARVERA
const int dacOutputPin = DAC;  // DAC output on A0 - TO MAXON ESCON 50/5 SPEED INPUT
const int thresholdPin = 3;    // Digital output for threshold detection - TO MAXON ESCON 50/5 ENABLE/DISABLE INPUT

// PWM reading variables
volatile unsigned long pulseStartTime = 0;
volatile unsigned long pulseWidth = 0;
volatile unsigned long pulsePeriod = 0;
volatile bool newPulseAvailable = false;

// Smoothing
const int numReadings = 5;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

// Output mode (like DFR1036)
const float OUTPUT_VOLTAGE_MAX = 5.0;  // Change to 10.0 for 0-10V mode (requires external circuit)
const float THRESHOLD_VOLTAGE = 0.2;   // Voltage threshold to trigger digital output

void setup() {
  // Set DAC resolution to 12-bit
  analogWriteResolution(12);
  
  // Configure PWM input pin
  pinMode(pwmInputPin, INPUT);
  
  // Configure threshold output pin
  pinMode(thresholdPin, OUTPUT);
  digitalWrite(thresholdPin, LOW);
  
  // Attach interrupt to measure PWM
  attachInterrupt(digitalPinToInterrupt(pwmInputPin), pwmRising, RISING);
  
  // Initialize smoothing array
  for (int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
  
  setDACOutput(0);
}

void loop() {
  static unsigned long lastPulseTime = 0;
  
  if (newPulseAvailable) {
    noInterrupts();
    unsigned long pw = pulseWidth;
    unsigned long period = pulsePeriod;
    newPulseAvailable = false;
    interrupts();
    
    // Calculate duty cycle percentage (0-100%)
    if (period > 0) {
      float dutyCycle = ((float)pw / (float)period) * 100.0;
      dutyCycle = constrain(dutyCycle, 0.0, 100.0);
      
      // Convert duty cycle to 8-bit value (0-255) like DFR1036 expects
      int pwmValue = (int)((dutyCycle / 100.0) * 255.0);
      pwmValue = constrain(pwmValue, 0, 255);
      
      // Apply smoothing
      total = total - readings[readIndex];
      readings[readIndex] = pwmValue;
      total = total + readings[readIndex];
      readIndex = (readIndex + 1) % numReadings;
      average = total / numReadings;
      
      // Convert 8-bit value to voltage (DFR1036 scaling)
      float outputVolts = ((float)average / 255.0) * OUTPUT_VOLTAGE_MAX;
      
      // Convert voltage to 12-bit DAC value
      int dacValue = (int)((outputVolts / 5.0) * 4095.0);
      setDACOutput(dacValue);
      
      // Set threshold pin based on voltage
      if (outputVolts > THRESHOLD_VOLTAGE) {
        digitalWrite(thresholdPin, HIGH);
      } else {
        digitalWrite(thresholdPin, LOW);
      }
      
      lastPulseTime = millis();
    }
  }
  
  // Check for signal timeout (no PWM for 200ms = set to 0V)
  if (millis() - lastPulseTime > 200 && lastPulseTime > 0) {
    setDACOutput(0);
    digitalWrite(thresholdPin, LOW);
    lastPulseTime = 0;
  }
}

// Interrupt service routine - rising edge
void pwmRising() {
  unsigned long now = micros();
  pulsePeriod = now - pulseStartTime;
  pulseStartTime = now;
  attachInterrupt(digitalPinToInterrupt(pwmInputPin), pwmFalling, FALLING);
}

// Interrupt service routine - falling edge
void pwmFalling() {
  pulseWidth = micros() - pulseStartTime;
  newPulseAvailable = true;
  attachInterrupt(digitalPinToInterrupt(pwmInputPin), pwmRising, RISING);
}

/**
 * @brief Set DAC output value (mimics DFR1036)
 * @param value 12-bit DAC value (0-4095)
 */
void setDACOutput(int value) {
  value = constrain(value, 0, 4095);
  analogWrite(DAC, value);
}