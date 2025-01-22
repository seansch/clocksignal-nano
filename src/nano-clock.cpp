/*
 * Stable Clock Signal Generator with Serial Console Commands
 * Commands: set [frequency], pause, resume, step, show
 */
#include <Arduino.h>

const uint32_t defaultFrequency = 10; // Default clock frequency in Hz
uint32_t clockFrequency = defaultFrequency;

volatile bool isPaused = false;       // Tracks if the clock is paused
volatile bool singleStep = false;     // Tracks if single-step mode is activated

uint16_t prescaler = 1;               // Global variable to store the selected prescaler

// Predefined frequencies for toggling
const uint32_t frequencies[] = {1, 10, 100, 1000};
const uint8_t numFrequencies = sizeof(frequencies) / sizeof(frequencies[0]);
uint8_t currentFrequencyIndex = 1;    // Start with 10 Hz

void selectPrescaler(uint32_t clockFrequency, uint16_t &prescaler);
void configureTimer1();
uint16_t calculateTopValue(uint32_t clockFrequency, uint16_t prescaler);
void handleTopValueLimits(uint16_t &topValue);
uint8_t prescalerToBits(uint16_t prescaler);
void processSerialCommand();          // Serial command processing

void pauseResumeISR();                // ISR for Pause/Resume Button
void singleStepISR();                 // ISR for Single-Step Button

void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT); // Pin 9 is connected to Timer1 (OC1A)
  pinMode(3, INPUT_PULLUP); // Single-Step Button
  pinMode(2, INPUT_PULLUP); // Pause/Resume Button
  pinMode(7, INPUT_PULLUP); // Frequency Toggle Button

  configureTimer1();

  uint16_t topValue = 0;

  selectPrescaler(clockFrequency, prescaler);
  topValue = calculateTopValue(clockFrequency, prescaler);
  handleTopValueLimits(topValue);

  OCR1A = topValue; // Set the TOP value for Timer1

  // Attach interrupts to buttons
  attachInterrupt(digitalPinToInterrupt(2), pauseResumeISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), singleStepISR, FALLING);

  Serial.println("Clock Generator Ready. Type 'help' for commands.");
}

void loop() {
  processSerialCommand();

  // Manage toggle button for frequency
  static bool lastToggleButtonState = HIGH; // Tracks the last state of the toggle button
  bool toggleButtonState = digitalRead(7); // Read the current state of the toggle button

  if (toggleButtonState == LOW && lastToggleButtonState == HIGH) {
    // Button press detected (falling edge)
    currentFrequencyIndex = (currentFrequencyIndex + 1) % numFrequencies;
    clockFrequency = frequencies[currentFrequencyIndex];

    selectPrescaler(clockFrequency, prescaler);
    uint16_t topValue = calculateTopValue(clockFrequency, prescaler);
    handleTopValueLimits(topValue);
    OCR1A = topValue;

    Serial.print("Toggled frequency to ");
    Serial.print(clockFrequency);
    Serial.println(" Hz.");
  }

  lastToggleButtonState = toggleButtonState; // Update the last state

  // Manage pause/resume state
  if (isPaused) {
    // Stop the timer
    TCCR1B &= ~(1 << CS10 | 1 << CS11 | 1 << CS12); // Disable clock
    TCCR1A &= ~(1 << COM1A0); // Disable toggle on compare match

    // Ensure the output pin is LOW
    digitalWrite(9, LOW);
  } else {
    // Resume timer with correct prescaler
    TCCR1B = (TCCR1B & ~(1 << CS10 | 1 << CS11 | 1 << CS12)) | prescalerToBits(prescaler);
    TCCR1A |= (1 << COM1A0); // Enable toggle on compare match
  }

  // Handle single-step mode
  if (singleStep) {
    singleStep = false; // Clear the flag

    // Enable the timer for one pulse
    TCCR1B = (TCCR1B & ~(1 << CS10 | 1 << CS11 | 1 << CS12)) | prescalerToBits(prescaler);
    TCCR1A |= (1 << COM1A0); // Ensure toggle is enabled

    // Wait for one full timer cycle (topValue represents the duration of one half-cycle)
    delayMicroseconds((OCR1A + 1) * 2 * prescaler / 16); // Adjust for prescaler and clock

    // Stop the timer
    TCCR1B &= ~(1 << CS10 | 1 << CS11 | 1 << CS12); // Disable clock
    TCCR1A &= ~(1 << COM1A0); // Disable toggle

    // Ensure the output pin is LOW after the pulse
    digitalWrite(9, LOW);
  }
}

/**
 * @brief Process serial commands from the console.
 */
void processSerialCommand() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove leading/trailing whitespace

    if (command.startsWith("set ")) {
      // Set the frequency
      uint32_t freq = command.substring(4).toInt();
      if (freq > 0 && freq <= 1000000) {
        clockFrequency = freq;
        selectPrescaler(clockFrequency, prescaler);
        uint16_t topValue = calculateTopValue(clockFrequency, prescaler);
        handleTopValueLimits(topValue);
        OCR1A = topValue;
        Serial.print("Frequency set to ");
        Serial.print(clockFrequency);
        Serial.println(" Hz.");
      } else {
        Serial.println("Invalid frequency. Must be between 1 and 1,000,000 Hz.");
      }
    } else if (command == "pause") {
      isPaused = true;
      Serial.println("Clock paused.");
    } else if (command == "resume") {
      isPaused = false;
      Serial.println("Clock resumed.");
    } else if (command == "step") {
      singleStep = true;
      Serial.println("Single pulse triggered.");
    } else if (command == "show") {
      Serial.print("Current frequency: ");
      Serial.print(clockFrequency);
      Serial.println(" Hz.");
    } else if (command == "help") {
      Serial.println("Commands:");
      Serial.println("  set [frequency] - Set clock frequency in Hz (1 to 1,000,000)");
      Serial.println("  pause           - Pause the clock");
      Serial.println("  resume          - Resume the clock");
      Serial.println("  step            - Generate a single pulse");
      Serial.println("  show            - Show current frequency");
    } else {
      Serial.println("Unknown command. Type 'help' for a list of commands.");
    }
  }
}

/**
 * @brief ISR for Pause/Resume Button
 * Toggles the clock signal between paused and running states.
 */
void pauseResumeISR() {
  static unsigned long lastDebounceTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > 50) { // Debounce delay
    isPaused = !isPaused;
    lastDebounceTime = currentTime;
  }
}

/**
 * @brief ISR for Single-Step Button
 * Activates single-step mode to generate one clock pulse.
 */
void singleStepISR() {
  static unsigned long lastDebounceTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > 50) { // Debounce delay
    singleStep = true;
    lastDebounceTime = currentTime;
  }
}

/**
 * @brief Selects the appropriate prescaler for Timer1 based on the given clock frequency.
 */
void selectPrescaler(uint32_t clockFrequency, uint16_t &prescaler) {
  TCCR1B &= ~(1 << CS10 | 1 << CS11 | 1 << CS12);

  if (clockFrequency > 1000) {
    prescaler = 1;
  } else if (clockFrequency > 125) {
    prescaler = 8;
  } else if (clockFrequency > 15) {
    prescaler = 64;
  } else if (clockFrequency > 2) {
    prescaler = 256;
  } else {
    prescaler = 1024;
  }
}

/**
 * @brief Converts a prescaler value into TCCR1B bit settings.
 */
uint8_t prescalerToBits(uint16_t prescaler) {
  switch (prescaler) {
    case 1: return (1 << CS10);
    case 8: return (1 << CS11);
    case 64: return (1 << CS11) | (1 << CS10);
    case 256: return (1 << CS12);
    case 1024: return (1 << CS12) | (1 << CS10);
    default: return 0;
  }
}

/**
 * @brief Configures Timer1 for CTC (Clear Timer on Compare Match) mode.
 */
void configureTimer1() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12); // CTC mode
}

/**
 * @brief Calculates the TOP value for Timer1 based on the given clock frequency and prescaler.
 */
uint16_t calculateTopValue(uint32_t clockFrequency, uint16_t prescaler) {
  uint32_t topValue = (16000000 / (2UL * prescaler * clockFrequency)) - 1;
  return (topValue > 65535) ? 65535 : static_cast<uint16_t>(topValue);
}

/**
 * @brief Handles the limits of the TOP value for Timer1.
 */
void handleTopValueLimits(uint16_t &topValue) {
  if (topValue < 1) {
    topValue = 1;
  } else if (topValue > 65535) {
    topValue = 65535;
  }
}
