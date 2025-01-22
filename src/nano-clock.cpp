/*
 * Stable Clock Signal Generator using Timer1 on Arduino Nano
 * Ensures the clock signal is LOW when paused.
 */
#include <Arduino.h>

const uint32_t clockFrequency = 120; // Desired clock frequency in Hz

volatile bool isPaused = false;       // Tracks if the clock is paused
volatile bool singleStep = false;     // Tracks if single-step mode is activated

uint16_t prescaler = 1;               // Global variable to store the selected prescaler

void selectPrescaler(uint32_t clockFrequency, uint16_t &prescaler);
void configureTimer1();
uint16_t calculateTopValue(uint32_t clockFrequency, uint16_t prescaler);
void handleTopValueLimits(uint16_t &topValue);
uint8_t prescalerToBits(uint16_t prescaler);

void pauseResumeISR();                // ISR for Pause/Resume Button
void singleStepISR();                 // ISR for Single-Step Button

void setup() {
  pinMode(9, OUTPUT); // Pin 9 is connected to Timer1 (OC1A)
  pinMode(2, INPUT_PULLUP); // Single-Step Button (interrupt pin 3)
  pinMode(3, INPUT_PULLUP);

  configureTimer1();

  uint16_t topValue = 0;

  selectPrescaler(clockFrequency, prescaler);
  topValue = calculateTopValue(clockFrequency, prescaler);
  handleTopValueLimits(topValue);

  OCR1A = topValue; // Set the TOP value for Timer1

  // Attach interrupts to buttons
  attachInterrupt(digitalPinToInterrupt(2), pauseResumeISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), singleStepISR, FALLING);
}

void loop() {
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
