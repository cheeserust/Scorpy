/*
 * TB6600 power-on motor test.
 *
 * Upload this sketch, then the motor starts spinning as soon as the Arduino
 * and TB6600 motor power are applied.
 *
 * Wiring used by this test:
 * - TB6600 PUL+ -> Arduino D6
 * - TB6600 PUL- -> Arduino GND
 * - TB6600 DIR+ -> Arduino D7
 * - TB6600 DIR- -> Arduino GND
 * - TB6600 ENA+ -> Arduino D8
 * - TB6600 ENA- -> Arduino GND
 * - Limit switch -> Arduino D3 and GND
 *
 * Many TB6600 modules are enabled when ENA is not driven. This sketch keeps
 * D8 LOW so the ENA optocoupler is off with the wiring above.
 */

#include <Arduino.h>

static const uint8_t PIN_STEP = 6;
static const uint8_t PIN_DIR = 7;
static const uint8_t PIN_ENABLE = 8;
static const uint8_t PIN_LIMIT = 3;
static const uint8_t PIN_LED = LED_BUILTIN;

static const bool ENABLE_ACTIVE_LOW = true;
static const bool DIR_LEVEL = HIGH;

static const uint16_t STEP_HIGH_US = 5;
static const uint16_t STEP_LOW_US = 800;

static void setDriverEnabled(bool enabled)
{
    if (ENABLE_ACTIVE_LOW) {
        digitalWrite(PIN_ENABLE, enabled ? LOW : HIGH);
    } else {
        digitalWrite(PIN_ENABLE, enabled ? HIGH : LOW);
    }
}

static void pulseOneStep()
{
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(STEP_HIGH_US);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(STEP_LOW_US);
}

static void updateLimitLed()
{
    bool limitPressed = digitalRead(PIN_LIMIT) == LOW;
    digitalWrite(PIN_LED, limitPressed ? LOW : HIGH);
}

void setup()
{
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_LIMIT, INPUT_PULLUP);
    pinMode(PIN_LED, OUTPUT);

    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR, DIR_LEVEL);
    updateLimitLed();
    setDriverEnabled(true);
}

void loop()
{
    updateLimitLed();
    pulseOneStep();
}
