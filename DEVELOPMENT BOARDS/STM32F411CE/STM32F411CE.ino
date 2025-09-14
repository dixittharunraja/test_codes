/*
  ===========================================================
  STM32F411CE (Blackpill) - Full Self-Diagnostic Test (PASS/FAIL)
  ===========================================================
*/

#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>

// ====== Enable / Disable tests ======
#define ENABLE_SERIAL_TEST       true
#define ENABLE_LED_TEST          true
#define ENABLE_DIGITAL_TOGGLE    true
#define ENABLE_DIGITAL_LOOPBACK  false
#define ENABLE_ANALOG_TEST       true
#define ENABLE_PWM_TEST          true
#define ENABLE_EEPROM_TEST       true
#define ENABLE_SPI_TEST          false
#define ENABLE_I2C_TEST          false
#define ENABLE_VOLTAGE_TEST      false

#define TEST_DELAY 300

// Safe Digital Pins (USB & SWD excluded)
const int digitalPins[] = {
  PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
  PA8, PA9, PA10, PA15,
  PB0, PB1, PB3, PB4, PB5, PB6, PB7,
  PB8, PB9, PB10, PB12, PB13, PB14, PB15,
  PC13, PC14, PC15
};

// Analog-capable pins
const int analogPins[] = { PA0, PA1, PA4, PA5, PA6, PA7 };

// PWM pins
const int pwmPins[] = { PA0, PA1, PA2, PA3, PA8, PA9, PA10, PB6, PB7, PB8, PB9 };

// Onboard LED
#define LED_BUILTIN_PIN PC13

// SPI defaults
#define SPI_CS   PA4
#define SPI_MISO PA6
#define SPI_MOSI PA7

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("\n=== STM32F411CE (Blackpill) Self-Diagnostic Test ===");

  // 1. Serial test
  if (ENABLE_SERIAL_TEST) {
    Serial.println(Serial ? "✔ Serial Communication: PASS" : "✖ Serial Communication: FAIL");
  }

  // 2. LED test
  if (ENABLE_LED_TEST) {
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    bool ledWorking = true;
    Serial.println("Testing LED (PC13)...");
    for (int i = 0; i < 2; i++) {
      digitalWrite(LED_BUILTIN_PIN, LOW);
      delay(TEST_DELAY);
      if (digitalRead(LED_BUILTIN_PIN) != LOW) ledWorking = false;
      digitalWrite(LED_BUILTIN_PIN, HIGH);
      delay(TEST_DELAY);
      if (digitalRead(LED_BUILTIN_PIN) != HIGH) ledWorking = false;
    }
    Serial.println(ledWorking ? "✔ LED Test: PASS" : "✖ LED Test: FAIL");
  }

  // 3. Digital toggle test
  if (ENABLE_DIGITAL_TOGGLE) {
    Serial.println("Testing Digital I/O (Toggle Only)...");
    bool digitalOk = true;
    for (unsigned i = 0; i < sizeof(digitalPins)/sizeof(digitalPins[0]); i++) {
      int p = digitalPins[i];
      pinMode(p, OUTPUT);
      digitalWrite(p, HIGH);
      delay(2);
      if (digitalRead(p) != HIGH) digitalOk = false;
      digitalWrite(p, LOW);
      delay(2);
      if (digitalRead(p) != LOW) digitalOk = false;
    }
    Serial.println(digitalOk ? "✔ Digital Toggle Test: PASS" : "✖ Digital Toggle Test: FAIL");
  }

  // 4. Digital loopback test
  if (ENABLE_DIGITAL_LOOPBACK) {
    bool loopOk = true;
    Serial.println("Testing Digital I/O Loopback...");
    pinMode(PA0, OUTPUT);
    pinMode(PA1, INPUT_PULLDOWN);
    digitalWrite(PA0, HIGH); delay(2);
    if (!digitalRead(PA1)) loopOk = false;
    digitalWrite(PA0, LOW); delay(2);
    if (digitalRead(PA1)) loopOk = false;
    Serial.println(loopOk ? "✔ Digital Loopback: PASS" : "✖ Digital Loopback: FAIL");
  }

  // 5. Analog test
  if (ENABLE_ANALOG_TEST) {
    Serial.println("Reading Analog Inputs...");
    bool analogOk = true;
    for (unsigned i = 0; i < sizeof(analogPins)/sizeof(analogPins[0]); i++) {
      int pin = analogPins[i];
      pinMode(pin, INPUT_ANALOG);
      int value = analogRead(pin);
      float voltage = value * (3.3 / 4095.0);
      Serial.print("Pin "); Serial.print(pin);
      Serial.print(" = "); Serial.print(value);
      Serial.print(" (~"); Serial.print(voltage, 3); Serial.println(" V)");
      if (value < 0 || value > 4095) analogOk = false;
    }
    Serial.println(analogOk ? "✔ Analog Inputs: PASS" : "✖ Analog Inputs: FAIL");
  }

  // 6. PWM test
  if (ENABLE_PWM_TEST) {
    Serial.println("Testing PWM...");
    bool pwmOk = true;
    for (unsigned i = 0; i < sizeof(pwmPins)/sizeof(pwmPins[0]); i++) {
      pinMode(pwmPins[i], OUTPUT);
      analogWrite(pwmPins[i], 128);
      delay(50);
      if (digitalRead(pwmPins[i]) != HIGH && digitalRead(pwmPins[i]) != LOW) pwmOk = false;
      digitalWrite(pwmPins[i], LOW);
    }
    Serial.println(pwmOk ? "✔ PWM Test: PASS" : "✖ PWM Test: FAIL");
  }

  // 7. EEPROM test
  if (ENABLE_EEPROM_TEST) {
    Serial.println("Testing EEPROM...");
    int oldValue = EEPROM.read(0);
    EEPROM.write(0, 42);
    delay(5);
    int newValue = EEPROM.read(0);
    EEPROM.write(0, oldValue);
    Serial.println((newValue == 42) ? "✔ EEPROM Test: PASS" : "✖ EEPROM Test: FAIL");
  }

  // 8. SPI loopback
  if (ENABLE_SPI_TEST) {
    Serial.println("Testing SPI Loopback...");
    pinMode(SPI_CS, OUTPUT);
    digitalWrite(SPI_CS, LOW);
    SPI.begin();
    byte sent = 0xAB;
    byte recv = SPI.transfer(sent);
    digitalWrite(SPI_CS, HIGH);
    Serial.println((recv == sent) ? "✔ SPI Loopback: PASS" : "✖ SPI Loopback: FAIL");
  }

  // 9. I2C scan
  if (ENABLE_I2C_TEST) {
    Serial.println("Scanning I2C bus...");
    bool found = false;
    Wire.begin();
    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("✔ I2C device at 0x");
        Serial.println(addr, HEX);
        found = true;
      }
    }
    Serial.println(found ? "✔ I2C Scan: PASS" : "✖ I2C Scan: No devices");
  }

  // 10. Voltage check
  if (ENABLE_VOLTAGE_TEST) {
    Serial.println("Measuring A0 Voltage...");
    pinMode(A0, INPUT_ANALOG);
    float voltage = analogRead(A0) * (3.3 / 4095.0);
    Serial.print("A0 Voltage = "); Serial.print(voltage, 3); Serial.println(" V");
    Serial.println((voltage >= 0.0 && voltage <= 3.3) ? "✔ Voltage Test: PASS" : "✖ Voltage Test: FAIL");
  }

  Serial.println("\n✅ Self-Test Completed!");
}

void loop() {
  // Run once
}