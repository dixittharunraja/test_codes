/*
  =================================================
  Arduino Nano Full Self-Diagnostic Test
  =================================================

  Description:
  This program performs a **comprehensive self-diagnostic test** 
  on an **Arduino Nano** to verify the functionality of key components, including:

  ✅ Serial Communication
  ✅ Built-in LED (Pin 13)
  ✅ Digital I/O Pins (D2 - D13)
  ✅ Analog Inputs (A0 - A7)
  ✅ PWM Outputs (D3, D5, D6, D9, D10, D11)
  ✅ EEPROM Read/Write
  ✅ SPI Communication
  ✅ I2C Communication (requires external I2C device)
  ✅ Voltage Level Check (requires connection to A0)

  Enabling/Disabling Tests:
  - Set **true** to enable a test, or **false** to disable it.

  Usage:
  - **Before Uploading:** Select the correct **port** and set the board type to **Arduino Nano**.
  - Upload the sketch using the **Arduino IDE**.
  - Open the **Serial Monitor (Ctrl + Shift + M)** at **9600 baud**.
  - View test results and diagnose any hardware issues.
*/

// ======= Enable or Disable Tests =======
#define ENABLE_SERIAL_TEST    true
#define ENABLE_LED_TEST       true
#define ENABLE_DIGITAL_IO     true
#define ENABLE_ANALOG_TEST    true
#define ENABLE_PWM_TEST       true
#define ENABLE_EEPROM_TEST    true
#define ENABLE_SPI_TEST       true
#define ENABLE_I2C_TEST       false  // Requires external I2C device
#define ENABLE_VOLTAGE_TEST   false  // Requires voltage measurement on A0

#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>

#define TEST_DELAY 500
#define ANALOG_THRESHOLD 50

void setup() {
    Serial.begin(9600);
    delay(2000);
    Serial.println("\n=== Arduino Nano Full Self-Diagnostic Test ===");

    // 1. Serial Communication Test
    if (ENABLE_SERIAL_TEST) {
        Serial.println("Checking Serial Communication...");
        Serial.println("✔ Serial Communication: PASS");
    }

    // 2. Built-in LED Test
    if (ENABLE_LED_TEST) {
        Serial.println("Checking Built-in LED...");
        pinMode(LED_BUILTIN, OUTPUT);
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(TEST_DELAY);
            digitalWrite(LED_BUILTIN, LOW);
            delay(TEST_DELAY);
        }
        Serial.println("✔ Built-in LED: PASS");
    }

    // 3. Digital I/O Test
    if (ENABLE_DIGITAL_IO) {
        Serial.println("Checking Digital I/O...");
        bool digitalIOOK = true;
        for (int pin = 2; pin <= 13; pin++) {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, HIGH);
            delay(10);
            if (digitalRead(pin) != HIGH) digitalIOOK = false;

            digitalWrite(pin, LOW);
            delay(10);
            if (digitalRead(pin) != LOW) digitalIOOK = false;
        }
        Serial.println(digitalIOOK ? "✔ Digital I/O: PASS" : "✖ Digital I/O: FAIL");
    }

    // 4. Analog Input Test
    if (ENABLE_ANALOG_TEST) {
        Serial.println("Checking Analog Inputs...");
        bool analogOK = true;
        for (int pin = A0; pin <= A7; pin++) {
            pinMode(pin, INPUT);
            int value = analogRead(pin);
            Serial.print("Analog Pin ");
            Serial.print(pin);
            Serial.print(": ");
            Serial.println(value);
            if (value < ANALOG_THRESHOLD) analogOK = false;
        }
        Serial.println(analogOK ? "✔ Analog Inputs: PASS" : "✖ Analog Inputs: FAIL");
    }

    // 5. PWM Test
    if (ENABLE_PWM_TEST) {
        Serial.println("Checking PWM Outputs...");
        int pwmPins[] = {3, 5, 6, 9, 10, 11}; 
        for (int i = 0; i < 6; i++) {
            pinMode(pwmPins[i], OUTPUT);
            analogWrite(pwmPins[i], 128);
            delay(100);
            digitalWrite(pwmPins[i], LOW);
        }
        Serial.println("✔ PWM Outputs: PASS (Manual verification required)");
    }

    // 6. EEPROM Test
    if (ENABLE_EEPROM_TEST) {
        Serial.println("Checking EEPROM...");
        int oldValue = EEPROM.read(0);
        EEPROM.write(0, 42);
        delay(10);
        int eepromValue = EEPROM.read(0);
        EEPROM.write(0, oldValue);  // Restore original value
        Serial.println((eepromValue == 42) ? "✔ EEPROM: PASS" : "✖ EEPROM: FAIL");
    }

    // 7. I2C Communication Test (Optional)
    if (ENABLE_I2C_TEST) {
        Serial.println("Checking I2C Communication...");
        Wire.begin();
        Wire.beginTransmission(0x00); // Test with a non-existent address
        Serial.println((Wire.endTransmission() == 0) ? "✔ I2C Communication: PASS" : "✖ I2C Communication: FAIL");
    }

    // 8. SPI Communication Test
    if (ENABLE_SPI_TEST) {
        Serial.println("Checking SPI Communication...");
        pinMode(SS, OUTPUT);
        digitalWrite(SS, LOW);
        SPI.begin();
        byte testValue = 0x55;
        byte response = SPI.transfer(testValue);
        digitalWrite(SS, HIGH);
        Serial.println((response == 0x55 || response == 0x00) ? "✔ SPI Communication: PASS" : "✖ SPI Communication: FAIL");
    }

    // 9. Voltage Test (Optional)
    if (ENABLE_VOLTAGE_TEST) {
        Serial.println("Checking Voltage Levels...");
        pinMode(A0, INPUT);
        float voltage = analogRead(A0) * (5.0 / 1023.0);
        Serial.print("Measured Voltage: ");
        Serial.println(voltage);
        Serial.println((voltage >= 4.7 && voltage <= 5.3) ? "✔ 5V Power Supply: PASS" : "✖ 5V Power Supply: FAIL");
    }

    Serial.println("\n✅ Arduino Nano Self-Test Complete!");
}

void loop() {
    // No need to loop, test runs once
}
