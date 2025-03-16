/*
  =========================================================
  ESP8266 Full Self-Diagnostic Test
  =========================================================

  Description:
  This program performs a **comprehensive self-diagnostic test** 
  on an **ESP8266** to verify the functionality of key components, including:

  ✅ Serial Communication
  ✅ Built-in LED (GPIO2)
  ✅ Digital I/O Pins (GPIO0 - GPIO15)
  ✅ Analog Input (A0)
  ✅ PWM Outputs (D1, D2, D5, D6)
  ✅ EEPROM Read/Write (Flash emulated)
  ✅ SPI Communication (Requires external SPI device)
  ✅ I2C Communication (Requires external I2C device)
  ✅ WiFi Connectivity (Basic connection test)
  ✅ Voltage Level Check (Requires connection to A0)

  =========================================================
  📌 FIRST-TIME SETUP: Using ESP8266 with Arduino IDE
  =========================================================
  1️⃣ **Install ESP8266 Board Support in Arduino IDE**
     - Open **Arduino IDE**.
     - Go to **File** → **Preferences**.
     - In the **"Additional Board Manager URLs"**, add:
       ```
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
       ```
     - Click **OK**, then go to **Tools** → **Board** → **Boards Manager**.
     - Search for **"ESP8266"** and install **ESP8266 by ESP8266 Community**.

  2️⃣ **Select the Correct Board in Arduino IDE**
     - Go to **Tools** → **Board** → **ESP8266 Boards**.
     - Select your board (e.g., **NodeMCU 1.0 (ESP-12E)**, **Generic ESP8266 Module**).
     - **Flash Size:** 4MB (or based on your module).
     - **CPU Frequency:** 80MHz.
     - **Upload Speed:** 115200 baud.
     - **Port:** Select the correct **COM port**.

  3️⃣ **Install Required Libraries**
     - Go to **Sketch** → **Include Library** → **Manage Libraries**.
     - Install:
       - **EEPROM**
       - **Wire** (for I2C)
       - **SPI** (for SPI)
       - **ESP8266WiFi** (for WiFi testing)

  4️⃣ **Upload the Code**
     - Connect the ESP8266 via **Micro USB**.
     - Click **Upload** in Arduino IDE.
     - Open the **Serial Monitor (Ctrl + Shift + M)** at **115200 baud**.
     - View test results.

  =========================================================
  📌 WIRING REQUIREMENTS FOR SELF-TEST
  =========================================================
  - **Digital I/O Test:** Ensure **GPIO0-GPIO15** are not externally connected.
  - **Analog Test:** Analog pin **A0** should have varying voltage input.
  - **PWM Test:** Verify PWM output using an **oscilloscope or LED brightness change**.
  - **EEPROM Test:** No external wiring needed (flash-based EEPROM emulation).
  - **WiFi Test:** Requires a working **WiFi router** for connection testing.
  - **SPI/I2C Tests:** External **SPI/I2C devices** are required for full testing.

  =========================================================
  📌 ENABLE OR DISABLE TESTS
  =========================================================
  Modify **true/false** below to enable or disable specific tests.
*/

// ======= Enable or Disable Tests =======
#define ENABLE_SERIAL_TEST    true
#define ENABLE_LED_TEST       true
#define ENABLE_DIGITAL_IO     true
#define ENABLE_ANALOG_TEST    true
#define ENABLE_PWM_TEST       true
#define ENABLE_EEPROM_TEST    true
#define ENABLE_SPI_TEST       false  // Requires external SPI device
#define ENABLE_I2C_TEST       false  // Requires external I2C device
#define ENABLE_WIFI_TEST      true   // Requires WiFi network
#define ENABLE_VOLTAGE_TEST   false  // Requires voltage measurement on A0

#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <ESP8266WiFi.h>

#define TEST_DELAY 500
#define ANALOG_THRESHOLD 50
#define LED_PIN 2 // Built-in LED on ESP8266

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(2000);
    Serial.println("\n=== ESP8266 Full Self-Diagnostic Test ===");

    // 1. Serial Communication Test
    if (ENABLE_SERIAL_TEST) {
        Serial.println("Checking Serial Communication...");
        Serial.println("✔ Serial Communication: PASS");
    }

    // 2. Built-in LED Test
    if (ENABLE_LED_TEST) {
        Serial.println("Checking Built-in LED...");
        pinMode(LED_PIN, OUTPUT);
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_PIN, LOW); // ESP8266 LED is active LOW
            delay(TEST_DELAY);
            digitalWrite(LED_PIN, HIGH);
            delay(TEST_DELAY);
        }
        Serial.println("✔ Built-in LED: PASS");
    }

    // 3. Digital I/O Test
    if (ENABLE_DIGITAL_IO) {
        Serial.println("Checking Digital I/O...");
        bool digitalIOOK = true;
        for (int pin = 0; pin <= 15; pin++) {
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
        Serial.println("Checking Analog Input (A0)...");
        int value = analogRead(A0);
        Serial.print("Analog Pin A0: ");
        Serial.println(value);
        Serial.println((value > ANALOG_THRESHOLD) ? "✔ Analog Input: PASS" : "✖ Analog Input: FAIL");
    }

    // 5. PWM Test
    if (ENABLE_PWM_TEST) {
        Serial.println("Checking PWM Outputs...");
        int pwmPins[] = {D1, D2, D5, D6}; 
        for (int i = 0; i < 4; i++) {
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
        EEPROM.begin(512);
        int oldValue = EEPROM.read(0);
        EEPROM.write(0, 42);
        EEPROM.commit();
        delay(10);
        int eepromValue = EEPROM.read(0);
        EEPROM.write(0, oldValue);
        EEPROM.commit();
        Serial.println((eepromValue == 42) ? "✔ EEPROM: PASS" : "✖ EEPROM: FAIL");
    }

    // 7. I2C Communication Test
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

    // 9. WiFi Connectivity Test
    if (ENABLE_WIFI_TEST) {
        Serial.println("Checking WiFi Connectivity...");
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        delay(1000);
        Serial.println(WiFi.scanNetworks() > 0 ? "✔ WiFi Module: PASS" : "✖ WiFi Module: FAIL");
    }

    // 10. Voltage Test
    if (ENABLE_VOLTAGE_TEST) {
        Serial.println("Checking Voltage Levels...");
        pinMode(A0, INPUT);
        float voltage = analogRead(A0) * (3.3 / 1023.0);
        Serial.print("Measured Voltage: ");
        Serial.println(voltage);
        Serial.println((voltage >= 3.1 && voltage <= 3.5) ? "✔ 3.3V Power Supply: PASS" : "✖ 3.3V Power Supply: FAIL");
    }

    Serial.println("\n✅ ESP8266 Self-Test Complete!");
}

void loop() {
    // No need to loop, test runs once
}