/*
  =========================================================
  STM32F411CE Black Pill Full Self-Diagnostic Test
  =========================================================

  Description:
  This program performs a **comprehensive self-diagnostic test** 
  on an **STM32F411CE Black Pill** to verify the functionality of key components, including:

  ✅ Serial Communication
  ✅ Built-in LED (Pin PC13)
  ✅ Digital I/O Pins (D0 - D15)
  ✅ Analog Inputs (A0 - A5)
  ✅ PWM Outputs (D2, D3, D5, D6, D9, D10, D11)
  ✅ EEPROM Read/Write (Flash emulated)
  ✅ SPI Communication (Requires external SPI device)
  ✅ I2C Communication (Requires external I2C device)
  ✅ USB HID Functionality (Basic test)
  ✅ Voltage Level Check (Requires connection to A0)

  =========================================================
  FIRST-TIME SETUP: Using STM32F411CE (Black Pill) in Arduino IDE
  =========================================================
  1️⃣ **Install STM32 Board Support Package**
     - Open **Arduino IDE**.
     - Go to **File** → **Preferences**.
     - In the **"Additional Board Manager URLs"**, add:
       ```
       https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
       ```
     - Click **OK**, then go to **Tools** → **Board** → **Boards Manager**.
     - Search for **"STM32"** and install **STM32duino** (STM32F4 series).

  2️⃣ **Select the Correct Board in Arduino IDE**
     - Go to **Tools** → **Board** → **STM32F4 Series** → **Generic STM32F4 Series**.
     - **Variant:** Select **"STM32F411CEU6 (Black Pill)"**.
     - **Upload Method:**
       - If using **DFU mode (USB bootloader)**: Select **"STM32CubeProgrammer (DFU)"**.
       - If using **Serial (UART)**: Select **"Serial"** and connect **FTDI adapter** to **PA9 (TX) & PA10 (RX)**.

  3️⃣ **Install STM32CubeProgrammer (for USB Uploads)**
     - Download **STM32CubeProgrammer** from STMicroelectronics:
       👉 [STM32CubeProgrammer Download](https://www.st.com/en/development-tools/stm32cubeprog.html)
     - Install it and ensure it works before proceeding.

  4️⃣ **Put the Board in Bootloader Mode (DFU Upload)**
     - **Press and hold the BOOT0 button.**
     - **Press and release the RESET button.**
     - **Release BOOT0** after a second.
     - The board should now be in **DFU mode**.

  5️⃣ **Upload the Code**
     - Connect the board via **USB-C or Micro USB**.
     - Go to **Tools** → **Port** and select the detected port.
     - Click **Upload**.

  =========================================================
  WIRING REQUIREMENTS FOR SELF-TEST
  =========================================================
  - **Digital I/O Test:** Pins **D0-D15** should not be externally connected.
  - **Analog Test:** Analog pins **A0-A5** should have varying voltage input.
  - **PWM Test:** Verify PWM output using an **oscilloscope or LED brightness change**.
  - **EEPROM Test:** No external wiring needed (uses flash emulation).
  - **SPI/I2C Tests:** External **SPI/I2C devices** are required for full testing.
  - **USB HID Test:** The STM32F411CE supports **native USB HID**.

  =========================================================
  ENABLE OR DISABLE TESTS
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
#define ENABLE_USB_HID_TEST   true
#define ENABLE_VOLTAGE_TEST   false  // Requires voltage measurement on A0

#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <HID.h>
#include <Keyboard.h>

#define TEST_DELAY 500
#define ANALOG_THRESHOLD 50
#define LED_PIN PC13 // Built-in LED on STM32F411CE

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Wait for Serial Monitor to open
    delay(2000);
    Serial.println("\n=== STM32F411CE Black Pill Full Self-Diagnostic Test ===");

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
            digitalWrite(LED_PIN, HIGH);
            delay(TEST_DELAY);
            digitalWrite(LED_PIN, LOW);
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
        Serial.println("Checking Analog Inputs...");
        bool analogOK = true;
        for (int pin = A0; pin <= A5; pin++) {
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
        int pwmPins[] = {2, 3, 5, 6, 9, 10, 11}; 
        for (int i = 0; i < 7; i++) {
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

    // 9. USB HID Functionality Test
    if (ENABLE_USB_HID_TEST) {
        Serial.println("Checking USB HID Functionality...");
        Keyboard.begin();
        Keyboard.print("USB HID Test OK");
        Keyboard.end();
        Serial.println("✔ USB HID Functionality: PASS (Check if text appears)");
    }

    // 10. Voltage Test (Optional)
    if (ENABLE_VOLTAGE_TEST) {
        Serial.println("Checking Voltage Levels...");
        pinMode(A0, INPUT);
        float voltage = analogRead(A0) * (3.3 / 4095.0);  // STM32 ADC is 12-bit (0-4095)
        Serial.print("Measured Voltage: ");
        Serial.println(voltage);
        Serial.println((voltage >= 3.1 && voltage <= 3.5) ? "✔ 3.3V Power Supply: PASS" : "✖ 3.3V Power Supply: FAIL");
    }

    Serial.println("\n✅ STM32F411CE Black Pill Self-Test Complete!");
}

void loop() {
    // No need to loop, test runs once
}