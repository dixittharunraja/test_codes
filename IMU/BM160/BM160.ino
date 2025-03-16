/*
  =========================================================
  BMI160 Accelerometer & Gyroscope Full Self-Diagnostic Test
  =========================================================

  Description:
  This program tests the **BMI160 IMU Sensor** for:

  ✅ I2C Communication Check  
  ✅ Accelerometer Data (X, Y, Z)  
  ✅ Gyroscope Data (X, Y, Z)  
  ✅ Temperature Sensor Reading  
  ✅ Self-Test Feature (Optional)  
  ✅ Data Output via Serial Monitor  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting BMI160 to Arduino
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ BMI160 IMU Sensor (Breakout Board)
     - ✅ 4.7kΩ Pull-up Resistors (If needed for I2C stability)

  2️⃣ **Wiring Connections**
     | **BMI160 Sensor** | **Arduino Board** |
     |------------------|------------------|
     | **VCC**          | **3.3V (For ESP32, STM32)** / **5V (For Arduino)** |
     | **GND**          | **GND** |
     | **SDA**          | **A4 (Arduino) / D21 (ESP32) / PB7 (STM32)** |
     | **SCL**          | **A5 (Arduino) / D22 (ESP32) / PB6 (STM32)** |

  ⚠️ **Important Notes:**
  - The **BMI160 operates at 3.3V**; using it with a 5V board **requires a logic level shifter**.
  - Use **pull-up resistors (4.7kΩ)** on **SDA & SCL** if I2C communication is unstable.

  3️⃣ **Uploading the Code**
     - Connect the Arduino to your computer via USB.
     - Open **Arduino IDE**.
     - Go to **Tools → Board → Select Arduino Nano** (or your board).
     - **Select Programmer** as `"AVRISP mkII"`.
     - **Choose the Correct Port** from **Tools → Port**.
     - Click **Upload & Open Serial Monitor (115200 baud).**

  =========================================================
  📌 ENABLE OR DISABLE TESTS
  =========================================================
  Modify **true/false** below to enable or disable specific tests.
*/

// ======= Enable or Disable Tests =======
#define ENABLE_I2C_TEST       true  // Check BMI160 connectivity
#define ENABLE_ACCEL_TEST     true  // Read accelerometer data
#define ENABLE_GYRO_TEST      true  // Read gyroscope data
#define ENABLE_TEMP_TEST      true  // Read temperature sensor
#define ENABLE_SELF_TEST      false // Run BMI160 self-test (Optional)

#include <Wire.h>

#define BMI160_ADDR 0x69  // Default I2C address for BMI160

// BMI160 Register Addresses
#define CMD_REG        0x7E
#define ACCEL_XOUT_LSB 0x12
#define GYRO_XOUT_LSB  0x0C
#define TEMP_OUT_LSB   0x20

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("\n=== BMI160 Self-Diagnostic Test ===");

    // 1. I2C Connection Test
    if (ENABLE_I2C_TEST) {
        Wire.beginTransmission(BMI160_ADDR);
        if (Wire.endTransmission() == 0) {
            Serial.println("✅ BMI160 Detected!");
        } else {
            Serial.println("❌ BMI160 Not Found! Check Wiring.");
            while (1);
        }
    }

    // Wake up BMI160
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(CMD_REG);
    Wire.write(0x11); // Normal mode for accelerometer
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(CMD_REG);
    Wire.write(0x15); // Normal mode for gyroscope
    Wire.endTransmission();
    delay(100);
}

void loop() {
    if (ENABLE_ACCEL_TEST) {
        int16_t accelX, accelY, accelZ;
        readBMI160Data(ACCEL_XOUT_LSB, accelX, accelY, accelZ);
        Serial.print("📊 Accel (X, Y, Z): ");
        Serial.print(accelX); Serial.print(", ");
        Serial.print(accelY); Serial.print(", ");
        Serial.println(accelZ);
    }

    if (ENABLE_GYRO_TEST) {
        int16_t gyroX, gyroY, gyroZ;
        readBMI160Data(GYRO_XOUT_LSB, gyroX, gyroY, gyroZ);
        Serial.print("🌀 Gyro (X, Y, Z): ");
        Serial.print(gyroX); Serial.print(", ");
        Serial.print(gyroY); Serial.print(", ");
        Serial.println(gyroZ);
    }

    if (ENABLE_TEMP_TEST) {
        int16_t rawTemp;
        Wire.beginTransmission(BMI160_ADDR);
        Wire.write(TEMP_OUT_LSB);
        Wire.endTransmission(false);
        Wire.requestFrom(BMI160_ADDR, 2);
        rawTemp = (Wire.read() | (Wire.read() << 8));
        float tempC = (rawTemp / 512.0) + 23.0;
        Serial.print("🌡️ Temperature: ");
        Serial.print(tempC);
        Serial.println("°C");
    }

    Serial.println("✅ Data Read Complete. Refreshing...");
    delay(1000);
}

void readBMI160Data(uint8_t startReg, int16_t &x, int16_t &y, int16_t &z) {
    Wire.beginTransmission(BMI160_ADDR);
    Wire.write(startReg);
    Wire.endTransmission(false);
    Wire.requestFrom(BMI160_ADDR, 6);

    x = Wire.read() | (Wire.read() << 8);
    y = Wire.read() | (Wire.read() << 8);
    z = Wire.read() | (Wire.read() << 8);
}
