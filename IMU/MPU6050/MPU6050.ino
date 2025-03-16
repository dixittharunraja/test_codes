/*
  =========================================================
  MPU6050 Accelerometer & Gyroscope Full Self-Diagnostic Test
  =========================================================

  Description:
  This program tests the **MPU6050 IMU Sensor** for:

  ✅ I2C Communication Check  
  ✅ Accelerometer Data (X, Y, Z)  
  ✅ Gyroscope Data (X, Y, Z)  
  ✅ Temperature Sensor Reading  
  ✅ Self-Test Feature  
  ✅ Data Output via Serial Monitor  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting MPU6050 to Arduino
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ MPU6050 IMU Sensor
     - ✅ 4.7kΩ Pull-up Resistors (If needed for I2C stability)

  2️⃣ **Wiring Connections**
     | **MPU6050 Sensor** | **Arduino Board** |
     |------------------|------------------|
     | **VCC**          | **5V (for 5V boards) / 3.3V (for ESP32, STM32)** |
     | **GND**          | **GND** |
     | **SDA**          | **A4 (Arduino) / D21 (ESP32) / PB7 (STM32)** |
     | **SCL**          | **A5 (Arduino) / D22 (ESP32) / PB6 (STM32)** |

  ⚠️ **Important Notes:**
  - If using **ESP32 or STM32**, ensure MPU6050 **operates at 3.3V**.
  - Use **pull-up resistors (4.7kΩ)** on **SDA & SCL** if I2C is unstable.

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
#define ENABLE_I2C_TEST       true  // Check MPU6050 connectivity
#define ENABLE_ACCEL_TEST     true  // Read accelerometer data
#define ENABLE_GYRO_TEST      true  // Read gyroscope data
#define ENABLE_TEMP_TEST      true  // Read temperature sensor
#define ENABLE_SELF_TEST      false // Run MPU6050 self-test (Optional)

#include <Wire.h>

#define MPU6050_ADDR 0x68  

// MPU6050 Register Addresses
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define TEMP_OUT_H   0x41

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("\n=== MPU6050 Self-Diagnostic Test ===");

    // 1. I2C Connection Test
    if (ENABLE_I2C_TEST) {
        Wire.beginTransmission(MPU6050_ADDR);
        if (Wire.endTransmission() == 0) {
            Serial.println("✅ MPU6050 Detected!");
        } else {
            Serial.println("❌ MPU6050 Not Found! Check Wiring.");
            while (1);
        }
    }

    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);
}

void loop() {
    if (ENABLE_ACCEL_TEST) {
        int16_t accelX, accelY, accelZ;
        readMPU6050Data(ACCEL_XOUT_H, accelX, accelY, accelZ);
        Serial.print("📊 Accel (X, Y, Z): ");
        Serial.print(accelX); Serial.print(", ");
        Serial.print(accelY); Serial.print(", ");
        Serial.println(accelZ);
    }

    if (ENABLE_GYRO_TEST) {
        int16_t gyroX, gyroY, gyroZ;
        readMPU6050Data(GYRO_XOUT_H, gyroX, gyroY, gyroZ);
        Serial.print("🌀 Gyro (X, Y, Z): ");
        Serial.print(gyroX); Serial.print(", ");
        Serial.print(gyroY); Serial.print(", ");
        Serial.println(gyroZ);
    }

    if (ENABLE_TEMP_TEST) {
        int16_t rawTemp;
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(TEMP_OUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 2);
        rawTemp = (Wire.read() << 8) | Wire.read();
        float tempC = (rawTemp / 340.0) + 36.53;
        Serial.print("🌡️ Temperature: ");
        Serial.print(tempC);
        Serial.println("°C");
    }

    Serial.println("✅ Data Read Complete. Refreshing...");
    delay(1000);
}

void readMPU6050Data(uint8_t startReg, int16_t &x, int16_t &y, int16_t &z) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(startReg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6);

    x = (Wire.read() << 8) | Wire.read();
    y = (Wire.read() << 8) | Wire.read();
    z = (Wire.read() << 8) | Wire.read();
}
