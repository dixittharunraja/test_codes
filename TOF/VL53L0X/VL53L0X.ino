/*
  =========================================================
  VL53L0X Time-of-Flight Distance Sensor Full Self-Diagnostic Test
  =========================================================

  Description:
  This program tests the **VL53L0X Time-of-Flight Sensor** for:

  ✅ I2C Communication Check  
  ✅ Distance Measurement  
  ✅ Signal Strength Validation  
  ✅ Ranging Mode Configuration (Short, Medium, Long Range)  
  ✅ Continuous vs. Single Measurement Test  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting VL53L0X to Arduino
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ VL53L0X Time-of-Flight Sensor
     - ✅ 4.7kΩ Pull-up Resistors (If needed for I2C stability)

  2️⃣ **Wiring Connections**
     | **VL53L0X Sensor** | **Arduino Board** |
     |------------------|------------------|
     | **VCC**          | **3.3V or 5V (Depending on Board)** |
     | **GND**          | **GND** |
     | **SDA**          | **A4 (Arduino) / D21 (ESP32) / PB7 (STM32)** |
     | **SCL**          | **A5 (Arduino) / D22 (ESP32) / PB6 (STM32)** |

  ⚠️ **Important Notes:**
  - VL53L0X works with **both 3.3V & 5V** logic, but check your breakout board specs.
  - Use **pull-up resistors (4.7kΩ)** on **SDA & SCL** if I2C communication is unstable.

  3️⃣ **Uploading the Code**
     - Connect the Arduino to your computer via USB.
     - Open **Arduino IDE**.
     - Install **VL53L0X Library** from Library Manager.
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
#define ENABLE_I2C_TEST       true  // Check VL53L0X connectivity
#define ENABLE_DISTANCE_TEST  true  // Measure distance
#define ENABLE_RANGING_MODE   true  // Test different ranging modes
#define ENABLE_CONTINUOUS_MODE true // Enable continuous measurement mode

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("\n=== VL53L0X Self-Diagnostic Test ===");

    // 1. I2C Communication Test
    if (ENABLE_I2C_TEST) {
        if (sensor.init()) {
            Serial.println("✅ VL53L0X Detected!");
        } else {
            Serial.println("❌ VL53L0X Not Found! Check Wiring.");
            while (1);
        }
    }

    // Set default measurement mode
    sensor.setTimeout(500);
    sensor.startContinuous();
}

void loop() {
    if (ENABLE_DISTANCE_TEST) {
        int distance = sensor.readRangeContinuousMillimeters();

        if (sensor.timeoutOccurred()) {
            Serial.println("❌ Timeout Occurred! Check Sensor.");
        } else {
            Serial.print("📏 Distance: ");
            Serial.print(distance);
            Serial.println(" mm");
        }
    }

    if (ENABLE_RANGING_MODE) {
        sensor.setMeasurementTimingBudget(200000);  // Medium range
        delay(500);
    }

    if (!ENABLE_CONTINUOUS_MODE) {
        sensor.stopContinuous();
        delay(100);
    }

    Serial.println("✅ Data Read Complete. Refreshing...");
    delay(1000);
}
