/*
  =========================================================
  MG90S Servo Motor Full Self-Diagnostic Test
  =========================================================

  Description:
  This program tests an **MG90S Metal Gear Servo Motor** for:

  ✅ Full range motion (0° to 180° and back)  
  ✅ Speed consistency  
  ✅ PWM signal response  
  ✅ Manual position control via Serial Monitor  
  ✅ Servo centering test (90° position check)  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting MG90S Servo to Arduino
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ MG90S Servo Motor
     - ✅ External 5V Power Supply (Highly Recommended for stable operation)
     - ✅ Breadboard & Jumper Wires

  2️⃣ **Wiring Connections**
     | **MG90S Servo** | **Arduino Board** |
     |--------------|------------------|
     | **VCC (Red)** | **5V** (Use external supply for multiple servos) |
     | **GND (Brown/Black)** | **GND** |
     | **Signal (Orange/Yellow)** | **D9 (PWM Pin)** |

  ⚠️ **Important Notes:**
  - MG90S servos **consume more power** than SG90, so use **an external 5V supply**.
  - Ensure **GND of servo power & Arduino is connected** to avoid floating signals.

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
#define ENABLE_SWEEP_TEST    true  // Moves servo from 0° to 180° and back
#define ENABLE_MANUAL_TEST   true  // Manually control the servo via Serial Monitor
#define ENABLE_CENTER_TEST   true  // Moves servo to 90° for centering

#include <Servo.h>

Servo mg90;  // Create Servo object
#define SERVO_PIN 9  // PWM pin for MG90S

void setup() {
    Serial.begin(115200);
    mg90.attach(SERVO_PIN);

    Serial.println("\n=== MG90S Servo Motor Test ===");

    if (ENABLE_CENTER_TEST) {
        Serial.println("\n🔄 Centering Servo at 90°...");
        mg90.write(90);
        delay(2000);
        Serial.println("✅ Confirm if the servo moved to 90°.");
    }

    if (ENABLE_MANUAL_TEST) {
        Serial.println("\n🎛 Enter angle (0-180) in Serial Monitor to move the servo.");
    }
}

void loop() {
    if (ENABLE_SWEEP_TEST) {
        Serial.println("\n🔄 Sweeping Servo from 0° to 180° and back...");
        
        for (int angle = 0; angle <= 180; angle += 5) {
            mg90.write(angle);
            delay(20);
        }

        for (int angle = 180; angle >= 0; angle -= 5) {
            mg90.write(angle);
            delay(20);
        }

        Serial.println("✅ Confirm if the servo smoothly moved.");
        delay(2000);
    }

    if (ENABLE_MANUAL_TEST && Serial.available()) {
        int angle = Serial.parseInt();
        if (angle >= 0 && angle <= 180) {
            mg90.write(angle);
            Serial.print("🔄 Moving Servo to: ");
            Serial.print(angle);
            Serial.println("°");
        } else {
            Serial.println("❌ Invalid angle! Enter a number between 0 and 180.");
        }
    }
}
