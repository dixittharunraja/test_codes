/*
  =========================================================
  SG90 Servo Motor Full Self-Diagnostic Test
  =========================================================

  Description:
  This program tests an **SG90 Servo Motor** for:

  ✅ Full range motion (0° to 180° and back)  
  ✅ Speed consistency  
  ✅ PWM signal response  
  ✅ Manual position control via Serial Monitor  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting SG90 Servo to Arduino
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ SG90 Servo Motor
     - ✅ External 5V Power Supply (Recommended)
     - ✅ Breadboard & Jumper Wires

  2️⃣ **Wiring Connections**
     | **SG90 Servo** | **Arduino Board** |
     |--------------|------------------|
     | **VCC (Red)** | **5V** (Use external supply for multiple servos) |
     | **GND (Brown)** | **GND** |
     | **Signal (Orange)** | **D9 (PWM Pin)** |

  ⚠️ **Important Notes:**
  - If using multiple servos, power them from an **external 5V supply**.
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

Servo sg90;  // Create Servo object
#define SERVO_PIN 9  // PWM pin for SG90

void setup() {
    Serial.begin(115200);
    sg90.attach(SERVO_PIN);

    Serial.println("\n=== SG90 Servo Motor Test ===");

    if (ENABLE_CENTER_TEST) {
        Serial.println("\n🔄 Centering Servo at 90°...");
        sg90.write(90);
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
            sg90.write(angle);
            delay(20);
        }

        for (int angle = 180; angle >= 0; angle -= 5) {
            sg90.write(angle);
            delay(20);
        }

        Serial.println("✅ Confirm if the servo smoothly moved.");
        delay(2000);
    }

    if (ENABLE_MANUAL_TEST && Serial.available()) {
        int angle = Serial.parseInt();
        if (angle >= 0 && angle <= 180) {
            sg90.write(angle);
            Serial.print("🔄 Moving Servo to: ");
            Serial.print(angle);
            Serial.println("°");
        } else {
            Serial.println("❌ Invalid angle! Enter a number between 0 and 180.");
        }
    }
}
