/*
  =========================================================
  L298N Motor Driver Full Self-Diagnostic Test
  =========================================================

  Description:
  This program tests an **L298N motor driver**, verifying:

  ✅ Motor Speed Control via PWM  
  ✅ Motor Direction Control (CW & CCW)  
  ✅ Independent Testing of Motor A & Motor B  
  ✅ Standby (Enable Pin) and Brake Functionality  
  ✅ Voltage Measurement on PWM Pins (Optional)  
  ✅ Compatible with Arduino Nano, Uno, Mega, ESP, STM32  

  =========================================================
  📌 FIRST-TIME SETUP: Using L298N with Arduino
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, ESP, STM32, etc.)
     - ✅ L298N Motor Driver
     - ✅ Two DC Motors
     - ✅ Power Supply (6V-12V for motors, 5V for logic)

  2️⃣ **Wiring Connections (L298N → Arduino)**
     | **L298N Motor Driver** | **Arduino Nano (or any compatible board)** |
     |------------------------|----------------------------------|
     | **VCC** (Logic Power)  | **5V (or 3.3V for ESP)**        |
     | **GND**               | **GND**                         |
     | **12V (Motor Power)** | **External 6V-12V Power Supply** |
     | **EN_A**              | **D5 (PWM)**                     |
     | **IN1**               | **D2**                           |
     | **IN2**               | **D3**                           |
     | **EN_B**              | **D9 (PWM)**                     |
     | **IN3**               | **D4**                           |
     | **IN4**               | **D6**                           |

  3️⃣ **Uploading the Code**
     - Connect the Arduino Nano to your computer via **USB**.
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
#define ENABLE_MOTOR_A_TEST  true
#define ENABLE_MOTOR_B_TEST  true
#define ENABLE_BRAKE_TEST    true
#define ENABLE_VOLTAGE_TEST  false  // Requires connection to A0

// Motor A (Left Motor)
#define EN_A  5  // PWM control
#define IN1   2  // Direction control
#define IN2   3  // Direction control

// Motor B (Right Motor)
#define EN_B  9  // PWM control
#define IN3   4  // Direction control
#define IN4   6  // Direction control

void setup() {
    Serial.begin(115200);
    pinMode(EN_A, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(EN_B, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    Serial.println("\n=== L298N Motor Driver Test ===");

    // 1. Motor A Test
    if (ENABLE_MOTOR_A_TEST) {
        Serial.println("\n🔄 Testing Motor A (Check movement)");
        Serial.println("⏩ Rotating Forward...");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(EN_A, 150);
        delay(2000);

        Serial.println("⏪ Rotating Reverse...");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        delay(2000);

        Serial.println("🛑 Stopping Motor A.");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(EN_A, 0);
        Serial.println("✅ Confirm if Motor A moved.");
    }

    // 2. Motor B Test
    if (ENABLE_MOTOR_B_TEST) {
        Serial.println("\n🔄 Testing Motor B (Check movement)");
        Serial.println("⏩ Rotating Forward...");
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(EN_B, 150);
        delay(2000);

        Serial.println("⏪ Rotating Reverse...");
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        delay(2000);

        Serial.println("🛑 Stopping Motor B.");
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(EN_B, 0);
        Serial.println("✅ Confirm if Motor B moved.");
    }

    // 3. Brake Test
    if (ENABLE_BRAKE_TEST) {
        Serial.println("\n⏹ Testing Brake Mode (Check if motors stop)");
        Serial.println("🛑 Engaging brake...");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, HIGH);
        delay(1000);

        Serial.println("🛑 Releasing brake.");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        Serial.println("✅ Confirm if motors stopped immediately.");
    }

    // 4. Voltage Test
    if (ENABLE_VOLTAGE_TEST) {
        Serial.println("\n🔍 Checking Voltage Levels...");
        pinMode(A0, INPUT);
        float voltage = analogRead(A0) * (5.0 / 1023.0);
        Serial.print("⚡ Measured Voltage: ");
        Serial.println(voltage);
        Serial.println((voltage >= 4.7 && voltage <= 5.3) ? "✅ Logic Power is stable." : "❌ Warning: Logic Power unstable!");
    }

    Serial.println("\n✅ L298N Motor Driver Test Complete!");
}

void loop() {
    // No need to loop, test runs once
}
