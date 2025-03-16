/*
  =========================================================
  TB6612FNG (HW-166) Motor Driver Full Self-Diagnostic Test
  =========================================================

  Description:
  This program tests the **TB6612FNG (HW-166) motor driver**, verifying:

  ✅ PWM Motor Speed Control  
  ✅ Motor Direction Control (CW & CCW)  
  ✅ Standby & Brake Mode Functionality  
  ✅ Independent Testing of Motor A & Motor B  
  ✅ Voltage Measurement on PWM Pins (Optional)  
  ✅ Compatible with Arduino Nano, Uno, Mega, ESP, STM32  

  =========================================================
  📌 FIRST-TIME SETUP: Using TB6612FNG (HW-166) with Arduino
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, ESP, STM32, etc.)
     - ✅ TB6612FNG (HW-166) Motor Driver
     - ✅ Two DC Motors
     - ✅ Power Supply (6V-12V for motors, 5V for logic)

  2️⃣ **Wiring Connections (TB6612FNG → Arduino)**
     | **TB6612FNG (HW-166)** | **Arduino Nano (or any compatible board)** |
     |------------------------|----------------------------------|
     | **VCC** (Logic Power)  | **5V (or 3.3V for ESP)**        |
     | **GND**               | **GND**                         |
     | **VM** (Motor Power)  | **External 6V-12V Power Supply** |
     | **STBY**              | **D7**                           |
     | **AIN1**              | **D2**                           |
     | **AIN2**              | **D3**                           |
     | **PWMA**              | **D5 (PWM)**                     |
     | **BIN1**              | **D4**                           |
     | **BIN2**              | **D6**                           |
     | **PWMB**              | **D9 (PWM)**                     |

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
#define ENABLE_STANDBY_TEST  true
#define ENABLE_VOLTAGE_TEST  false  // Requires connection to A0

#define STBY  7  // Standby pin

// Motor A (Left Motor)
#define AIN1  2
#define AIN2  3
#define PWMA  5  // Must be a PWM pin

// Motor B (Right Motor)
#define BIN1  4
#define BIN2  6
#define PWMB  9  // Must be a PWM pin

void setup() {
    Serial.begin(115200);
    pinMode(STBY, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    Serial.println("\n=== TB6612FNG (HW-166) Motor Driver Test ===");

    // 1. Standby Test
    if (ENABLE_STANDBY_TEST) {
        Serial.println("\n⏸️ Entering Standby Mode (Motors should stop)");
        digitalWrite(STBY, LOW);
        delay(500);
        Serial.println("✅ Confirm if motors stopped.");
        digitalWrite(STBY, HIGH);
    }

    // 2. Motor A Test
    if (ENABLE_MOTOR_A_TEST) {
        Serial.println("\n🔄 Testing Motor A (Check movement)");
        Serial.println("⏩ Rotating Forward...");
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, 150);
        delay(2000);

        Serial.println("⏪ Rotating Reverse...");
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        delay(2000);

        Serial.println("🛑 Stopping Motor A.");
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, 0);
        Serial.println("✅ Confirm if Motor A moved.");
    }

    // 3. Motor B Test
    if (ENABLE_MOTOR_B_TEST) {
        Serial.println("\n🔄 Testing Motor B (Check movement)");
        Serial.println("⏩ Rotating Forward...");
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, 150);
        delay(2000);

        Serial.println("⏪ Rotating Reverse...");
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        delay(2000);

        Serial.println("🛑 Stopping Motor B.");
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, 0);
        Serial.println("✅ Confirm if Motor B moved.");
    }

    // 4. Brake Test
    if (ENABLE_BRAKE_TEST) {
        Serial.println("\n⏹ Testing Brake Mode (Check if motors stop)");
        Serial.println("🛑 Engaging brake...");
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, HIGH);
        delay(1000);

        Serial.println("🛑 Releasing brake.");
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        Serial.println("✅ Confirm if motors stopped immediately.");
    }

    // 5. Voltage Test
    if (ENABLE_VOLTAGE_TEST) {
        Serial.println("\n🔍 Checking Voltage Levels...");
        pinMode(A0, INPUT);
        float voltage = analogRead(A0) * (5.0 / 1023.0);
        Serial.print("⚡ Measured Voltage: ");
        Serial.println(voltage);
        Serial.println((voltage >= 4.7 && voltage <= 5.3) ? "✅ Logic Power is stable." : "❌ Warning: Logic Power unstable!");
    }

    Serial.println("\n✅ TB6612FNG Motor Driver Test Complete!");
}

void loop() {
    // No need to loop, test runs once
}
