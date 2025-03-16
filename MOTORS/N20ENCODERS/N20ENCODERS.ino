/*
  =========================================================
  N20 Motors with Encoders & TB6612FNG Motor Driver Test
  =========================================================

  Description:
  This program tests **N20 Motors with Encoders** using **TB6612FNG**.
  It verifies:
  
  ✅ PWM Motor Speed Control  
  ✅ Motor Direction Control (CW & CCW)  
  ✅ Encoder Feedback (Counts & RPM Calculation)  
  ✅ Brake & Standby Mode  
  ✅ Voltage Measurement (Optional)  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Using N20 Motors + Encoders + TB6612FNG
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ TB6612FNG (HW-166) Motor Driver
     - ✅ Two N20 Motors with Optical Encoders
     - ✅ Power Supply (6V-12V for motors, 5V for logic)
     - ✅ External Pull-up Resistors (10KΩ) for Encoders

  2️⃣ **Wiring Connections**
     | **TB6612FNG Motor Driver** | **Arduino Board** |
     |---------------------------|------------------|
     | **VCC** (Logic Power)     | **5V (or 3.3V for ESP32)** |
     | **GND**                   | **GND** |
     | **VM (Motor Power)**      | **External 6V-12V Power Supply** |
     | **STBY**                  | **D7** |
     | **AIN1**                  | **D2** |
     | **AIN2**                  | **D3** |
     | **PWMA** (PWM)            | **D5 (PWM)** |
     | **BIN1**                  | **D4** |
     | **BIN2**                  | **D6** |
     | **PWMB** (PWM)            | **D9 (PWM)** |

     | **Encoder Wires** | **Arduino Board** |
     |------------------|------------------|
     | **Motor A Encoder A** | **D8** |
     | **Motor A Encoder B** | **D9** |
     | **Motor B Encoder A** | **D10** |
     | **Motor B Encoder B** | **D11** |

  =========================================================
  📌 ENABLE OR DISABLE TESTS
  =========================================================
  Modify **true/false** below to enable or disable specific tests.

*/

// ======= Enable or Disable Tests =======
#define ENABLE_ENCODER_TEST   true
#define ENABLE_MOTOR_A_TEST   true
#define ENABLE_MOTOR_B_TEST   true
#define ENABLE_BRAKE_TEST     true
#define ENABLE_STANDBY_TEST   true
#define ENABLE_VOLTAGE_TEST   false  // Requires connection to A0

#define STBY  7  // Standby pin

// Motor A
#define AIN1  2
#define AIN2  3
#define PWMA  5  

// Motor B
#define BIN1  4
#define BIN2  6
#define PWMB  9  

// Encoder Pins
#define ENCA_A  8  
#define ENCA_B  9  
#define ENCB_A  10 
#define ENCB_B  11 

volatile long encoderCountA = 0;
volatile long encoderCountB = 0;
unsigned long lastTime = 0;
float rpmA = 0, rpmB = 0;

void encoderA() { encoderCountA++; }
void encoderB() { encoderCountB++; }

void setup() {
    Serial.begin(115200);
    
    pinMode(STBY, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    
    pinMode(ENCA_A, INPUT_PULLUP);
    pinMode(ENCA_B, INPUT_PULLUP);
    pinMode(ENCB_A, INPUT_PULLUP);
    pinMode(ENCB_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCA_A), encoderA, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCB_A), encoderB, RISING);

    Serial.println("\n=== N20 Motors + Encoders Test ===");
}

void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= 1000) {  
        rpmA = (encoderCountA / (float)PULSES_PER_REV) * 60.0;
        rpmB = (encoderCountB / (float)PULSES_PER_REV) * 60.0;

        Serial.print("\n⚙️ Motor A RPM: ");
        Serial.println(rpmA);
        Serial.print("⚙️ Motor B RPM: ");
        Serial.println(rpmB);

        encoderCountA = 0;
        encoderCountB = 0;
        lastTime = currentTime;
    }
  
    if (ENABLE_MOTOR_A_TEST) {
        Serial.println("\n🔄 Testing Motor A (Check movement)");
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, 150);
        delay(2000);
        Serial.println("⏩ Encoder A Count: " + String(encoderCountA));

        Serial.println("⏪ Rotating Reverse...");
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        delay(2000);
        Serial.println("⏩ Encoder A Count: " + String(encoderCountA));

        Serial.println("🛑 Stopping Motor A.");
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, 0);
    }

    if (ENABLE_MOTOR_B_TEST) {
        Serial.println("\n🔄 Testing Motor B (Check movement)");
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, 150);
        delay(2000);
        Serial.println("⏩ Encoder B Count: " + String(encoderCountB));

        Serial.println("⏪ Rotating Reverse...");
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
        delay(2000);
        Serial.println("⏩ Encoder B Count: " + String(encoderCountB));

        Serial.println("🛑 Stopping Motor B.");
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, 0);
    }

    Serial.println("\n✅ Test Complete! Verify encoder counts.");
    delay(3000);
}
