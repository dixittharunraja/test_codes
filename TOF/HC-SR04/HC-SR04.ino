/*
  =========================================================
  HC-SR04 Ultrasonic Sensor Full Self-Diagnostic Test
  =========================================================

  Description:
  This program tests the **HC-SR04 Ultrasonic Distance Sensor** for:

  ✅ Basic Functionality & Wiring Check  
  ✅ Distance Measurement in cm & inches  
  ✅ Signal Response Time Analysis  
  ✅ Error Handling (Timeout & No Echo Detection)  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting HC-SR04 to Arduino
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ HC-SR04 Ultrasonic Sensor
     - ✅ 330Ω & 470Ω Resistors (If using 5V logic with ESP32/STM32)

  2️⃣ **Wiring Connections**
     | **HC-SR04 Sensor** | **Arduino Board** |
     |------------------|------------------|
     | **VCC**          | **5V (or 3.3V with level shifter for ESP32)** |
     | **GND**          | **GND** |
     | **Trig**         | **D9** |
     | **Echo**         | **D10** (Use voltage divider for ESP32/STM32) |

  ⚠️ **Important Notes:**
  - The **HC-SR04 operates at 5V**. If using **ESP32 or STM32**, use a **voltage divider** on the Echo pin.
  - For **reliable readings**, ensure the **sensor is positioned correctly** and **avoid obstructions**.

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
#define ENABLE_DISTANCE_TEST  true  // Measure distance in cm & inches
#define ENABLE_RESPONSE_TIME  true  // Measure response time of the sensor
#define ENABLE_ERROR_CHECK    true  // Detect no echo or timeout errors

#define TRIG_PIN 9
#define ECHO_PIN 10

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    Serial.println("\n=== HC-SR04 Self-Diagnostic Test ===");
}

void loop() {
    long duration;
    float distance_cm, distance_in;
    
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (max ~500cm)

    if (ENABLE_ERROR_CHECK && duration == 0) {
        Serial.println("❌ No Echo Received! Check wiring or object distance.");
    } else {
        distance_cm = duration * 0.0343 / 2;
        distance_in = distance_cm * 0.3937;

        if (ENABLE_DISTANCE_TEST) {
            Serial.print("📏 Distance: ");
            Serial.print(distance_cm);
            Serial.print(" cm (");
            Serial.print(distance_in);
            Serial.println(" inches)");
        }

        if (ENABLE_RESPONSE_TIME) {
            Serial.print("⏱ Response Time: ");
            Serial.print(duration);
            Serial.println(" µs");
        }
    }

    Serial.println("✅ Data Read Complete. Refreshing...");
    delay(1000);
}
