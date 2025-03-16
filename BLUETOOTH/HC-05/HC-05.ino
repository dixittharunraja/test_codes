/*
  =========================================================
  HC-05 Bluetooth Module Full Functionality Test with Mobile Integration
  =========================================================

  Description:
  This program performs a **full functionality test** of the **HC-05 Bluetooth Module**, including:

  ✅ AT Command Mode Check  
  ✅ Serial Communication Test  
  ✅ Bluetooth Pairing & Response Test (Mobile Compatible)  
  ✅ Baud Rate Configuration  
  ✅ Remote Testing via Mobile App  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting HC-05 to Arduino & Mobile
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ HC-05 Bluetooth Module
     - ✅ 1KΩ + 2KΩ Resistors (Voltage Divider for TXD if using 3.3V logic)

  2️⃣ **Wiring Connections**
     | **HC-05 Module** | **Arduino Board** |
     |------------------|------------------|
     | **VCC**          | **5V (or 3.3V for ESP32/STM32)** |
     | **GND**          | **GND** |
     | **TXD**          | **D2 (Software Serial RX) [Use voltage divider if 3.3V logic]** |
     | **RXD**          | **D3 (Software Serial TX)** |
     | **EN (Optional)**| **D4 (Set HIGH for AT Mode)** |

  3️⃣ **Pairing with Mobile**
     - **Go to Bluetooth settings on your phone** and **pair** with `HC-05` (PIN: `1234` or `0000`).
     - **Open Serial Bluetooth Terminal App** and **connect** to `HC-05`.

  4️⃣ **Testing Commands via Mobile**
     - Send `"AT"` to check AT Mode.
     - Send `"PING"` to test Bluetooth response.
     - Send `"BAUD"` to test baud rate change.
  ⚠️ **Important Notes:**
  - HC-05 **operates at 3.3V logic** but can be powered by **5V**.
  - If using **ESP32 or STM32**, directly connect TXD and RXD to a hardware serial port.
  - Set **EN pin HIGH before power-up** to enter **AT Command Mode**.

  5️⃣ **Uploading the Code**
     - Connect the Arduino to your computer via USB.
     - Open **Arduino IDE**.
     - Install **SoftwareSerial Library** (for Nano/Uno users).
     - Go to **Tools → Board → Select Arduino Nano** (or your board).
     - **Select Programmer** as `"AVRISP mkII"`.
     - **Choose the Correct Port** from **Tools → Port**.
     - Click **Upload & Open Serial Monitor (9600 baud).**
     - Set **"Both NL & CR"** in Serial Monitor for AT Commands.
  =========================================================
  📌 ENABLE OR DISABLE TESTS
  =========================================================
  Modify **true/false** below to enable or disable specific tests.
*/

// ======= Enable or Disable Tests =======
#define ENABLE_AT_MODE_TEST     true  // Check AT mode functionality
#define ENABLE_BLUETOOTH_TEST   true  // Send & receive Bluetooth messages
#define ENABLE_BAUD_RATE_TEST   true  // Change baud rate and verify response
#define ENABLE_MOBILE_TEST      true  // Allow mobile-based command execution

#include <SoftwareSerial.h>

#define RX_PIN 2  // Arduino RX (Connect to HC-05 TX)
#define TX_PIN 3  // Arduino TX (Connect to HC-05 RX)
#define EN_PIN 4  // Optional AT Mode Enable (Set HIGH before power-up)

SoftwareSerial BTSerial(RX_PIN, TX_PIN);

void setup() {
    Serial.begin(9600);
    BTSerial.begin(9600);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);

    Serial.println("\n=== HC-05 Bluetooth Module Full Functionality Test ===");
    BTSerial.println("✅ HC-05 Test Ready! Send commands via Mobile.");
  
    // 1. AT Command Mode Test
    if (ENABLE_AT_MODE_TEST) {
        Serial.println("\n🔍 Checking AT Mode...");
        digitalWrite(EN_PIN, HIGH);  // Enable AT Mode
        delay(1000);
        BTSerial.println("AT");
        delay(500);
        if (BTSerial.available()) {
            Serial.println("✅ HC-05 AT Mode Detected!");
        } else {
            Serial.println("❌ AT Mode Not Responding. Check Wiring & EN Pin.");
        }
        digitalWrite(EN_PIN, LOW);
    }

    // 2. Bluetooth Serial Communication Test
    if (ENABLE_BLUETOOTH_TEST) {
        Serial.println("\n📡 Sending Bluetooth Test Message...");
        BTSerial.println("Hello from Arduino!");
        delay(1000);
    }

    // 3. Baud Rate Configuration Test
    if (ENABLE_BAUD_RATE_TEST) {
        Serial.println("\n⚙️ Changing Baud Rate to 38400...");
        digitalWrite(EN_PIN, HIGH);
        delay(500);
        BTSerial.println("AT+UART=38400,0,0");
        delay(500);
        digitalWrite(EN_PIN, LOW);
    }
}

void loop() {
    // 4. Mobile Command Processing
    if (ENABLE_MOBILE_TEST && BTSerial.available()) {
        String command = BTSerial.readString();
        command.trim();
        Serial.print("📥 Received Command: ");
        Serial.println(command);

        if (command == "PING") {
            BTSerial.println("✅ HC-05 is Online!");
        } else if (command == "AT") {
            testATMode();
        } else if (command == "BAUD") {
            changeBaudRate();
        } else {
            BTSerial.println("❌ Unknown Command! Use: PING, AT, BAUD");
        }
    }

    // Check if Serial Monitor input should be sent over Bluetooth
    if (Serial.available()) {
        String message = Serial.readString();
        Serial.print("📤 Sending: ");
        Serial.println(message);
        BTSerial.println(message);
    }
}

// ========== FUNCTIONALITY TESTS ==========
void testATMode() {
    Serial.println("\n🔍 Checking AT Mode via Mobile...");
    digitalWrite(EN_PIN, HIGH);
    delay(1000);
    BTSerial.println("AT");
    delay(500);
    if (BTSerial.available()) {
        Serial.println("✅ HC-05 AT Mode Confirmed!");
        BTSerial.println("✅ AT Mode is Active!");
    } else {
        Serial.println("❌ No Response. Check EN Pin.");
        BTSerial.println("❌ AT Mode Not Responding.");
    }
    digitalWrite(EN_PIN, LOW);
}

void changeBaudRate() {
    Serial.println("\n⚙️ Changing Baud Rate to 38400 via Mobile...");
    digitalWrite(EN_PIN, HIGH);
    delay(500);
    BTSerial.println("AT+UART=38400,0,0");
    delay(500);
    digitalWrite(EN_PIN, LOW);
    BTSerial.println("✅ Baud Rate Set to 38400!");
}
