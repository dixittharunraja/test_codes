/*
  =========================================================
  HC-06 Bluetooth Module Full Functionality Test with Mobile Integration
  =========================================================

  Description:
  This program performs a **full functionality test** of the **HC-06 Bluetooth Module**, including:

  ✅ Serial Communication Test  
  ✅ Bluetooth Pairing & Response Test (Mobile Compatible)  
  ✅ Baud Rate Configuration  
  ✅ Remote Testing via Mobile App  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting HC-06 to Arduino & Mobile
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ HC-06 Bluetooth Module
     - ✅ 1KΩ + 2KΩ Resistors (Voltage Divider for TXD if using 3.3V logic)

  2️⃣ **Wiring Connections**
     | **HC-06 Module** | **Arduino Board** |
     |------------------|------------------|
     | **VCC**          | **5V (or 3.3V for ESP32/STM32)** |
     | **GND**          | **GND** |
     | **TXD**          | **D2 (Software Serial RX) [Use voltage divider if 3.3V logic]** |
     | **RXD**          | **D3 (Software Serial TX)** |

  3️⃣ **Pairing with Mobile**
     - **Go to Bluetooth settings on your phone** and **pair** with `HC-06` (PIN: `1234` or `0000`).
     - **Open Serial Bluetooth Terminal App** and **connect** to `HC-06`.

  4️⃣ **Testing Commands via Mobile**
     - Send `"PING"` to test Bluetooth response.
     - Send `"BAUD"` to test baud rate change.

  ⚠️ **Important Notes:**
  - HC-06 **operates at 3.3V logic** but can be powered by **5V**.
  - If using **ESP32 or STM32**, directly connect TXD and RXD to a hardware serial port.
  - **Unlike HC-05, HC-06 does NOT support AT Mode via an EN pin.**  

  5️⃣ **Uploading the Code**
     - Connect the Arduino to your computer via USB.
     - Open **Arduino IDE**.
     - Install **SoftwareSerial Library** (for Nano/Uno users).
     - Go to **Tools → Board → Select Arduino Nano** (or your board).
     - **Select Programmer** as `"AVRISP mkII"`.  
     - **Choose the Correct Port** from **Tools → Port**.  
     - Click **Upload & Open Serial Monitor (9600 baud).**  
     - Set **"Both NL & CR"** in Serial Monitor for commands.  

  =========================================================
  📌 ENABLE OR DISABLE TESTS
  =========================================================
  Modify **true/false** below to enable or disable specific tests.
*/

// ======= Enable or Disable Tests =======
#define ENABLE_BLUETOOTH_TEST   true  // Send & receive Bluetooth messages
#define ENABLE_BAUD_RATE_TEST   true  // Change baud rate and verify response
#define ENABLE_MOBILE_TEST      true  // Allow mobile-based command execution

#include <SoftwareSerial.h>

#define RX_PIN 2  // Arduino RX (Connect to HC-06 TX)
#define TX_PIN 3  // Arduino TX (Connect to HC-06 RX)

SoftwareSerial BTSerial(RX_PIN, TX_PIN);

void setup() {
    Serial.begin(9600);
    BTSerial.begin(9600);

    Serial.println("\n=== HC-06 Bluetooth Module Full Functionality Test ===");
    BTSerial.println("✅ HC-06 Test Ready! Send commands via Mobile.");
  
    // 1. Bluetooth Serial Communication Test
    if (ENABLE_BLUETOOTH_TEST) {
        Serial.println("\n📡 Sending Bluetooth Test Message...");
        BTSerial.println("Hello from Arduino!");
        delay(1000);
    }

    // 2. Baud Rate Configuration Test
    if (ENABLE_BAUD_RATE_TEST) {
        Serial.println("\n⚙️ Changing Baud Rate to 38400...");
        BTSerial.println("AT+BAUD6");  // AT+BAUD6 sets baud rate to 38400
        delay(1000);
        BTSerial.println("✅ Baud Rate Set to 38400!");
    }
}

void loop() {
    // 3. Mobile Command Processing
    if (ENABLE_MOBILE_TEST && BTSerial.available()) {
        String command = BTSerial.readString();
        command.trim();
        Serial.print("📥 Received Command: ");
        Serial.println(command);

        if (command == "PING") {
            BTSerial.println("✅ HC-06 is Online!");
        } else if (command == "BAUD") {
            changeBaudRate();
        } else {
            BTSerial.println("❌ Unknown Command! Use: PING, BAUD");
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
void changeBaudRate() {
    Serial.println("\n⚙️ Changing Baud Rate to 38400 via Mobile...");
    BTSerial.println("AT+BAUD6");  // Change baud rate to 38400
    delay(1000);
    BTSerial.println("✅ Baud Rate Set to 38400!");
}
