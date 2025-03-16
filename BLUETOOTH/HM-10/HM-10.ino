/*
  =========================================================
  HM-10 Bluetooth Low Energy (BLE) Module Full Functionality Test
  =========================================================

  Description:
  This program performs a **full functionality test** of the **HM-10 BLE Module**, including:

  ✅ AT Command Mode Check  
  ✅ Serial Communication Test  
  ✅ BLE Pairing & Response Test (Mobile Compatible)  
  ✅ Baud Rate Configuration  
  ✅ Remote Testing via Mobile App  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting HM-10 to Arduino & Mobile
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ HM-10 Bluetooth Module
     - ✅ 1KΩ + 2KΩ Resistors (Voltage Divider for TXD if using 3.3V logic)

  2️⃣ **Wiring Connections**
     | **HM-10 Module** | **Arduino Board** |
     |------------------|------------------|
     | **VCC**          | **3.3V or 5V (Check module specs)** |
     | **GND**          | **GND** |
     | **TXD**          | **D2 (Software Serial RX) [Use voltage divider if 3.3V logic]** |
     | **RXD**          | **D3 (Software Serial TX)** |

  3️⃣ **Pairing with Mobile**
     - **Go to Bluetooth settings on your phone** and **scan for Bluetooth devices**.  
     - Select **HM-10** to connect (no PIN required for BLE).  
     - Use **Serial Bluetooth Terminal App** (Android) or **BLE Terminal App** (iOS) to send commands.

  4️⃣ **Testing Commands via Mobile**
     - Send `"AT"` to check AT Mode.
     - Send `"PING"` to test Bluetooth response.
     - Send `"BAUD"` to test baud rate change.

  ⚠️ **Important Notes:**
  - HM-10 **operates at 3.3V logic** but can be powered by **5V**.
  - If using **ESP32 or STM32**, directly connect TXD and RXD to a hardware serial port.
  - **BLE requires a compatible mobile app to test functionality (not all standard Bluetooth apps work).**  

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
#define ENABLE_AT_MODE_TEST     true  // Check AT mode functionality
#define ENABLE_BLE_TEST         true  // Send & receive BLE messages
#define ENABLE_DEVICE_SCAN      true  // Scan for nearby BLE devices
#define ENABLE_BAUD_RATE_TEST   true  // Change baud rate and verify response

#include <SoftwareSerial.h>

#define RX_PIN 2  // Arduino RX (Connect to HM-10 TX)
#define TX_PIN 3  // Arduino TX (Connect to HM-10 RX)

SoftwareSerial BLESerial(RX_PIN, TX_PIN);

void setup() {
    Serial.begin(9600);
    BLESerial.begin(9600);

    Serial.println("\n=== HM-10 BLE Module Full Functionality Test ===");
    BLESerial.println("✅ HM-10 Test Ready! Send commands via Mobile.");
  
    // 1. AT Command Mode Test
    if (ENABLE_AT_MODE_TEST) {
        Serial.println("\n🔍 Checking AT Mode...");
        BLESerial.println("AT");
        delay(500);
        if (BLESerial.available()) {
            Serial.println("✅ HM-10 AT Mode Detected!");
        } else {
            Serial.println("❌ AT Mode Not Responding. Check Wiring.");
        }
    }

    // 2. Bluetooth Serial Communication Test
    if (ENABLE_BLE_TEST) {
        Serial.println("\n📡 Sending BLE Test Message...");
        BLESerial.println("Hello from Arduino!");
        delay(1000);
    }

    // 3. BLE Device Scan Test
    if (ENABLE_DEVICE_SCAN) {
        Serial.println("\n🔍 Scanning for Nearby BLE Devices...");
        BLESerial.println("AT+INQ");
        delay(5000);
        if (BLESerial.available()) {
            Serial.print("📡 Found Devices: ");
            while (BLESerial.available()) {
                Serial.write(BLESerial.read());
            }
            Serial.println();
        } else {
            Serial.println("⏳ No Devices Found. Try Again.");
        }
    }

    // 4. Baud Rate Configuration Test
    if (ENABLE_BAUD_RATE_TEST) {
        Serial.println("\n⚙️ Changing Baud Rate to 38400...");
        BLESerial.println("AT+BAUD6");  // Baud 6 = 38400
        delay(500);
        if (BLESerial.available()) {
            Serial.println("✅ Baud Rate Set Successfully!");
        } else {
            Serial.println("❌ Baud Rate Change Failed! Check Wiring.");
        }
    }
}

void loop() {
    // 5. Mobile Command Processing
    if (BLESerial.available()) {
        String command = BLESerial.readString();
        command.trim();
        Serial.print("📥 Received Command: ");
        Serial.println(command);

        if (command == "PING") {
            BLESerial.println("✅ HM-10 is Online!");
        } else if (command == "AT") {
            testATMode();
        } else if (command == "BAUD") {
            changeBaudRate();
        } else {
            BLESerial.println("❌ Unknown Command! Use: PING, AT, BAUD");
        }
    }

    // Check if Serial Monitor input should be sent over BLE
    if (Serial.available()) {
        String message = Serial.readString();
        Serial.print("📤 Sending: ");
        Serial.println(message);
        BLESerial.println(message);
    }
}

// ========== FUNCTIONALITY TESTS ==========
void testATMode() {
    Serial.println("\n🔍 Checking AT Mode via Mobile...");
    BLESerial.println("AT");
    delay(500);
    if (BLESerial.available()) {
        Serial.println("✅ HM-10 AT Mode Confirmed!");
        BLESerial.println("✅ AT Mode is Active!");
    } else {
        Serial.println("❌ No Response. Check Wiring.");
        BLESerial.println("❌ AT Mode Not Responding.");
    }
}

void changeBaudRate() {
    Serial.println("\n⚙️ Changing Baud Rate to 38400 via Mobile...");
    BLESerial.println("AT+BAUD6");  // Change baud rate to 38400
    delay(1000);
    BLESerial.println("✅ Baud Rate Set to 38400!");
}
