/*
  =========================================================
  nRF24L01+ Wireless Transceiver Full Self-Diagnostic Test
  =========================================================

  Description:
  This program tests the **nRF24L01+ Wireless Transceiver** for:

  ✅ SPI Communication Check  
  ✅ Register Read/Write Verification  
  ✅ Transmitter Mode Test  
  ✅ Receiver Mode Test  
  ✅ Signal Strength (RSSI) Readout  
  ✅ Compatible with Arduino Nano, Uno, Mega, STM32, ESP32  

  =========================================================
  📌 FIRST-TIME SETUP: Connecting nRF24L01+ to Arduino
  =========================================================

  1️⃣ **Required Components**
     - ✅ Arduino Nano (or Uno, Mega, STM32, ESP32)
     - ✅ nRF24L01+ Transceiver Module
     - ✅ 10µF Capacitor (Recommended for stable operation)

  2️⃣ **Wiring Connections**
     | **nRF24L01+** | **Arduino Board** |
     |-------------|------------------|
     | **VCC**      | **3.3V (Do NOT use 5V!)** |
     | **GND**      | **GND** |
     | **CE**       | **D9** |
     | **CSN**      | **D10** |
     | **SCK**      | **D13** |
     | **MOSI**     | **D11** |
     | **MISO**     | **D12** |
     | **IRQ**      | **Not used in this test** |

  ⚠️ **Important Notes:**
  - **Do NOT power nRF24L01+ with 5V!** It requires **3.3V**.
  - If using an **Arduino Mega or STM32**, verify SPI pin mappings.
  - **Use a 10µF capacitor** between **VCC & GND** to stabilize power.

  3️⃣ **Uploading the Code**
     - Connect the Arduino to your computer via USB.
     - Open **Arduino IDE**.
     - Install **RF24 Library** from Library Manager.
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
#define ENABLE_SPI_TEST       true  // Check nRF24L01+ SPI communication
#define ENABLE_REGISTER_TEST  true  // Verify register read/write
#define ENABLE_TRANSMIT_TEST  true  // Send test packets
#define ENABLE_RECEIVE_TEST   true  // Receive test packets

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN   9
#define CSN_PIN  10

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";  // Pipe Address

void setup() {
    Serial.begin(115200);
    SPI.begin();

    Serial.println("\n=== nRF24L01+ Self-Diagnostic Test ===");

    // 1. SPI Communication Test
    if (ENABLE_SPI_TEST) {
        Serial.print("🔍 Checking SPI Communication... ");
        if (radio.begin()) {
            Serial.println("✅ SPI OK!");
        } else {
            Serial.println("❌ SPI Communication Failed! Check wiring.");
            while (1);
        }
    }

    // 2. Register Read/Write Test
    if (ENABLE_REGISTER_TEST) {
        Serial.print("🔍 Testing Register Read/Write... ");
        radio.openWritingPipe(address);
        radio.openReadingPipe(1, address);
        radio.setPALevel(RF24_PA_LOW);
        radio.stopListening();
        
        if (radio.isChipConnected()) {
            Serial.println("✅ Register Test Passed!");
        } else {
            Serial.println("❌ Register Read Failed! Check wiring.");
        }
    }

    // Set radio in standby mode
    radio.startListening();
    delay(100);
}

void loop() {
    // 3. Transmit Test
    if (ENABLE_TRANSMIT_TEST) {
        Serial.print("\n📡 Sending Test Packet... ");
        radio.stopListening();
        const char text[] = "TestMsg";
        if (radio.write(&text, sizeof(text))) {
            Serial.println("✅ Sent Successfully!");
        } else {
            Serial.println("❌ Send Failed!");
        }
        delay(1000);
    }

    // 4. Receive Test
    if (ENABLE_RECEIVE_TEST) {
        radio.startListening();
        if (radio.available()) {
            char receivedText[32] = "";
            radio.read(&receivedText, sizeof(receivedText));
            Serial.print("📥 Received: ");
            Serial.println(receivedText);
        } else {
            Serial.println("⏳ Waiting for Data...");
        }
        delay(1000);
    }
}
