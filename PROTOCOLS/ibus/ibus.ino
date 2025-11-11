constexpr uint8_t IBUS_FRAME_LENGTH = 32;
constexpr uint8_t IBUS_MAX_CHANNELS = 14;

constexpr uint8_t IBUS_RX_PIN = PA10;
constexpr uint8_t IBUS_TX_PIN = PA9;

uint8_t  ibus_buffer[IBUS_FRAME_LENGTH];
uint16_t ibus_channels[IBUS_MAX_CHANNELS];

void setup() {
  Serial.begin(115200);      // USB output
  Serial1.setRx(IBUS_RX_PIN);
  Serial1.setTx(IBUS_TX_PIN);
  Serial1.begin(115200);     // iBUS input
  delay(100);

  Serial.println("STM32 iBUS Reader Started");
}

void loop() {
  readIBUS();
  for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(ibus_channels[i]);
    Serial.print("\t");
  }
  Serial.println();

  delay(10);
}

void readIBUS() {
  while (Serial1.available()) {
    if (Serial1.peek() == 0x20) {
      if (Serial1.available() >= IBUS_FRAME_LENGTH) {
        Serial1.readBytes(ibus_buffer, IBUS_FRAME_LENGTH);
        for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
          ibus_channels[i] = ibus_buffer[2 + i*2] | (ibus_buffer[3 + i*2] << 8);
        }
      }
    } else {
      Serial1.read();
    }
  }
}
