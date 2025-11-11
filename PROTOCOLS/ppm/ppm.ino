constexpr uint8_t PPM_INPUT_PIN = 7;
constexpr uint8_t PPM_CHANNELS = 6;

volatile uint16_t ppmValues[PPM_CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500};
volatile uint8_t currentChannel = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PPM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_INPUT_PIN), readPPM, RISING);
  Serial.println("PPM Reader Started");
}

void loop() {
  for (int i = 0; i < PPM_CHANNELS; i++) {
    Serial.print("CH");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(ppmValues[i]);
    Serial.print("\t");
  }
  Serial.println();

  delay(50);
}

void readPPM() {
  static unsigned long lastTime = 0;
  unsigned long now = micros();
  unsigned long pulseWidth = now - lastTime;
  lastTime = now;

  if (pulseWidth > 3000) {
    currentChannel = 0;
  } 

  else if (currentChannel < PPM_CHANNELS) {
    ppmValues[currentChannel++] = pulseWidth;
  }
}
