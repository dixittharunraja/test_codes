volatile long encoderCount = 0;
int lastEncoded = 0;

void setup() {
  pinMode(2, INPUT_PULLUP); // Channel A
  pinMode(3, INPUT_PULLUP); // Channel B
  
  attachInterrupt(digitalPinToInterrupt(2), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), updateEncoder, CHANGE);
  
  Serial.begin(9600);
}

void loop() {
  Serial.println(encoderCount);
  delay(100);
}

void updateEncoder() {
  int MSB = digitalRead(2); // Channel A
  int LSB = digitalRead(3); // Channel B
  
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderCount++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderCount--;
  
  lastEncoded = encoded;
}
