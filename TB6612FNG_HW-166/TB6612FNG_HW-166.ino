/*
  TB6612FNG HW-166 Motor Driver Safe Test
  ---------------------------------------
  - The motor will remain STOPPED on startup for safety.
  - You must send a command to start it via the Serial Monitor.
  - Enter 'F' to move forward, 'B' to move backward, and 'S' to stop.
  - Use 'U' to increase speed and 'D' to decrease speed.

  Wiring (Arduino Nano to TB6612FNG HW-166):
  - PWMA  -> D9  (PWM control for Motor A)
  - AIN1  -> D4  (Direction control 1 A)
  - AIN2  -> D3  (Direction control 2 A)
  - PWMB  -> D10 (PWM control for Motor B)
  - BIN1  -> D6  (Direction control 1 B)
  - BIN2  -> D7  (Direction control 2 B)
  - STBY  -> D10 (Standby, must be HIGH to enable motors)
  - GND   -> Arduino GND
  - VCC   -> Arduino 5V
  - VM    -> External motor power supply (e.g., 9V or 12V)

  Written for Arduino Nano.
*/

#define AIN1 4   // Motor A direction pin 1
#define AIN2 3   // Motor A direction pin 2
#define PWMA 9   // Motor A PWM speed control
#define BIN1 6   // Motor B direction pin 1
#define BIN2 7   // Motor B direction pin 2
#define PWMB 10  // Motor B PWM speed control
#define STBY 5   // Standby pin

int speedValue = 25;  // Initial speed (0-255)
bool motorRunning = false;  // Motor state (OFF by default)

void setup() {
  Serial.begin(9600);
  
  // Set pin modes
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, LOW);  // Keep the motor in standby mode initially
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0); // Ensure motor is stopped
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0); // Ensure motor is stopped


  Serial.println("Motor is OFF. Enter 'S' to enable standby mode.");
  Serial.println("Enter 'F' for Forward, 'B' for Backward, 'S' to Stop.");
  Serial.println("Use 'U' to increase speed, 'D' to decrease speed.");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();  // Read command from Serial Monitor
    
    switch (command) {
      case 'S': // Enable standby mode (Allow motor operation)
        digitalWrite(STBY, HIGH);
        motorRunning = true;
        Serial.println("Motor is READY. Enter 'F' or 'B' to move.");
        break;
        
      case 'F': // Move forward (Only if motorRunning is true)
        if (motorRunning) {
          digitalWrite(AIN1, HIGH);
          digitalWrite(AIN2, LOW);
          analogWrite(PWMA, speedValue);
          digitalWrite(BIN1, HIGH);
          digitalWrite(BIN2, LOW);
          analogWrite(PWMB, speedValue);
          Serial.println("Motor moving FORWARD");
        } else {
          Serial.println("Enable standby mode first by entering 'S'.");
        }
        break;
        
      case 'B': // Move backward (Only if motorRunning is true)
        if (motorRunning) {
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, HIGH);
          analogWrite(PWMA, speedValue);
          digitalWrite(BIN1, LOW);
          digitalWrite(BIN2, HIGH);
          analogWrite(PWMB, speedValue);
          Serial.println("Motor moving BACKWARD");
        } else {
          Serial.println("Enable standby mode first by entering 'S'.");
        }
        break;
        
      case 'X': // Turn off standby mode and stop motor
        motorRunning = false;
        digitalWrite(STBY, LOW);
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        analogWrite(PWMA, 0);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        analogWrite(PWMB, 0);
        Serial.println("Motor is OFF. Enter 'S' to enable it again.");
        break;

      case 'U': // Increase speed
        if (motorRunning) {
          speedValue = constrain(speedValue + 25, 0, 255);
          Serial.print("Speed increased to: ");
          Serial.println(speedValue);
        } else {
          Serial.println("Enable standby mode first by entering 'S'.");
        }
        break;
        
      case 'D': // Decrease speed
        if (motorRunning) {
          speedValue = constrain(speedValue - 25, 0, 255);
          Serial.print("Speed decreased to: ");
          Serial.println(speedValue);
        } else {
          Serial.println("Enable standby mode first by entering 'S'.");
        }
        break;
        
      default:
        Serial.println("Invalid command! Use 'S', 'F', 'B', 'X', 'U', or 'D'.");
    }

    // Clear Serial buffer
    delay(10);
    while (Serial.available()) {
      Serial.read();
    }
  }
}
