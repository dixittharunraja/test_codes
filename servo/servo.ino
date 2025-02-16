/*
  Servo Motor Control using Arduino Nano
  --------------------------------------
  - This program allows you to control a servo motor via the Serial Monitor.
  - Enter an angle (0 to 180) in the Serial Monitor and press ENTER.
  - The servo will move to the entered angle.
  - If an invalid input is detected, it will prompt the user to enter a valid angle.

  Wiring:
  - Servo Red (VCC)  -> Arduino 5V
  - Servo Black (GND) -> Arduino GND
  - Servo Signal      -> Arduino D3

  Written for Arduino Nano.
*/

#include <Servo.h>

Servo myServo;  // Create servo object

void setup() {
  Serial.begin(9600);  // Start serial communication
  myServo.attach(3);   // Attach servo to pin D9
  Serial.println("Enter an angle (0 to 180) to move the servo:");
}

void loop() {
  if (Serial.available()) {  // Check if data is available
    int angle = Serial.parseInt();  // Read the integer from Serial Monitor

    if (angle >= 0 && angle <= 180) {  // Validate input range
      myServo.write(angle);  // Move servo to the desired angle
      Serial.print("Servo moved to: ");
      Serial.println(angle);
    } else {
      Serial.println("Invalid input! Please enter a value between 0 and 180.");
    }

    // Clear any extra input
    while (Serial.available()) {
      Serial.read();
    }
  }
}
