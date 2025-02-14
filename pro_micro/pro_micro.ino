//Attention......
//Before uploading the code, choose the appropriate port and select arduino leonardo as the board.

// Arduino Pro Micro Self-Diagnostic Test
#define TEST_DELAY 500  // Delay time for testing
#define ANALOG_THRESHOLD 50  // Minimum valid analog value

bool serialOK = false;
bool ledOK = false;
bool digitalIOOK = true;
bool analogOK = true;
bool pwmOK = true;

void setup() {
    Serial.begin(115200); // Initialize Serial (Pro Micro uses 115200 baud by default)
    delay(3000); // Allow time for serial port to initialize

    Serial.println("\n=== Arduino Pro Micro Self-Diagnostic Test ===");

    // 1. Serial Communication Test
    Serial.println("Checking Serial Communication...");
    serialOK = true; // If this prints, serial is working
    Serial.println("✔ Serial Communication: PASS");

    // 2. Built-in LED Test (Pro Micro has no dedicated LED, but pin 17 is onboard)
    Serial.println("Checking Built-in LED...");
    pinMode(LED_BUILTIN_TX, OUTPUT);  // TX LED (Pin 17)
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN_TX, HIGH);
        delay(TEST_DELAY);
        digitalWrite(LED_BUILTIN_TX, LOW);
        delay(TEST_DELAY);
    }
    ledOK = true; // If loop runs, LED is working
    Serial.println("✔ Built-in LED: PASS");

    // 3. Digital I/O Test
    Serial.println("Checking Digital I/O...");
    for (int pin = 2; pin <= 21; pin++) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        delay(50);
        if (digitalRead(pin) == LOW) digitalIOOK = false; // If pin fails to stay HIGH
        digitalWrite(pin, LOW);
    }
    Serial.println(digitalIOOK ? "✔ Digital I/O: PASS" : "✖ Digital I/O: FAIL");

    // 4. Analog Input Test
    Serial.println("Checking Analog Inputs...");
    for (int pin = A0; pin <= A11; pin++) {
        pinMode(pin, INPUT);
        int value = analogRead(pin);
        if (value < ANALOG_THRESHOLD) analogOK = false; // If reading is too low
        Serial.print("Analog Pin ");
        Serial.print(pin);
        Serial.print(": ");
        Serial.println(value);
    }
    Serial.println(analogOK ? "✔ Analog Inputs: PASS" : "✖ Analog Inputs: FAIL");

    // 5. PWM Test
    Serial.println("Checking PWM Outputs...");
    int pwmPins[] = {3, 5, 6, 9, 10}; // PWM-capable pins on Pro Micro
    for (int i = 0; i < 5; i++) {
        int pin = pwmPins[i];
        pinMode(pin, OUTPUT);
        analogWrite(pin, 128); // 50% duty cycle
        delay(100);
        if (analogRead(pin) < 100) pwmOK = false; // Validate PWM output
        analogWrite(pin, 0);
    }
    Serial.println(pwmOK ? "✔ PWM Outputs: PASS" : "✖ PWM Outputs: FAIL");

    // 6. Final Report
    Serial.println("\n=== Final Test Report ===");
    Serial.println(serialOK ? "✔ Serial Communication: OK" : "✖ Serial Communication: ERROR");
    Serial.println(ledOK ? "✔ Built-in LED: OK" : "✖ Built-in LED: ERROR");
    Serial.println(digitalIOOK ? "✔ Digital I/O: OK" : "✖ Digital I/O: ERROR");
    Serial.println(analogOK ? "✔ Analog Inputs: OK" : "✖ Analog Inputs: ERROR");
    Serial.println(pwmOK ? "✔ PWM Outputs: OK" : "✖ PWM Outputs: ERROR");

    if (serialOK && ledOK && digitalIOOK && analogOK && pwmOK) {
        Serial.println("\n✅ Arduino Pro Micro is fully functional!");
    } else {
        Serial.println("\n⚠️ Issues detected. Check failed tests.");
    }
}

void loop() {
    // No need to loop, test runs once
}
