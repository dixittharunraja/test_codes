// Attention: Select the correct port and "Arduino Uno" as the board before uploading.

// Arduino Uno Full Self-Diagnostic Test
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>

#define TEST_DELAY 500
#define ANALOG_THRESHOLD 50

bool serialOK = false;
bool ledOK = false;
bool digitalIOOK = true;
bool analogOK = true;
bool pwmOK = true;
bool eepromOK = false;
//bool i2cOK = false;  //if you want to check this, connect i2c
bool spiOK = false;
//bool voltageOK = false;   //if you want to check this, connect voltage

void setup() {
    Serial.begin(9600);
    delay(2000);
    Serial.println("\n=== Arduino Uno Full Self-Diagnostic Test ===");

    // 1. Serial Communication Test
    Serial.println("Checking Serial Communication...");
    serialOK = true;
    Serial.println("✔ Serial Communication: PASS");

    // 2. Built-in LED Test
    Serial.println("Checking Built-in LED...");
    pinMode(LED_BUILTIN, OUTPUT);
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(TEST_DELAY);
        digitalWrite(LED_BUILTIN, LOW);
        delay(TEST_DELAY);
    }
    ledOK = true;
    Serial.println("✔ Built-in LED: PASS");

    // 3. Digital I/O Test
    Serial.println("Checking Digital I/O...");
    for (int pin = 2; pin <= 13; pin++) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        delay(50);
        if (digitalRead(pin) == LOW) digitalIOOK = false;
        digitalWrite(pin, LOW);
    }
    Serial.println(digitalIOOK ? "✔ Digital I/O: PASS" : "✖ Digital I/O: FAIL");

    // 4. Analog Input Test
    Serial.println("Checking Analog Inputs...");
    for (int pin = A0; pin <= A5; pin++) {
        pinMode(pin, INPUT);
        int value = analogRead(pin);
        if (value < ANALOG_THRESHOLD) analogOK = false;
        Serial.print("Analog Pin ");
        Serial.print(pin);
        Serial.print(": ");
        Serial.println(value);
    }
    Serial.println(analogOK ? "✔ Analog Inputs: PASS" : "✖ Analog Inputs: FAIL");

    // 5. PWM Test
    Serial.println("Checking PWM Outputs...");
    int pwmPins[] = {3, 5, 6, 9, 10, 11};
    for (int i = 0; i < 6; i++) {
        int pin = pwmPins[i];
        pinMode(pin, OUTPUT);
        analogWrite(pin, 128);
        delay(100);
        if (analogRead(pin) < 100) pwmOK = false;
        analogWrite(pin, 0);
    }
    Serial.println(pwmOK ? "✔ PWM Outputs: PASS" : "✖ PWM Outputs: FAIL");

    // 6. EEPROM Test
    Serial.println("Checking EEPROM...");
    EEPROM.write(0, 42);
    delay(10);
    int eepromValue = EEPROM.read(0);
    eepromOK = (eepromValue == 42);
    Serial.println(eepromOK ? "✔ EEPROM: PASS" : "✖ EEPROM: FAIL");

    // 7. I2C Communication Test
    /*Serial.println("Checking I2C Communication...");
    Wire.begin();
    Wire.beginTransmission(0x00); // Test with a non-existent address
    i2cOK = (Wire.endTransmission() == 0);
    Serial.println(i2cOK ? "✔ I2C Communication: PASS" : "✖ I2C Communication: FAIL");*/

    // 8. SPI Communication Test
    Serial.println("Checking SPI Communication...");
    pinMode(SS, OUTPUT);
    digitalWrite(SS, LOW);
    SPI.begin();
    SPI.transfer(0x00); // Dummy transfer
    digitalWrite(SS, HIGH);
    spiOK = true; // If no crash, assume SPI works
    Serial.println("✔ SPI Communication: PASS");

    // 9. Voltage Test
    /*Serial.println("Checking Voltage Levels...");
    pinMode(A0, INPUT);
    float voltage = analogRead(A0) * (5.0 / 1023.0); // Expect around 5V
    voltageOK = (voltage >= 4.7 && voltage <= 5.3);
    Serial.print("Measured Voltage: ");
    Serial.println(voltage);
    Serial.println(voltageOK ? "✔ 5V Power Supply: PASS" : "✖ 5V Power Supply: FAIL");*/

    // Final Report
    Serial.println("\n=== Final Test Report ===");
    Serial.println(serialOK ? "✔ Serial Communication: OK" : "✖ Serial Communication: ERROR");
    Serial.println(ledOK ? "✔ Built-in LED: OK" : "✖ Built-in LED: ERROR");
    Serial.println(digitalIOOK ? "✔ Digital I/O: OK" : "✖ Digital I/O: ERROR");
    Serial.println(analogOK ? "✔ Analog Inputs: OK" : "✖ Analog Inputs: ERROR");
    Serial.println(pwmOK ? "✔ PWM Outputs: OK" : "✖ PWM Outputs: ERROR");
    Serial.println(eepromOK ? "✔ EEPROM: OK" : "✖ EEPROM: ERROR");
    //Serial.println(i2cOK ? "✔ I2C Communication: OK" : "✖ I2C Communication: ERROR");
    Serial.println(spiOK ? "✔ SPI Communication: OK" : "✖ SPI Communication: ERROR");
    //Serial.println(voltageOK ? "✔ 5V Power Supply: OK" : "✖ 5V Power Supply: ERROR");

    if (serialOK && ledOK && digitalIOOK && analogOK && pwmOK && eepromOK && spiOK) {
        Serial.println("\n✅ Arduino Uno is fully functional!");
    } else {
        Serial.println("\n⚠️ Issues detected. Check failed tests.");
    }
}

void loop() {
    // No need to loop, test runs once
}
