/*
   EMG Read + Motor Actuation
   Pin 9 is used for the motor/LED.
*/
#include <Servo.h>

Servo middle;

const int emgPin = A0;
const int middlepin = 9; // Connect your motor driver or LED here

void setup() {
  Serial.begin(115200);
  pinMode(emgPin, INPUT);
  middle.attach(middlepin);
   // Set motor pin as output

  middle.write(90);
}

void loop() {
  // --- 1. SENSING (Send to MATLAB) ---
  int rawValue = analogRead(emgPin);
  Serial.println(rawValue);
  // Serial.print(",");
  // Serial.print(0);
  // Serial.print(",");
  // Serial.println(1300);

  // --- 2. ACTUATION (Receive from MATLAB) ---
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1') {
      middle.write(0);
    } else if (command == '0') {
      middle.write(90);  // Stop
    }
  }

  // 1ms delay for ~1000Hz sampling
  delay(1);
}