#include <stdlib.h>

// Motor Driver Configuration
int motor1PWM = 10;  // PWM pin for Motor 1
int motor1Dir = 11;  // Direction pin 1 for Motor 1
int motor1Slp = 12;  // Direction pin 2 for Motor 1

int motor2PWM = 6;   // PWM pin for Motor 2
int motor2Dir = 5;   // Direction pin 1 for Motor 2
int motor2Slp = 4;   // Direction pin 2 for Motor 2

int speed = 70, rotSpeed = 50, val = 0;

void setup() {
  Serial.begin(9600);

  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor1Slp, OUTPUT);

  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motor2Slp, OUTPUT);

  pinMode(13, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim(); // Remove leading and trailing whitespaces, including the newline character
    val = atoi(receivedData.c_str());
    // val = Serial.parseInt();
    // delay(20);
    Serial.println(val);
    if (val == 1) {
      digitalWrite(13, HIGH);
      // delay(50);
    } else {
      digitalWrite(13, LOW);
      // delay(50);
    }

    controlMotors(val);
  }
}

// // This is the code for arduino on the right (motor 1 & 2)
// void controlMotors(int val) {
//   // Motor control based on val
//   if (val == 1) {
//     // Forward
//     digitalWrite(motor1Dir, HIGH);
//     digitalWrite(motor1Slp, LOW);
//     analogWrite(motor1PWM, speed);

//     digitalWrite(motor2Dir, LOW);
//     digitalWrite(motor2Slp, LOW);
//     analogWrite(motor2PWM, speed);
//   } else if (val == 2) {
//     // Backward
//     digitalWrite(motor1Dir, LOW);
//     digitalWrite(motor1Slp, LOW);
//     analogWrite(motor1PWM, speed);

//     digitalWrite(motor2Dir, HIGH);
//     digitalWrite(motor2Slp, LOW);
//     analogWrite(motor2PWM, speed);
//   } else if (val == 3) {
//     // Left
//     digitalWrite(motor1Dir, HIGH);
//     digitalWrite(motor1Slp, LOW);
//     analogWrite(motor1PWM, speed);

//     digitalWrite(motor2Dir, HIGH);
//     digitalWrite(motor2Slp, LOW);
//     analogWrite(motor2PWM, speed);
//   } else if (val == 4) {
//     // Right
//     digitalWrite(motor1Dir, LOW);
//     digitalWrite(motor1Slp, LOW);
//     analogWrite(motor1PWM, speed);

//     digitalWrite(motor2Dir, LOW);
//     digitalWrite(motor2Slp, LOW);
//     analogWrite(motor2PWM, speed);
//   } else if (val == 5) {
//     // Counter-Clockwise (CCW)
//     digitalWrite(motor1Dir, LOW);
//     digitalWrite(motor1Slp, LOW);
//     analogWrite(motor1PWM, rotSpeed);

//     digitalWrite(motor2Dir, HIGH);
//     digitalWrite(motor2Slp, LOW);
//     analogWrite(motor2PWM, rotSpeed);
//   } else if (val == 6) {
//     // Clockwise (CW)
//     digitalWrite(motor1Dir, HIGH);
//     digitalWrite(motor1Slp, LOW);
//     analogWrite(motor1PWM, rotSpeed);

//     digitalWrite(motor2Dir, LOW);
//     digitalWrite(motor2Slp, LOW);
//     analogWrite(motor2PWM, rotSpeed);
//   } else if (val == 7) {
//     // Forward
//     digitalWrite(motor1Dir, HIGH);
//     digitalWrite(motor1Slp, LOW);
//     analogWrite(motor1PWM, speed);

//     digitalWrite(motor2Dir, LOW);
//     digitalWrite(motor2Slp, LOW);
//     analogWrite(motor2PWM, speed);
//   } else {
//     // Stop
//     digitalWrite(motor1Dir, LOW);
//     digitalWrite(motor1Slp, HIGH);
//     analogWrite(motor1PWM, 0);

//     digitalWrite(motor2Dir, LOW);
//     digitalWrite(motor2Slp, HIGH);
//     analogWrite(motor2PWM, 0);
//   }
// }

// This is the code for arduino on the left (motor 3 & 4)
void controlMotors(int val) {
  // Motor control based on val
  if (val == 1) {
    // Forward
    digitalWrite(motor1Dir, HIGH);
    digitalWrite(motor1Slp, LOW);
    analogWrite(motor1PWM, speed);

    digitalWrite(motor2Dir, LOW);
    digitalWrite(motor2Slp, LOW);
    analogWrite(motor2PWM, speed);
  } else if (val == 2) {
    // Backward
    digitalWrite(motor1Dir, LOW);
    digitalWrite(motor1Slp, LOW);
    analogWrite(motor1PWM, speed);

    digitalWrite(motor2Dir, HIGH);
    digitalWrite(motor2Slp, LOW);
    analogWrite(motor2PWM, speed);
  } else if (val == 3) {
    // Left
    digitalWrite(motor1Dir, LOW);
    digitalWrite(motor1Slp, LOW);
    analogWrite(motor1PWM, speed);

    digitalWrite(motor2Dir, LOW);
    digitalWrite(motor2Slp, LOW);
    analogWrite(motor2PWM, speed);
  } else if (val == 4) {
    // Right
    digitalWrite(motor1Dir, HIGH);
    digitalWrite(motor1Slp, LOW);
    analogWrite(motor1PWM, speed);

    digitalWrite(motor2Dir, HIGH);
    digitalWrite(motor2Slp, LOW);
    analogWrite(motor2PWM, speed);
  } else if (val == 5) {
    // Counter-Clockwise (CCW)
    digitalWrite(motor1Dir, HIGH);
    digitalWrite(motor1Slp, LOW);
    analogWrite(motor1PWM, rotSpeed);

    digitalWrite(motor2Dir, LOW);
    digitalWrite(motor2Slp, LOW);
    analogWrite(motor2PWM, rotSpeed);
  } else if (val == 6) {
    // Clockwise (CW)
    digitalWrite(motor1Dir, LOW);
    digitalWrite(motor1Slp, LOW);
    analogWrite(motor1PWM, rotSpeed);

    digitalWrite(motor2Dir, HIGH);
    digitalWrite(motor2Slp, LOW);
    analogWrite(motor2PWM, rotSpeed);
  } else if (val == 7) {
    // Forward
    digitalWrite(motor1Dir, HIGH);
    digitalWrite(motor1Slp, LOW);
    analogWrite(motor1PWM, speed);

    digitalWrite(motor2Dir, LOW);
    digitalWrite(motor2Slp, LOW);
    analogWrite(motor2PWM, speed);
  } else {
    // Stop
    digitalWrite(motor1Dir, LOW);
    digitalWrite(motor1Slp, HIGH);
    analogWrite(motor1PWM, 0);

    digitalWrite(motor2Dir, LOW);
    digitalWrite(motor2Slp, HIGH);
    analogWrite(motor2PWM, 0);
  }
}