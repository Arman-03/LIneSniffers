#include <Arduino.h>

// Motor pins
const int motor1A = 19;
const int motor1B = 22;
const int motor1Enable = 18; // Enable pin for motor 1

const int motor2A = 16;
const int motor2B = 23;
const int motor2Enable = 5; // Enable pin for motor 2

// IR sensor pins
const int irSensor1 = 34;
const int irSensor2 = 35;
const int irSensor3 = 32;
const int irSensor4 = 33;
const int irSensor5 = 25;

// Motor speed variables (0-255)
int motorSpeed = 90;

void setup() {
  // Motor pins setup
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor1Enable, OUTPUT);

  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(motor2Enable, OUTPUT);

  // IR sensor pins setup
  pinMode(irSensor1, INPUT);
  pinMode(irSensor2, INPUT);
  pinMode(irSensor3, INPUT);
  pinMode(irSensor4, INPUT);
  pinMode(irSensor5, INPUT);
}

void loop() {
  int sensor1 = digitalRead(irSensor1);
  int sensor2 = digitalRead(irSensor2);
  int sensor3 = digitalRead(irSensor3);
  int sensor4 = digitalRead(irSensor4);
  int sensor5 = digitalRead(irSensor5);

  // Adjust motor speeds based on sensor readings
  if (sensor3 == LOW) {
    // On track, move forward
    analogWrite(motor1Enable, motorSpeed);
    analogWrite(motor2Enable, motorSpeed);
    
    digitalWrite(motor1A, HIGH);
    digitalWrite(motor1B, LOW);
    
    digitalWrite(motor2A, HIGH);
    digitalWrite(motor2B, LOW);
  } else {
    // Off track, adjust motor speeds
    if (sensor2 == LOW || sensor4 == LOW) {
      // Slightly turn left
      analogWrite(motor1Enable, motorSpeed);
      analogWrite(motor2Enable, motorSpeed);
      
      digitalWrite(motor1A, HIGH);
      digitalWrite(motor1B, LOW);
      
      digitalWrite(motor2A, LOW);
      digitalWrite(motor2B, HIGH);
    } else if (sensor1 == LOW) {
      // Hard turn left
      analogWrite(motor1Enable, motorSpeed);
      analogWrite(motor2Enable, motorSpeed);
      
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, HIGH);
      
      digitalWrite(motor2A, HIGH);
      digitalWrite(motor2B, LOW);
    } else if (sensor5 == LOW) {
      // Slightly turn right
      analogWrite(motor1Enable, motorSpeed);
      analogWrite(motor2Enable, motorSpeed);
      
      digitalWrite(motor1A, LOW);
      digitalWrite(motor1B, HIGH);
      
      digitalWrite(motor2A, HIGH);
      digitalWrite(motor2B, LOW);
    } else if (sensor4 == LOW) {
      // Hard turn right
      analogWrite(motor1Enable, motorSpeed);
      analogWrite(motor2Enable, motorSpeed);
      
      digitalWrite(motor1A, HIGH);
      digitalWrite(motor1B, LOW);
      
      digitalWrite(motor2A, LOW);
      digitalWrite(motor2B, HIGH);
    } else {
      // Stop if no sensor detects the line
      analogWrite(motor1Enable, 0);
      analogWrite(motor2Enable, 0);
    }
  }
}
