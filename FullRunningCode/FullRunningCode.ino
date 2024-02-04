#include <QTRSensors.h>
#include <L298N.h>
#include "BluetoothSerial.h"

const int ledPin = 2;

//Assigning the motor pins (Connecting to the Motor Driver and the Motor)
const int rightMotor1 = 19;
const int rightMotor2 = 18;
const int leftMotor1 = 17;
const int leftMotor2 = 16;

//Enable pins of Motor driver to control the motor speed
const int rightPWM = 13;
const int leftPWM = 14;
// const int buttonPin = ;

BluetoothSerial SerialBT;

//Assinging pinNumbers of Motor driver using library
L298N motor1(rightPWM, rightMotor1, rightMotor2);
L298N motor2(leftPWM, leftMotor1, leftMotor2);

const int numSensors = 8;
const int sensorPins[numSensors] = {36, 39, 34, 35, 32, 33, 25, 26};

//SetPoint from the center of the line
double setPoint = 4.5;
int speedOfBot = 120;
double input, output, prevInput = 0, prevErr = 0, iErr, integralConstant = 0;

//Declare and assigning the PID values to zero initially
double kp = 124.0;
double kd = 43.0;
double ki = 4.5;

//pId values setting helper variables
double multiP = 1, multiI = 1, multiD = 1;
double Pvalue, Ivalue, Dvalue;

bool onoff = false;

//Bluetooth connectivity values variables
int val, cnt = 0, v[3];

void setup() {
  Serial.begin(112500);
  SerialBT.begin("MINE <3");
  //Assigning sensor pinNumbers to the sensor
  for(int i=0; i<numSensors; i++) {
    pinMode(sensorPins[i] , INPUT);
  }

  //setting the pinModes of Enables of the motor(incase it is not)
  pinMode(rightPWM, OUTPUT);
  pinMode(leftPWM, OUTPUT);
}

void loop() {
  //Bluetooth Functionality
  if(SerialBT.available()) {
    while(SerialBT.available() == 0) {
      valuesRead();
      processing();
    }
  }

  if(onoff == true) {
    int sensorValues[numSensors];
    //Reading the sensorValues as digital
    for(int i=0; i<numSensors; i++) {
      sensorValues[i] = digitalRead(sensorPins[i]);
      //For Debugging: reading sensor values
      Serial.print(sensorValues[i]);
      Serial.print(" ");
    }


    //Calculating the weightedSum
    double weightedSum = 0, sum = 0;
    for(int i=0; i<numSensors; i++) {
      weightedSum += (i+1)*sensorValues[i];
      sum += sensorValues[i];
    }
    //For debugging: 

    if(sum) {
      input = weightedSum / sum;
      prevInput = input;
    }
    else {
      //To avoid division by zero
      input = prevInput;
    }

    //Declaring and assigning the error
    double err = input - setPoint;
    //For Debugging: Finding the errorValue
    // Serial.print("Error: ");
    // Serial.println(err);
    // Limiting the variable to a specific range
    // iErr = constrain(iErr, -1*speedOfBot, speedOfBot);

    double P = err;
    double I = I + err;
    double D = err - prevErr;

    Pvalue = (kp / pow(10, multiP))*P;
    Dvalue = (kd / pow(10, multiD))*D;
    Ivalue = (ki / pow(10, multiI))*I;

    double pId = Pvalue + Dvalue + Ivalue;

    // pId = kp*err + kd*(err-prevErr) + ki*iErr;
    prevErr = err;
    // iErr += err;

    //Setting the speedOfMotor according to the PID
    int leftMotorSpeed = speedOfBot - pId;
    int rightMotorSpeed = speedOfBot + pId;

    //For Debugging: 
    Serial.print("leftMotorSpeed: ");
    Serial.print(leftMotorSpeed);
    Serial.print(", rightMotorSpeed: ");
    Serial.println(rightMotorSpeed);
    // Serial.print("PID: ");
    // Serial.println(pId);

    //Applying the motor speeds
    if(leftMotorSpeed>0) {
      digitalWrite(leftMotor1, 1);
      digitalWrite(leftMotor2 ,0);
    }
    else{
      digitalWrite(leftMotor1, 0);
      digitalWrite(leftMotor2 ,1);
    }

    if(rightMotorSpeed>0) {
      digitalWrite(rightMotor1, 1);
      digitalWrite(rightMotor2 ,0);
    }
    else{
      digitalWrite(rightMotor1, 0);
      digitalWrite(rightMotor2 ,1);
    }

    //Adjusting motor speeds based on pId output
    leftMotorSpeed = constrain(abs(leftMotorSpeed), 0 , speedOfBot);
    rightMotorSpeed = constrain(abs(rightMotorSpeed), 0 , speedOfBot);
  } 
  else if(onoff == false) {
    motor1.stop();
    motor2.stop();
  }
}

void valuesRead() {
  val = SerialBT.read();
  cnt++;
  v[cnt] = val;
  if(cnt == 2) {
    cnt = 0;
  }
}

void processing() {
  int a = v[1];
  if(a==1) {
    kp = v[2];
  }
  if(a==2) {
    multiP = v[2];
  }
  if(a==3) {
    ki = v[2];
  }
  if(a==4) {
    multiI = v[2];
  }
  if(a==5) {
    kd = v[2];
  }
  if(a==6) {
    multiD = v[2];
  }
  if(a==7) {
    onoff = v[2];
  }
}
