#include <L298N.h>

#include <QTRSensors.h>

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define LED_BUILTIN 2

#define AIN1 19  //Assign the motor pins
#define BIN1 16
#define AIN2 22
#define BIN2 23
#define PWMA 18
#define PWMB 5
#define STBY 19


const int offsetA = 1;
const int offsetB = 1;

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

QTRSensors qtr;
BluetoothSerial SerialBT;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float Kp = 0;
float Ki = 0;
float Kd = 0;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

boolean onoff = false;
int Speed = 125; 

int val, cnt = 0, v[3];

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = Speed;

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){34, 35, 32, 33, 25}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  
  Serial.begin(115200);
  SerialBT.begin();
  Serial.println("Bluetooth Started! Ready to pair...");
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();

  delay(1000);
}

void loop()
{
  if (SerialBT.available()){
    while(SerialBT.available() == 0);
    valuesread();
    processing();
  }
  if (onoff == true){
    robot_control();
  }
  else if(onoff == false){
    motor1.stop();
    motor2.stop();
  }
}
void robot_control(){
  position = qtr.readLineBlack(sensorValues);
  error = 2000 - position;
  while(sensorValues[0]>=1000 && sensorValues[1]>=1000 && sensorValues[2]>=1000 && sensorValues[3]>=1000 && sensorValues[4]>=1000){ // A case when the line follower leaves the line
    if(previousError>0){       //Turn left if the line was to the left before
      motor_drive(-Speed,Speed);
    }
    else{
      motor_drive(Speed,-Speed); // Else turn right
    }
    position = qtr.readLineBlack(sensorValues);
  }
  
  PID_Linefollow(error);
}
void PID_Linefollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    if (lsp > Speed) {
      lsp = Speed;
    }
    if (lsp < -1*Speed) {
      lsp = -1*Speed;
    }
    if (rsp > Speed) {
      rsp = Speed;
    }
    if (rsp < -1*Speed) {
      rsp = -1*Speed;
    }
    motor_drive(lsp,rsp);
}

void valuesread()  {
  val = SerialBT.read();
  cnt++;
  v[cnt] = val;
  if (cnt == 2)
    cnt = 0;
}

//In this void the the 2 read values are assigned.
void  processing() {
  int a = v[1];
  if (a == 1) {
    Kp = v[2];
  }
  if (a == 2) {
    multiP = v[2];
  }
  if (a == 3) {
    Ki = v[2];
  }
  if (a == 4) {
    multiI = v[2];
  }
  if (a == 5) {
    Kd  = v[2];
  }
  if (a == 6) {
    multiD = v[2];
  }
  if (a == 7)  {
    onoff = v[2];
  }
} 
void motor_drive(int left, int right){
  
  if(right>0)
  {
    motor2.setSpeed(right);
    motor2.forward();
  }
  else 
  {
    motor2.setSpeed(right);
    motor2.backward();
  }
  
 
  if(left>0)
  {
    motor1.setSpeed(left);
    motor1.forward();
  }
  else 
  {
    motor1.setSpeed(left);
    motor1.backward();
  }
}
