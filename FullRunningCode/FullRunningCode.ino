#include <L298N.h>

#include <QTRSensors.h>

#define LED_BUILTIN 2

#define AIN1 19 
#define BIN1 16
#define AIN2 22
#define BIN2 23
#define PWMA 18
#define PWMB 5
#define STBY 19
const int SwitchPin = 13;

const int offsetA = 1;
const int offsetB = 1;

L298N motor1(PWMA, AIN1, AIN2);
L298N motor2(PWMB, BIN1, BIN2);

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

float Kp = 9;
float Ki = 0;
float Kd = 10;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

boolean onoff = true;
int Speed = 125; 


uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = Speed;

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){34, 35, 32, 33, 25}, SensorCount);
  pinMode(SwitchPin, INPUT);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 
  
  Serial.begin(115200);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); 

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
  Serial.print(position);
  Serial.print(",");
  Serial.print(error);
  Serial.println();

    while (sensorValues[0] >= 1000 && sensorValues[1] >= 1000 && sensorValues[2] >= 1000 && sensorValues[3] >= 1000 && sensorValues[4] >= 1000)
    {
      if (previousError > 0)
      {
        motor_drive(-1*Speed, Speed);
      }
      else
      {
        motor_drive(Speed, -1*Speed);
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
