// Motor Driver Pins
#define ENA 18  // Enable pin for Motor A
#define IN1 19  // Input 1 for Motor A
#define IN2 22  // Input 2 for Motor A

// Motor B
#define ENB 5   // Enable pin for Motor B
#define IN3 16  // Input 3 for Motor B
#define IN4 23  // Input 4 for Motor B

// Motor Configuration Constants
const int offsetA = 1;
const int offsetB = 1;

// IR Sensor Pins
const int irSensorPins[] = {0, 34, 35, 32, 33, 25};
const int NUM_SENSORS = 5; // Number of IR sensors

// PID Controller Constants
float Kp = 0.06;
float Kd = 1.5;
float Ki = 0;

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfSpeed = 120;
int currentSpeed = 30;

int onLine = 1;
int minValues[NUM_SENSORS], maxValues[NUM_SENSORS], threshold[NUM_SENSORS], sensorValue[NUM_SENSORS], brakeFlag = 0; 
int lineThickness = 25;  // Line thickness in mm
bool isBlackLine = true; // Set to true for black line, false for white line

void setup() {
  Serial.begin(115200);

  // Motor Driver Setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // IR Sensor Setup
  for (int i = 1; i <= NUM_SENSORS; i++) {
    pinMode(irSensorPins[i], INPUT);
  }
}

void loop() {
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (1) {
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;
    if (onLine == 1) {  // PID LINE FOLLOW
      linefollow();
      digitalWrite(13, HIGH);
      brakeFlag = 0;
    } else {
      digitalWrite(13, LOW);
      if (error > 0) {
        if (brakeFlag == 0) {
          motorDrive(-100, 150);
          delay(30);
        }
        motorDrive(-100, 150);
        brakeFlag = 1;
      } else {
        if (brakeFlag == 0) {
          motorDrive(150, -100);
          delay(30);
        }
        motorDrive(150, -100);
        brakeFlag = 1;
      }
    }
  }
}

void linefollow() {
  error = (3 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 3 * sensorValue[5]);

  if (lineThickness > 22) {
    error = -1 * error;
  }
  if (isBlackLine) {
    error = -1 * error;
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  motorDrive(constrain(lsp, 0, 255), constrain(rsp, 0, 255));
}

void calibrate() {
  for (int i = 1; i <= NUM_SENSORS; i++) {
    minValues[i] = analogRead(irSensorPins[i]);
    maxValues[i] = analogRead(irSensorPins[i]);
  }

  for (int i = 0; i < 10000; i++) {
    motorDrive(50, -50);

    for (int i = 1; i <= NUM_SENSORS; i++) {
      int sensorReading = analogRead(irSensorPins[i]);
      if (sensorReading < minValues[i]) {
        minValues[i] = sensorReading;
      }
      if (sensorReading > maxValues[i]) {
        maxValues[i] = sensorReading;
      }
    }
  }

  for (int i = 1; i <= NUM_SENSORS; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();

  motorDrive(0, 0);
}

void readLine() {
  onLine = 0;
  for (int i = 1; i <= NUM_SENSORS; i++) {
    sensorValue[i] = map(analogRead(irSensorPins[i]), minValues[i], maxValues[i], 0, 1000);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    if ((isBlackLine && sensorValue[i] > 700) || (!isBlackLine && sensorValue[i] < 700)) {
      onLine = 1;
    }
  }
}

void motorDrive(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, abs(leftSpeed));
  digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH);

  analogWrite(ENB, abs(rightSpeed));
  digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH);
}
