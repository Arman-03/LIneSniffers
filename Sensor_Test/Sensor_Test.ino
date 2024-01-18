const int irSensorPin1 = 34;
const int irSensorPin2 = 35;
const int irSensorPin3 = 32;
const int irSensorPin4 = 33;
const int irSensorPin5 = 25;

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud

  pinMode(irSensorPin1, INPUT);
  pinMode(irSensorPin2, INPUT);
  pinMode(irSensorPin3, INPUT);
  pinMode(irSensorPin4, INPUT);
  pinMode(irSensorPin5, INPUT);
}

void loop() {
  int irSensorValue1 = analogRead(irSensorPin1);
  int irSensorValue2 = analogRead(irSensorPin2);
  int irSensorValue3 = analogRead(irSensorPin3);
  int irSensorValue4 = analogRead(irSensorPin4);
  int irSensorValue5 = analogRead(irSensorPin5);
  
  Serial.print(irSensorValue1);
  Serial.print("");
  Serial.print(irSensorValue2);
  Serial.print(",");
  Serial.print(irSensorValue3);
  Serial.print(",");
  Serial.print(irSensorValue4);
  Serial.print(","); 
  Serial.println(irSensorValue5);

//  delay(1000); // Delay for 1 second (adjust as needed)
}
