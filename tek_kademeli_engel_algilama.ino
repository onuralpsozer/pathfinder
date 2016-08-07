int ultrasonicDataPin = 7; //ultrasonic sensor data read pin

 //Distance Zones
int upperBoundary = 150 ;
int lowerBoundary = 50 ;

int rightMotorPin = 5; //right motor speed control pin
int leftMotorPin = 6; //left motor speed control pin

//output: returns distance in cm between upperBoundary and lowerBoundary
int getUltrasonicData(){
  return ( pulseIn(ultrasonicDataPin, HIGH) / 147 ) * 2.54;
}

//controls motors to signal obstacles in the path
void runPathfinder(){
  int distance = getUltrasonicData();
  Serial.print(distance);
  Serial.println();
  if(distance < lowerBoundary){
    analogWrite(rightMotorPin,0);
    analogWrite(leftMotorPin, 0);
   // Serial.print(distance);
   // Serial.println();
  }else if(upperBoundary > distance){
    analogWrite(rightMotorPin,255 );
    analogWrite(leftMotorPin, 255);
   // Serial.print(distance);
   // Serial.println();
    delay(200);
    analogWrite(rightMotorPin,0);
    analogWrite(leftMotorPin, 0);
  }
 }
 void pathfinderInitialization(){
  pinMode(ultrasonicDataPin, INPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
 }
void setup() {
  pathfinderInitialization();
  Serial.begin(9600);
}

void loop() {
  runPathfinder();
}
