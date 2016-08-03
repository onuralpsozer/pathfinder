int ultrasonicPin = 7; //ultrasonic sensor data read pin
int filterSize = 1;

int farBoundary=180;
int middleBoundary = 160; //Distance Zones
int immediateBoundary = 140;
int cutoffBoundary=50;

int rightMotorPin = 3; //right motor speed control pin
int leftMotorPin = 6; //left motor speed control pin





//output: returns distance in cm between maxDistance and minDistance
int getDistance(){
  int distance = 0;
  while(true)
  {
    distance = ( pulseIn(ultrasonicPin, HIGH) / 147 ) * 2.54;
      Serial.println(distance);
      return distance;
    
  }
}

//controls motors to signal obstacles in the path
void runPathfinder(){
  int distance = getDistance();
  if(distance < cutoffBoundary){
    analogWrite(rightMotorPin,0);
    analogWrite(leftMotorPin, 0);
    Serial.print(distance);
    Serial.println();
  }else if(immediateBoundary > distance){
    Serial.print("its immediate: ");
    Serial.print(distance);
    Serial.println();
    analogWrite(rightMotorPin,255 );
    analogWrite(leftMotorPin, 255);
    delay(200);
  }else if(middleBoundary > distance){
    Serial.print("its middle: ");
    Serial.print(distance);
    Serial.println();
    analogWrite(rightMotorPin, 180);
    analogWrite(leftMotorPin, 180);
    delay(200);
  }else if(farBoundary > distance){
    Serial.print("its far: ");
    Serial.print(distance);
    Serial.println();
    analogWrite(rightMotorPin, 100);
    analogWrite(leftMotorPin, 100);
    delay(200);
    
  }
  
  analogWrite(rightMotorPin,0);
  analogWrite(leftMotorPin, 0);
}







void setup() {
  pinMode(ultrasonicPin, INPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  //pinMode(buzzer,OUTPUT);
  //pinMode(buton,INPUT);
  //pinMode(x_ekseni,INPUT);
  //pinMode(y_ekseni,INPUT);
  //pinMode(guc,OUTPUT);
  Serial.begin(9600);
}

void loop() {
  //digitalWrite(guc,HIGH);
  
  runPathfinder();
  
}

