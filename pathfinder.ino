
int ultrasonicPin = 7; //ultrasonic sensor data read pin
int filterSize = 1;
int maxDistance = 250;
int minDistance = 20;

int middleBoundary = 180; //Distance Zones
int immediateBoundary = 140;

int rightMotorPin = 5; //right motor speed control pin
int leftMotorPin = 6; //left motor speed control pin

//output: returns distance in cm between maxDistance and minDistance
int getDistance(){
  int distance = 0;
  while(true)
  {
    distance = ( pulseIn(ultrasonicPin, HIGH) / 147 ) * 2.54;
    if((minDistance < distance) && (maxDistance > distance))
    {
      return distance;
    }
  }
}

//output: returns avarage of distance 
int averageDistance(){
  int distance = 0;
  for (int i = 0; i < filterSize; i++)
  {
    distance = distance + getDistance();
  }
  return distance / filterSize;  
}

//controls motors to signal obstacles in the path
void runPathfinder(){
  int distance = averageDistance();
  if(distance > middleBoundary){
    Serial.print("its far: ");
    Serial.print(distance);
    Serial.println();
    analogWrite(rightMotorPin, 100);
    analogWrite(leftMotorPin, 100);
    delay(200);
    
  }else if(distance > immediateBoundary){
    Serial.print("its middle: ");
    Serial.print(distance);
    Serial.println();
    analogWrite(rightMotorPin, 150);
    analogWrite(leftMotorPin, 150);
    delay(200);
  }else{
    Serial.print("its immediate: ");
    Serial.print(distance);
    Serial.println();
    analogWrite(rightMotorPin, 255);
    analogWrite(leftMotorPin, 255);
    delay(200);
  }
}


void setup() {
  pinMode(ultrasonicPin, INPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  runPathfinder(); 
}



