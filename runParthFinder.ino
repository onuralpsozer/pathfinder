int ultrasonicPin = 7; //ultrasonic sensor data read pin
int filterSize = 1;
int maxDistance = 250;
int minDistance = 20;

int middleBoundary = 180; //Distance Zones
int immediateBoundary = 140;

int rightMotorPin = 5; //right motor speed control pin
int leftMotorPin = 6; //left motor speed control pin

const int x_ekseni = A1;
const int y_ekseni = A0;

const int buton = 8;
const int buzzer = 3;

float x_durum;
float y_durum;
float b_durum;


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

float xEkseni(){
  return analogRead(x_ekseni);
  }
float yEkseni(){
  return analogRead(y_ekseni);
  }
int but(){
  return digitalRead(buton);
  }


void joystickAndBuzzer() {
  x_durum = xEkseni();
  y_durum = yEkseni();
  b_durum = but();
  Serial.print("Buton Durumu : ");
  Serial.println(b_durum);
  Serial.print("x : ");
  Serial.println(x_durum);
  Serial.print("y : ");
  Serial.println(y_durum);
  Serial.println("--------------------");

  analogWrite(rightMotorPin,(x_durum)/4.11);
  analogWrite(leftMotorPin,(y_durum)/4.11);
  if(x_durum>=460 && x_durum<=500 && y_durum>=460 && y_durum<=500){
    analogWrite(rightMotorPin,0);
    analogWrite(leftMotorPin,0);
    }

    if(!b_durum){
      analogWrite(buzzer,255);
    }
    else{
      analogWrite(buzzer,0);
    }
  }


void setup() {
  pinMode(ultrasonicPin, INPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(buton,INPUT);
  Serial.begin(115200);
}

void loop() {
  runPathfinder();
  joystickAndBuzzer();
}

