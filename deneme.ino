bool IsPathfinder = false;

int upperBoundary = 150 ; // Ultrasonic Sensor data read upper boundary
int lowerBoundary = 50 ; // Ultrasonic Sensor data read lower boundary

const int motorPin = 3; // Right motor speed control pin

const int ultrasonicDataPin = 7; // Ultrasonic sensor data read pin
int motorDurumu=0;
int t2,eskiZaman2=0,yeniZaman2;
void setup() {
  Serial.begin(9600);
  pinMode(ultrasonicDataPin, INPUT);
  pinMode(motorPin, OUTPUT);
}

void loop() {
  yeniZaman2 = millis();
  t2=yeniZaman2-eskiZaman2;
  int distance = ( pulseIn(ultrasonicDataPin, HIGH) / 147 ) * 2.54;
  Serial.println(distance);
  if(upperBoundary > distance || distance >lowerBoundary){
    //analogWrite(motorPin,255 );
    //delay(200);
    //analogWrite(motorPin,0);

    if(t2 > 200){
     if(motorDurumu == 1){
      analogWrite(motorPin,0);
      motorDurumu = 0;
     }
     else{
      analogWrite(motorPin,200 );
      motorDurumu = 1;
     }
     eskiZaman2 = yeniZaman2;
  }
  }
   if(distance < lowerBoundary || distance > upperBoundary){
    analogWrite(motorPin,0);
  }
}
