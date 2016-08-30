bool IsPathfinder = false;

volatile uint8_t UltrasonicSensorData = 0; // First state initializations
volatile uint8_t batteryLevelPercent = 0;

int t2;
int motorDurumu=0;
unsigned long eskiZaman2=0;
unsigned long yeniZaman2;

int upperBoundary = 150 ; // Ultrasonic Sensor data read upper boundary
int lowerBoundary = 50 ; // Ultrasonic Sensor data read lower boundary

volatile int rightMotorPin = 5; // Right motor speed control pin
volatile int leftMotorPin = 6; // Left motor speed control pin

volatile int buzzerPin= 4;
volatile int batteryLevelPin = A1; 

volatile int buttonPin1 = 2;
volatile int buttonPin2 = 3;

volatile uint8_t Button1State = 0x00;
volatile uint8_t Button2State = 0x00;

int ultrasonicDataPin = 7; // Ultrasonic sensor data read pin

void startUpBuzzer(){
  digitalWrite(buzzerPin, HIGH);
  delay(250);
  digitalWrite(buzzerPin, LOW); 
}

void pathfinderInitialization( void ){ // Input or output initialization
  pinMode(ultrasonicDataPin, INPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
}

void buttonBuzzerInitialization(){
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buzzerPin, OUTPUT);
 }

void buttonInterrupt1(){
  Button1State = digitalRead(buttonPin1);
  Serial.println(Button1State);
}
void buttonInterrupt2(){
  Button2State = digitalRead(buttonPin2);
  Serial.println(Button2State);
}

uint8_t getUltrasonicData(){
  return ( pulseIn(ultrasonicDataPin, HIGH) / 147 ) * 2.54; // Ultrasonic data to cm 
}

uint8_t batteryLevel(){
  int batteryLevelValue = 0;
  batteryLevelValue = analogRead(batteryLevelPin);
  batteryLevelPercent = ((batteryLevelValue * (5.0 / 1023.0))-3)*100.0/1.2 ;
  
  if ( batteryLevelPercent > 100 ) {
    batteryLevelPercent = 100 ;
  }
  else if ( batteryLevelPercent < 0){
    batteryLevelPercent = 0 ;
  }
  
  return (batteryLevelPercent);
}

void runPathfinder( void ){
  yeniZaman2 = millis();
  t2=yeniZaman2-eskiZaman2;
  int distance = getUltrasonicData();  // Pathfinder algorithm 50 to 150 cm working area and giving motors power
  //Serial.print(distance);
  //Serial.println();
  if(upperBoundary > distance || distance >lowerBoundary){
    //analogWrite(motorPin,255 );
    //delay(200);
    //analogWrite(motorPin,0);

    if(t2 > 200){
     if(motorDurumu == 1){
      analogWrite(rightMotorPin,0);
      analogWrite(leftMotorPin,0);
      motorDurumu = 0;
     }
     else{
      analogWrite(rightMotorPin,255);
      analogWrite(leftMotorPin,255);
      motorDurumu = 1;
     }
     eskiZaman2 = yeniZaman2;
  }
  }
   if(distance < lowerBoundary || distance > upperBoundary){
    analogWrite(rightMotorPin,0);
    analogWrite(leftMotorPin,0);
  }
}

void Drive(int Type, int PWM){  // Motor or buzzer drive function
  analogWrite(Type, PWM);
}

void StopDriving(int Type ){
  analogWrite(Type, 0);
}

