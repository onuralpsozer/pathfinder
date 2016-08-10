int ultrasonicDataPin = 7; //ultrasonic sensor data read pin

 //Distance Zones
int upperBoundary = 150 ;
int lowerBoundary = 50 ;

int rightMotorPin = 5; //right motor speed control pin
int leftMotorPin = 6; //left motor speed control pin

bool durum0 = false;

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
    analogWrite(rightMotorPin, 200);
    analogWrite(leftMotorPin, 200);
    
    //Serial.print(distance);
   // Serial.println();
    
  }
 }
 void pathfinderInitialization(){
  pinMode(ultrasonicDataPin, INPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
 }
 ISR(TIMER1_COMPA_vect){   //timer1 interrupt ı 1Hz de tetikleniyor.
  if (durum0){
    analogWrite(rightMotorPin,0);
    analogWrite(leftMotorPin, 0);
    durum0 = 0;
  }
  else{
   
    durum0 = 1;
  }
 }
 void interraptSetup(){
  cli(); //interreptler durduruluyor.
  //timer1 1Hz' e ayarlanıyor
    //registerler sıfırlanır
    TCCR1A = 0;// TCCR1A register 0'lanıyor
    TCCR1B = 0;// TCCR1B register 0'lanıyor
    TCNT1  = 0;//sayac değeri  0'la
 // OCRxA karşılaştırma registeri 1Hz değer için ayarlanıyor
  //16 MHz osilatör,1Hz timer1 ın çalışma frekansı,1024 prescalar
    OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (değer 65536 dan küçük)
//   CTC mod açılıyor.
    TCCR1B |= (1 << WGM12);
  //   CS10 ve CS12 bitleri 1024 prescaler için ayarlanıyor
    TCCR1B |= (1 << CS12) | (1 << CS10);  
  // timer karşılaştırma interruptı aktifleştiriliyor
    TIMSK1 |= (1 << OCIE1A);
    //interreuptlar aktif
    sei();
 }
 

void setup() {
  interraptSetup();
  pathfinderInitialization();
  Serial.begin(9600);
}

void loop() {
  runPathfinder();
}
