#include <SPI.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include <services.h> 

int i=0,k=50;
unsigned char len = 0;
char str[16];
String string1,string2,string3,string4,string5,string6;

int ultrasonicDataPin = 7; //ultrasonic sensor data read pin

 //Distance Zones
int upperBoundary = 150 ;
int lowerBoundary = 50 ;

int rightMotorPin = 5; //right motor speed control pin
int leftMotorPin = 6; //left motor speed control pin

int button1Pin = A2;
int button2Pin = A0;
int button3Pin = A5;

int buzzerPin= 3;


//output: returns distance in cm between upperBoundary and lowerBoundary
int getUltrasonicData(){
  return ( pulseIn(ultrasonicDataPin, HIGH) / 147 ) * 2.54;
}

void simpleChat()
{
  if ( ble_available() )
  {
    string1=String("");
    while ( ble_available() ){
    for(i=1 ; i<sizeof(ble_read()) ;i++){
    str[i]=ble_read();
     Serial.print(str[i]);
     string1=string1+String(str[i]);
   }
    }
   Serial.println();
   Serial.println(string1);
   
   Serial.println();
   //string1=String("");
  }
  
  if ( Serial.available() )
  {
    delay(5);
    
    while ( Serial.available() )
        ble_write( Serial.read() );
  }
  
  ble_do_events();
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
 void buttonBuzzer(){
  if(analogRead(button1Pin) > 10){
    Serial.print("artarak çalışıyor");
    Serial.println();
    k=k+40;
    analogWrite(buzzerPin , k);
    delay(50);
    analogWrite(buzzerPin , 0);
  }
  if(analogRead(button2Pin) > 10){
    Serial.print("artarak çalışıyor");
    Serial.println();
    k=k-40;
    analogWrite(buzzerPin , k);
    delay(50);
    analogWrite(buzzerPin , 0);
  }
 if(analogRead(button3Pin) > 10){
    Serial.print("artarak çalışıyor");
    Serial.println();
   
    analogWrite(buzzerPin , 180);
    delay(50);
    analogWrite(buzzerPin , 0);
  }
 }
 void buttonBuzzerInitialization(){
  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(button3Pin, INPUT);
  pinMode(buzzerPin, OUTPUT);
 }
 void pathfinderInitialization(){
  pinMode(ultrasonicDataPin, INPUT);
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
 }
void setup() {
  ble_set_name("PATHFINDER");
  ble_begin();
  buttonBuzzerInitialization();
  pathfinderInitialization();
  Serial.begin(57600);
}

void loop() {
  string2 = String("kapat");
  string3 = String("acik");
  string4 = String("sagmotor");
  string5 = String("solmotor");
  string6 = String("buzzer");
 
  simpleChat();
  if(string1==string2){
    while(string1==string2){
   analogWrite(rightMotorPin,0);
   analogWrite(leftMotorPin,0);
   analogWrite(buzzerPin , 0);
   simpleChat();
    }
    simpleChat();
  }
   buttonBuzzer();
   runPathfinder();
   if(string1==string3){
   while(string1==string3){
   runPathfinder();
   buttonBuzzer();
   simpleChat();
   }
   simpleChat();
   }
   if(string1==string4){
   while(string1==string4){
   analogWrite(rightMotorPin,255);
   simpleChat();
   }
   simpleChat();
   }
   if(string1==string5){
   while(string1==string5){
   analogWrite(leftMotorPin,255);
   simpleChat();
   }
   simpleChat();
   }
   if(string1==string6){
   while(string1==string6){
   analogWrite(buzzerPin , 180);
   simpleChat();
   }
   simpleChat();
   }
}
