const int x_ekseni = A1;
const int y_ekseni = A0;
const int motor1 = 5;
const int motor2 = 6;
const int buton = 7;
const int buzzer = 3;
int x_durum;
int y_durum;
int b_durum;

void setup() {
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(buton,INPUT);
  Serial.begin(9600);
}

void loop() {
  x_durum = analogRead(x_ekseni);
  y_durum = analogRead(y_ekseni);
  b_durum = digitalRead(buton);
  Serial.print("Buton Durumu : ");
  Serial.println(b_durum);
  Serial.print("x : ");
  Serial.println(x_durum);
  Serial.print("y : ");
  Serial.println(y_durum);
  Serial.println("--------------------");

  analogWrite(motor1,(x_durum)/4.11);
  analogWrite(motor2,(y_durum)/4.11);
  if(x_durum>=460 && x_durum<=500 && y_durum>=460 && y_durum<=500){
    analogWrite(motor1,0);
    analogWrite(motor2,0);
    }

    if(!b_durum){
      analogWrite(buzzer,255);
    }
    else{
      analogWrite(buzzer,0);
    }
  }
