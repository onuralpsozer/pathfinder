int batteryLevelPin = A1;

void setup() {
  Serial.begin(9600);
}
  void loop(){
  int batteryLevelValue = 0;
  float batteryLevelVoltage = 0;
  batteryLevelValue = analogRead(batteryLevelPin);
  batteryLevelVoltage = batteryLevelValue * (5.0 / 1023.0);
  Serial.print(batteryLevelVoltage);
  Serial.println();
  if(batteryLevelVoltage < 3){
    Serial.print("% 0");
    Serial.println();
  }
  else {
  float BatteryLevelPercent = (batteryLevelVoltage - 3.0)*100.0/1.2;
  Serial.print("%");
  Serial.print(BatteryLevelPercent);
  Serial.println();
  }
  delay (1000);
}
