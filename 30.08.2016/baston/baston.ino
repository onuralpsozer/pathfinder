#include <avr/io.h> 
#include <avr/wdt.h>

#include "bluetooth.h"

void setup(void)
{
  wdt_disable();  // disable watch dog timer because we dont need it in setup process
  pathfinderInitialization(); 
  buttonBuzzerInitialization();
  InitializeBluetooth(); // setup functions in pathfinder.h and bluetooth.h are called 
  attachInterrupt(digitalPinToInterrupt(buttonPin1),buttonInterrupt1,CHANGE); // 
  attachInterrupt(digitalPinToInterrupt(buttonPin2),buttonInterrupt2,CHANGE);
  Serial.begin(115200); // Arduino's data read speed ( baud ) 115200
  startUpBuzzer(); //Works once for every start
  wdt_enable(WDTO_4S); // Enable watch dog timer 
}

void loop()
{
  AciLoop();

  DoBluetoothTasks();  

  if(IsPathfinder){
    runPathfinder();
  }

  wdt_reset();
}
