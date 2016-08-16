#include <SPI.h>
#include <EEPROM.h>
#include <lib_aci.h>
#include <aci_setup.h>

int upperBoundary = 150 ;
int lowerBoundary = 50 ;

int rightMotorPin = 5; //right motor speed control pin
int leftMotorPin = 6; //left motor speed control pin

int button1Pin = A2;
int button2Pin = A0;
int button3Pin = A5;

int buzzerPin= 3;

int ultrasonicDataPin = 7; //ultrasonic sensor data read pin
int a = 1;//runPathfinder() control variable

/**
Put the nRF8001 setup in the RAM of the nRF8001.
*/
#include "services.h"

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t
        services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif
static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

//@todo have an aci_struct that will contain
// total initial credits
// current credit
// current state of the aci (setup/standby/active/sleep)
// open remote pipe pending
// close remote pipe pending
// Current pipe available bitmap
// Current pipe closed bitmap
// Current connection interval, slave latency and link supervision timeout
// Current State of the the GATT client (Service Discovery)
// Status of the bond (R) Peer address
static struct aci_state_t aci_state;
static hal_aci_evt_t aci_data;
//static hal_aci_data_t aci_cmd;

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}

void setup(void)
{
  buttonBuzzerInitialization();
  pathfinderInitialization();
  Serial.begin(115200);
  Serial.println(F("Arduino setup"));

  /**
  Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
  */
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = (hal_aci_data_t*) setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

  /*
  Tell the ACI library, the MCU to nRF8001 pin connections.
  The Active pin is optional and can be marked UNUSED
  */
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details
  aci_state.aci_pins.reqn_pin   = 9;
  aci_state.aci_pins.rdyn_pin   = 8;
  aci_state.aci_pins.mosi_pin   = MOSI;
  aci_state.aci_pins.miso_pin   = MISO;
  aci_state.aci_pins.sck_pin    = SCK;

  aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 2MHz SPI speed
                                                             //SPI_CLOCK_DIV16 = 1MHz SPI speed

  aci_state.aci_pins.reset_pin              = 4; //4 for Nordic board, UNUSED for REDBEARLABS
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false;
  aci_state.aci_pins.interrupt_number       = 1;

  /* We initialize the data structures required to setup the nRF8001
  */
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false);
}
int getUltrasonicData(){
  return ( pulseIn(ultrasonicDataPin, HIGH) / 147 ) * 2.54;
}
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
    analogWrite(buzzerPin , 50);
    delay(50);
    analogWrite(buzzerPin , 0);
  }
  if(analogRead(button2Pin) > 10){
    analogWrite(buzzerPin , 100);
    delay(50);
    analogWrite(buzzerPin , 0);
  }
 if(analogRead(button3Pin) > 10){
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
 
void loop()
{
  while(a != 0){
    BluControl();
    runPathfinder();
    buttonBuzzer();
  }
    BluControl();
    buttonBuzzer();
  
}
void BluControl(){
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;

    aci_evt = &aci_data.evt;
    switch(aci_evt->evt_opcode)
    {
      /**
      As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
      case ACI_EVT_DEVICE_STARTED:
      {
        aci_state.data_credit_available = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            Serial.println(F("Evt Device Started: Setup"));
            setup_required = true;
            break;

          case ACI_DEVICE_STANDBY:
            Serial.println(F("Evt Device Started: Standby"));
            if (aci_evt->params.device_started.hw_error)
            {
              delay(20); //Magic number used to make sure the HW error event is handled correctly.
            }
            else
            {
            lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
            Serial.println(F("Advertising started"));
            }
            break;
        }
      }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.println(F("Evt Cmd respone: Error. Arduino is in an while(1); loop"));
          while (1);
        }
        break;

      case ACI_EVT_CONNECTED:
        Serial.println(F("Evt Connected"));
        break;

      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        break;

      case ACI_EVT_DISCONNECTED:
        Serial.println(F("Evt Disconnected/Advertising timed out"));
        lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
        Serial.println(F("Advertising started"));
        break;

      case ACI_EVT_PIPE_ERROR:
        //See the appendix in the nRF8001 Product Specication for details on the error codes
        Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        Serial.print(F("  Pipe Error Code: 0x"));
        Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;
      
      case ACI_EVT_DATA_RECEIVED:
        if(1 == aci_evt->params.data_received.rx_data.pipe_number)
        {
            Serial.print("Algoritma Durumu : ");
            if(1 == aci_evt->params.data_received.rx_data.aci_data[0])
             {
                a=1;
                Serial.println("ACIK.");
                
             }
            else
             {
                a=0;
                Serial.println("KAPALI.");
             }                
        }
        
        else if(2 == aci_evt->params.data_received.rx_data.pipe_number)
        {
            unsigned int RightMotorValue = 0;
            Serial.print("Sag Motor Degeri : ");
            RightMotorValue = aci_evt->params.data_received.rx_data.aci_data[0] ;
            RightMotorValue = RightMotorValue * 255 / 153;
            Serial.println(RightMotorValue); 
            analogWrite(rightMotorPin,RightMotorValue );
            
        }
        else if(3 == aci_evt->params.data_received.rx_data.pipe_number)
        {
            unsigned int LeftMotorValue = 0;
            Serial.print("Sol Motor Degeri : ");
            LeftMotorValue = aci_evt->params.data_received.rx_data.aci_data[0] ;
            LeftMotorValue = LeftMotorValue * 255 / 153;
            Serial.println(LeftMotorValue); 
            analogWrite(leftMotorPin, LeftMotorValue);
        }
        else if(4 == aci_evt->params.data_received.rx_data.pipe_number)
        {
            Serial.print("Buzzer Durumu : ");
            if(1 == aci_evt->params.data_received.rx_data.aci_data[0])
             {
              Serial.print("acik");
                analogWrite(buzzerPin , 200);
             }
            else
             {              
              Serial.println("kapali");
                analogWrite(buzzerPin , 0);
             }                
        }     
        else if(5 == aci_evt->params.data_received.rx_data.pipe_number) 
        {
         // notify gelicek !!                   
        }   
        else if(6 == aci_evt->params.data_received.rx_data.pipe_number)
        {
        // notify gelicek              
        }  
        else if(7 == aci_evt->params.data_received.rx_data.pipe_number)
        {
        // notify gelicek              
        }  
        else if(8 == aci_evt->params.data_received.rx_data.pipe_number)
        {
        // notify gelicek          
        }          
        break;
        
      case ACI_EVT_HW_ERROR:
        Serial.print(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
        Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        Serial.println();
        lib_aci_connect(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
        Serial.println(F("Advertising started"));
        break;
    }
  }
  else
  {
    //Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }

  /* setup_required is set to true when the device starts up and enters setup mode.
   * It indicates that do_aci_setup() should be called. The flag should be cleared if
   * do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   */
  if(setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }
 
}
