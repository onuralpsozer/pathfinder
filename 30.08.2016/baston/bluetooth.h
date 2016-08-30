#include <SPI.h>
#include <EEPROM.h>
#include <lib_aci.h>
#include <aci_setup.h>

#include"pathfinder.h"

/**
Put the nRF8001 setup in the RAM of the nRF8001.
*/
#include "services.h"
/**
Include the services_lock.h to put the setup in the OTP memory of the nRF8001.
This would mean that the setup cannot be changed once put in.
However this removes the need to do the setup of the nRF8001 on every reset.
*/


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

static bool radio_ack_pending  = false;
static bool timing_change_done = false;

// Notification queue parameters
enum queue{
  UltrasonicData,
  Button1,
  Button2,
  BatteryLevel
};

int toBeNotified = UltrasonicData;

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

void InitializeBluetooth(void)
{
  Serial.println("Bluetooth Setup");

  
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

  aci_state.aci_pins.reset_pin              = UNUSED; //4 for Nordic board, UNUSED for REDBEARLABS
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false;
  aci_state.aci_pins.interrupt_number       = 1;

  /* We initialize the data structures required to setup the nRF8001
  */
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false);
}

void AciLoop (void)
{
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
        aci_state.data_credit_total = 2;//aci_evt->params.device_started.credit_total;
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
          Serial.println(F("Evt Cmd respone: Error. Waiting in while(1) loop."));
          while(1);
        }
        break;

      case ACI_EVT_CONNECTED:
        Serial.println(F("Evt Connected"));
        radio_ack_pending = false;
        timing_change_done = false;
        aci_state.data_credit_available = 2;//aci_state.data_credit_total;
        break;

      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        if ((lib_aci_is_pipe_available(&aci_state, PIPE_BASTON_SERVICE_ULTRASONIC_SENSOR_TX) 
        || lib_aci_is_pipe_available(&aci_state, PIPE_BASTON_SERVICE_BUTTON1_TX)
        || lib_aci_is_pipe_available(&aci_state, PIPE_BASTON_SERVICE_BUTTON2_TX)
        || lib_aci_is_pipe_available(&aci_state, PIPE_BATTERY_BATTERY_LEVEL_TX))
        && (false == timing_change_done) )
        {
          /*
          Request a change to the link timing as set in the GAP -> Preferred Peripheral Connection Parameters
          Change the setting in nRFgo studio -> nRF8001 configuration -> GAP Settings and recompile the xml file.
          */
          lib_aci_change_timing_GAP_PPCP();
          timing_change_done = true;
        }
        break;
        
      case ACI_EVT_TIMING:
        /*
        Link timing has changed.
        */
        Serial.print(F("Timing changed: "));
        Serial.println(aci_evt->params.timing.conn_rf_interval, HEX);
        break;

      case ACI_EVT_DISCONNECTED:
        Serial.println(F("Evt Disconnected/Advertising timed out"));
        lib_aci_connect(180/* in seconds */, 0x0100 /* advertising interval 100ms*/);
        Serial.println(F("Advertising started"));
        break;
        
      case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        /**
        Bluetooth Radio ack received from the peer radio for the data packet sent.
        This also signals that the buffer used by the nRF8001 for the data packet is available again.
        */
        radio_ack_pending = false;
        break;

      case ACI_EVT_PIPE_ERROR:
        /**
        Send data failed. ACI_EVT_DATA_CREDIT will not come.
        This can happen if the pipe becomes unavailable by the peer unsubscribing to the Heart Rate
        Measurement characteristic.
        This can also happen when the link is disconnected after the data packet has been sent.
        */
        radio_ack_pending = false;
      
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

        if(PIPE_BASTON_SERVICE_ALGORITHM_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
            Serial.print("Algoritma Durumu : ");
            if(1 == aci_evt->params.data_received.rx_data.aci_data[0])
             {
                IsPathfinder = true;
                Serial.println("ACIK.");
                
             }
            else
             {
                analogWrite(leftMotorPin,0);
                analogWrite(rightMotorPin,0);
                IsPathfinder = false;
                Serial.println("KAPALI.");
             }                
        }
        
        else if(PIPE_BASTON_SERVICE_MOTORCONTROL_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
            if(0x00 == aci_evt->params.data_received.rx_data.aci_data[0])
            {
              Serial.println("Motorlar Durdu.");
              StopDriving(rightMotorPin);
              StopDriving(leftMotorPin);
            }
            else if(0x01 == aci_evt->params.data_received.rx_data.aci_data[0]){
              Serial.print("Sag motor calisti. PWM:");
              Serial.println(aci_evt->params.data_received.rx_data.aci_data[1],HEX);
              Drive(rightMotorPin, aci_evt->params.data_received.rx_data.aci_data[1]);
            }
            else if(0x10 == aci_evt->params.data_received.rx_data.aci_data[0]){
              Serial.print("Sol motor calisti. PWM:");
              Serial.println(aci_evt->params.data_received.rx_data.aci_data[1],HEX);
              Drive(leftMotorPin, aci_evt->params.data_received.rx_data.aci_data[1]);
            }
            else if(0x11 == aci_evt->params.data_received.rx_data.aci_data[0]){
              Serial.print("Iki motorda calisti. PWM:");
              Serial.println(aci_evt->params.data_received.rx_data.aci_data[1],HEX);
              Drive(rightMotorPin, aci_evt->params.data_received.rx_data.aci_data[1]);
              Drive(leftMotorPin, aci_evt->params.data_received.rx_data.aci_data[1]);
            } 
        }
        else if(PIPE_BASTON_SERVICE_BUZZER_CONTROL_RX == aci_evt->params.data_received.rx_data.pipe_number)
        {
          if(0x00 == aci_evt->params.data_received.rx_data.aci_data[0]){
            Serial.println("Buzzer durduruldu.");
            StopDriving(buzzerPin);
            //digitalWrite(buzzerPin,LOW);
          }
          else if(0x01 == aci_evt->params.data_received.rx_data.aci_data[0]){
            Serial.print("Buzzer calistirildi. PWM:");
            Serial.println(aci_evt->params.data_received.rx_data.aci_data[1],HEX);
            Drive(buzzerPin, aci_evt->params.data_received.rx_data.aci_data[1]);
            //digitalWrite(buzzerPin,HIGH);
          }
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
      Serial.println("Bluetooth chip setup OK");
      setup_required = false;
    }
    else
    {
      Serial.println("Bluetooth chip setup NOT OK");
    }
  }
}

void DoBluetoothTasks(void) 
{
  if((false == radio_ack_pending) && (true == timing_change_done) && (aci_state.data_credit_available >= 1) )
  {
    switch(toBeNotified){
      case UltrasonicData:
          toBeNotified = Button1;
          if (lib_aci_is_pipe_available(&aci_state, PIPE_BASTON_SERVICE_ULTRASONIC_SENSOR_TX))
          {
            uint8_t UltrasonicSensorData = getUltrasonicData();
            if(lib_aci_send_data(PIPE_BASTON_SERVICE_ULTRASONIC_SENSOR_TX, &UltrasonicSensorData, sizeof(UltrasonicSensorData))) // over the air via bluetooth
            {
              aci_state.data_credit_available--;
              Serial.print(F("UltrasonicSensorData sent: "));
              Serial.println(UltrasonicSensorData);
              radio_ack_pending = true;
            }
          }
      break;

      case Button1:
          toBeNotified = Button2;
          if (lib_aci_is_pipe_available(&aci_state, PIPE_BASTON_SERVICE_BUTTON1_TX))
          {
            if(lib_aci_send_data(PIPE_BASTON_SERVICE_BUTTON1_TX, &Button1State, sizeof(Button1State))) // over the air via bluetooth
            {
              aci_state.data_credit_available--;
              Serial.print(F("Button1State sent: "));
              Serial.println(Button1State);
              radio_ack_pending = true;
            }
          }
      break;

      case Button2:
          toBeNotified = BatteryLevel;
          if (lib_aci_is_pipe_available(&aci_state, PIPE_BASTON_SERVICE_BUTTON2_TX))
          {
            if(lib_aci_send_data(PIPE_BASTON_SERVICE_BUTTON2_TX, &Button2State, sizeof(Button2State))) // over the air via bluetooth
            {
              aci_state.data_credit_available--;
              Serial.print(F("Button2State sent: "));
              Serial.println(Button2State);
              radio_ack_pending = true;
            }
          }
      break;

      case BatteryLevel:
          toBeNotified = UltrasonicData;
          if (lib_aci_is_pipe_available(&aci_state, PIPE_BATTERY_BATTERY_LEVEL_TX))
          {
            uint8_t batteryLevelPercent = batteryLevel();
            if(lib_aci_send_data(PIPE_BATTERY_BATTERY_LEVEL_TX, &batteryLevelPercent, sizeof(batteryLevelPercent))) // over the air via bluetooth
            {
              aci_state.data_credit_available--;
              Serial.print(F("Battery Level Percentage :"));
              Serial.println(batteryLevelPercent);
              radio_ack_pending = true;
            }
          }
      break;
    }// end of switch
    
  }//end of if
}
