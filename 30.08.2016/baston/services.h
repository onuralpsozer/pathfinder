/**
* This file is autogenerated by nRFgo Studio 1.21.2RC1.4
*/

#ifndef SETUP_MESSAGES_H__
#define SETUP_MESSAGES_H__

#include "hal_platform.h"
#include "aci.h"


#define SETUP_ID 0
#define SETUP_FORMAT 3 /** nRF8001 D */
#define ACI_DYNAMIC_DATA_SIZE 157

/* Service: Baston Service - Characteristic: Algorithm - Pipe: RX */
#define PIPE_BASTON_SERVICE_ALGORITHM_RX          1
#define PIPE_BASTON_SERVICE_ALGORITHM_RX_MAX_SIZE 1

/* Service: Baston Service - Characteristic: MotorControl - Pipe: RX */
#define PIPE_BASTON_SERVICE_MOTORCONTROL_RX          2
#define PIPE_BASTON_SERVICE_MOTORCONTROL_RX_MAX_SIZE 2

/* Service: Baston Service - Characteristic: Buzzer Control - Pipe: RX */
#define PIPE_BASTON_SERVICE_BUZZER_CONTROL_RX          3
#define PIPE_BASTON_SERVICE_BUZZER_CONTROL_RX_MAX_SIZE 2

/* Service: Baston Service - Characteristic: Ultrasonic Sensor - Pipe: TX */
#define PIPE_BASTON_SERVICE_ULTRASONIC_SENSOR_TX          4
#define PIPE_BASTON_SERVICE_ULTRASONIC_SENSOR_TX_MAX_SIZE 1

/* Service: Baston Service - Characteristic: Button1 - Pipe: TX */
#define PIPE_BASTON_SERVICE_BUTTON1_TX          5
#define PIPE_BASTON_SERVICE_BUTTON1_TX_MAX_SIZE 1

/* Service: Baston Service - Characteristic: Button2 - Pipe: TX */
#define PIPE_BASTON_SERVICE_BUTTON2_TX          6
#define PIPE_BASTON_SERVICE_BUTTON2_TX_MAX_SIZE 1

/* Service: Battery - Characteristic: Battery Level - Pipe: TX */
#define PIPE_BATTERY_BATTERY_LEVEL_TX          7
#define PIPE_BATTERY_BATTERY_LEVEL_TX_MAX_SIZE 1


#define NUMBER_OF_PIPES 7

#define SERVICES_PIPE_TYPE_MAPPING_CONTENT {\
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_RX},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
  {ACI_STORE_LOCAL, ACI_TX},   \
}

#define GAP_PPCP_MAX_CONN_INT 0xffff /**< Maximum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_MIN_CONN_INT  0xffff /**< Minimum connection interval as a multiple of 1.25 msec , 0xFFFF means no specific value requested */
#define GAP_PPCP_SLAVE_LATENCY 0
#define GAP_PPCP_CONN_TIMEOUT 0xffff /** Connection Supervision timeout multiplier as a multiple of 10msec, 0xFFFF means no specific value requested */

#define NB_SETUP_MESSAGES 28
#define SETUP_MESSAGES_CONTENT {\
    {0x00,\
        {\
            0x07,0x06,0x00,0x00,0x03,0x02,0x42,0x07,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x00,0x07,0x01,0x01,0x00,0x00,0x06,0x00,0x01,\
            0xd1,0x0f,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x1c,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x03,0x90,0x01,0xff,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x10,0x38,0xff,0xff,0x02,0x58,0x0a,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x05,0x06,0x10,0x54,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x00,0x04,0x04,0x02,0x02,0x00,0x01,0x28,0x00,0x01,0x00,0x18,0x04,0x04,0x05,0x05,0x00,\
            0x02,0x28,0x03,0x01,0x02,0x03,0x00,0x00,0x2a,0x04,0x04,0x14,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x1c,0x06,0x00,0x03,0x2a,0x00,0x01,0x42,0x61,0x73,0x74,0x6f,0x6e,0x73,0x65,0x6d,0x69,\
            0x2e,0x63,0x6f,0x6d,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x38,0x05,0x05,0x00,0x04,0x28,0x03,0x01,0x02,0x05,0x00,0x01,0x2a,0x06,0x04,0x03,0x02,\
            0x00,0x05,0x2a,0x01,0x01,0x00,0x00,0x04,0x04,0x05,0x05,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x54,0x06,0x28,0x03,0x01,0x02,0x07,0x00,0x04,0x2a,0x06,0x04,0x09,0x08,0x00,0x07,0x2a,\
            0x04,0x01,0xff,0xff,0xff,0xff,0x00,0x00,0xff,0xff,0x04,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x70,0x02,0x02,0x00,0x08,0x28,0x00,0x01,0x01,0x18,0x04,0x04,0x10,0x10,0x00,0x09,0x28,\
            0x00,0x01,0x86,0x85,0x19,0x52,0x1e,0xc2,0xdd,0x99,0x93,0x4a,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0x8c,0x5a,0xb8,0x00,0x10,0xbe,0xdf,0x04,0x04,0x13,0x13,0x00,0x0a,0x28,0x03,0x01,0x06,\
            0x0b,0x00,0x86,0x85,0x19,0x52,0x1e,0xc2,0xdd,0x99,0x93,0x4a,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xa8,0x5a,0xb8,0x01,0x10,0xbe,0xdf,0x46,0x14,0x02,0x01,0x00,0x0b,0x10,0x01,0x02,0x00,\
            0x04,0x04,0x13,0x13,0x00,0x0c,0x28,0x03,0x01,0x06,0x0d,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xc4,0x86,0x85,0x19,0x52,0x1e,0xc2,0xdd,0x99,0x93,0x4a,0x5a,0xb8,0x02,0x10,0xbe,0xdf,\
            0x46,0x14,0x03,0x02,0x00,0x0d,0x10,0x02,0x02,0x00,0x00,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xe0,0x04,0x13,0x13,0x00,0x0e,0x28,0x03,0x01,0x06,0x0f,0x00,0x86,0x85,0x19,0x52,0x1e,\
            0xc2,0xdd,0x99,0x93,0x4a,0x5a,0xb8,0x03,0x10,0xbe,0xdf,0x46,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x20,0xfc,0x14,0x03,0x02,0x00,0x0f,0x10,0x03,0x02,0x00,0x00,0x04,0x04,0x13,0x13,0x00,0x10,\
            0x28,0x03,0x01,0x12,0x11,0x00,0x86,0x85,0x19,0x52,0x1e,0xc2,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x18,0xdd,0x99,0x93,0x4a,0x5a,0xb8,0x04,0x10,0xbe,0xdf,0x16,0x04,0x02,0x01,0x00,0x11,\
            0x10,0x04,0x02,0x00,0x46,0x14,0x03,0x02,0x00,0x12,0x29,0x02,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x34,0x01,0x00,0x00,0x04,0x04,0x13,0x13,0x00,0x13,0x28,0x03,0x01,0x12,0x14,0x00,0x86,\
            0x85,0x19,0x52,0x1e,0xc2,0xdd,0x99,0x93,0x4a,0x5a,0xb8,0x05,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x50,0x10,0xbe,0xdf,0x16,0x04,0x02,0x01,0x00,0x14,0x10,0x05,0x02,0x00,0x46,0x14,0x03,\
            0x02,0x00,0x15,0x29,0x02,0x01,0x00,0x00,0x04,0x04,0x13,0x13,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x6c,0x00,0x16,0x28,0x03,0x01,0x12,0x17,0x00,0x86,0x85,0x19,0x52,0x1e,0xc2,0xdd,0x99,\
            0x93,0x4a,0x5a,0xb8,0x06,0x10,0xbe,0xdf,0x16,0x04,0x02,0x01,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0x88,0x00,0x17,0x10,0x06,0x02,0x00,0x46,0x14,0x03,0x02,0x00,0x18,0x29,0x02,0x01,0x00,\
            0x00,0x04,0x04,0x02,0x02,0x00,0x19,0x28,0x00,0x01,0x0f,0x18,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x21,0xa4,0x04,0x04,0x05,0x05,0x00,0x1a,0x28,0x03,0x01,0x12,0x1b,0x00,0x19,0x2a,0x16,0x04,\
            0x02,0x01,0x00,0x1b,0x2a,0x19,0x01,0x10,0x46,0x14,0x03,0x02,\
        },\
    },\
    {0x00,\
        {\
            0x0b,0x06,0x21,0xc0,0x00,0x1c,0x29,0x02,0x01,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x00,0x10,0x01,0x02,0x00,0x08,0x04,0x00,0x0b,0x00,0x00,0x10,0x02,0x02,0x00,0x08,0x04,\
            0x00,0x0d,0x00,0x00,0x10,0x03,0x02,0x00,0x08,0x04,0x00,0x0f,\
        },\
    },\
    {0x00,\
        {\
            0x1f,0x06,0x40,0x1c,0x00,0x00,0x10,0x04,0x02,0x00,0x02,0x04,0x00,0x11,0x00,0x12,0x10,0x05,0x02,0x00,\
            0x02,0x04,0x00,0x14,0x00,0x15,0x10,0x06,0x02,0x00,0x02,0x04,\
        },\
    },\
    {0x00,\
        {\
            0x11,0x06,0x40,0x38,0x00,0x17,0x00,0x18,0x2a,0x19,0x01,0x00,0x02,0x04,0x00,0x1b,0x00,0x1c,\
        },\
    },\
    {0x00,\
        {\
            0x13,0x06,0x50,0x00,0x86,0x85,0x19,0x52,0x1e,0xc2,0xdd,0x99,0x93,0x4a,0x5a,0xb8,0x00,0x00,0xbe,0xdf,\
        },\
    },\
    {0x00,\
        {\
            0x18,0x06,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
            0x00,0x00,0x00,0x00,0x00,\
        },\
    },\
    {0x00,\
        {\
            0x06,0x06,0xf0,0x00,0x03,0x5e,0x3b,\
        },\
    },\
}

#endif