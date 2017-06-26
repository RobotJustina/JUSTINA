#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>

// main define
#define NUM_PRODUCTS    3

/////////////////////////////////////////////////////////////////////////////////
//                                PIN DEFINES                                  //
/////////////////////////////////////////////////////////////////////////////////

#define PIN_DIST_SENSOR 2


/////////////////////////////////////////////////////////////////////////////////
//                             PROGRAM SETTINGS                                //
/////////////////////////////////////////////////////////////////////////////////

#define BAUD            115200      // baudrate   
#define VERSION         31          // this firmware version
#define SAMPLE_RATE     3           // sensor sample period (ms)
#define FILTER_POINTS   10          // sensor moving average filter points 


// misc.
#define MAX_ELEM 100
#define PIN_MOTOR_PWM 9  
#define MOTOR_THRESHOLD_IN 1
#define MOTOR_THRESHOLD_OUT 2
#define MOTOR_UP 0
#define MOTOR_DOWN 180
#define MOTOR_STOP 90
#define OUTLAYER_LIMIT 25
#define PARO_BUTTON_PIN 3



////////////////////////////////////////////////////////////////////////////////
//                       COMMUNICATION MESSAGES OPCODES                       //
////////////////////////////////////////////////////////////////////////////////

// Device ID
#define PC_ID    0
#define MY_ID    1

// MOD list
#define MOD_SYSTEM    0
#define MOD_SENSORS   1
#define MOD_MOTORS    2

// system opcodes 
#define OP_PING        0              
#define OP_VERSION     1
#define OP_LISTMOD     2
#define OP_ERROR       3
#define OP_STOP        4              


// boxes opcodes (RASP -> ARDUINO)
#define OP_DISPATCH      0
#define OP_SETSTATE      1
#define OP_SETLED        2
#define OP_GETSTOCK      3
#define OP_GETREADY      4
#define OP_SETRELAY      5

// boxes opcodes (ARDUINO -> RASP)
#define OP_BUTTON        10
#define OP_MONEY         11
#define OP_SENDSTOCK     12
#define OP_SENDREADY     13

// sensors opcodes
#define OP_SETDISTMAX        0
#define OP_GETDISTMAX        1
#define OP_SETDISTMIN        2
#define OP_GETDISTMIN        3
#define OP_GETCURRENTDIST    4
#define OP_SETTARGETVEL      5
#define OP_GETTARGETVEL      6
#define OP_GETCURRENTVEL     7

// motors opcodes
#define OP_SETTORSOPOSE      0
#define OP_CALIBRATE         1
#define OP_GOUP              2
#define OP_GODOWN            3




#endif








