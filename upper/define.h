#ifndef DEFINE_h
#define DEFINE_h

#include "Arduino.h"

struct coords{
    double x;
    double y;
    double z;
};

#define SERIAL_PC       Serial
#define SERIAL_ROBOCLAW Serial1
#define SERIAL_MASTER   Serial2
#define SERIAL_CON      Serial3

#define PIN_DIP1 36
#define PIN_DIP2 37
#define PIN_DIP3 38
#define PIN_DIP4 39

#define INT_TIME 0.01
#define INT_TIME_MS INT_TIME*1000

#endif
