#ifndef DEFINE_h
#define DEFINE_h

#include "Arduino.h"

struct coords{
    double x;
    double y;
    double z;
};

// スイッチやLEDのピン設定
#define PIN_DIP1 40
#define PIN_DIP2 41
#define PIN_DIP3 42
#define PIN_DIP4 43

#define PIN_SW_BLUE  39
#define PIN_SW_RED   38

#define PIN_LED_1   30
#define PIN_LED_2   31
#define PIN_LED_3   32
#define PIN_LED_4   33

#define PIN_LIMIT_1 36
#define PIN_LIMIT_2 37

#define PIN_FET_1 55
#define PIN_FET_2 53
#define PIN_FET_3 52
#define PIN_FET_4 21

#define PIN_SENSOR_1 A3
#define PIN_SENSOR_2 A4
#define PIN_SENSOR_3 A5

#define SERIAL_ROBOCLAW Serial0
#define SERIAL_MASTER   Serial5
#define SERIAL_CON      Serial6
#define SERIAL_XBEE     Serial7

#define PIN_INDEX_1 13
#define PIN_INDEX_2 4
#define PIN_INDEX_3 12
#define PIN_INDEX_4 5
#define PIN_INDEX_5 11
#define PIN_INDEX_6 8
#define PIN_INDEX_7 10
#define PIN_INDEX_8 9

// 制御周期
#define INT_TIME			( 0.01 )

// RoboClaw関連
#define ADR_MD1   ( 131 )

#endif
