#ifndef DEFINE_h
#define DEFINE_h

#include <Arduino.h>

struct coords
{
    double x;
    double y;
    double z;
};

/* シリアル通信の定義 */
#define SERIAL_PC       Serial
#define SERIAL_ROBOCLAW Serial1 //ダミー
#define SERIAL_MASTER   Serial2 //ダミー
#define SERIAL_CON      Serial3 //ダミー

/* 入力ピンの定義 */
#define PIN_DIP1 36 //ダミー
#define PIN_DIP2 37 //ダミー
#define PIN_DIP3 38 //ダミー
#define PIN_DIP4 39 //ダミー

/* 出力ピンの定義 */
#define PIN_EXPAND 40

/* 出力の定義 */
#define LED_OLL_LOW  0x00
#define LED_OLL_HIGH 0x0F

/* 制御（処理）に関する定義 */
#define INT_TIME ( 0.01 )
#define INT_TIME_MS ( INT_TIME * 1000 )

#endif
