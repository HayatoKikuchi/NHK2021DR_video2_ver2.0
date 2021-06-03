#ifndef _OPERATOR_H_INCLUDED
#define _OPERATOR_H_INCLUDED

#include <Arduino.h>
#include "define.h"

#define END_BYTE 0xB4

class Operator
{
public:
    /*********** 変数宣言 ***********/

    /*********** 関数宣言 ***********/
    Operator(HardwareSerial *_upper);
    void setup(void); //DRに関するセットアップを行う
    void allOutputLow(void); //全てのデジタル出力をLOWにする
    void LEDblink(byte pin, int times, int interval); //LEDを点滅させる
    void RGB_led(int period); //フルカラーLEDを奇麗に光らせる

    void sendUpperCmd();
    void updateUpperCmd(uint8_t *status);

    uint8_t sendData[7];
    uint8_t reciveData[4];

private:
    HardwareSerial *upper;

    uint8_t recv_num[4];

};

#endif