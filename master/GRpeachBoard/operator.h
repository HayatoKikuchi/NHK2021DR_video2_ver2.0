#ifndef _OPERATOR_H_INCLUDED
#define _OPERATOR_H_INCLUDED

#include <Arduino.h>
#include "define.h"

class Operator
{
public:
    /*********** 変数宣言 ***********/

    /*********** 関数宣言 ***********/
    Operator();
    void setup(void); //DRに関するセットアップを行う
    void allOutputLow(void); //全てのデジタル出力をLOWにする
    void LEDblink(byte pin, int times, int interval); //LEDを点滅させる
    void RGB_led(int period); //フルカラーLEDを奇麗に光らせる

private:
};

#endif