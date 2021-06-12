#ifndef _OPERATOR_H_INCLUDED
#define _OPERATOR_H_INCLUDED

#include <Arduino.h>
#include "define.h"

#define SENDDATANUM 6
#define RECVDATANUM 4

/* 受信で使用 */
#define UPPER_ON 0x01
#define UPPER_IS_OK 0x02
#define HOLD_HANDLE 0x04

/* 送信で使用 */
#define MASTER_ON 0x01
#define MASTER_IS_OK 0x02
#define TABLE_POSITION 0x04
#define EXPAND 0x08
#define SENDING_TABLE_CMD 0x10
#define TABLE_POSI_NEGATIVE 0x20
#define TABLE_OMEGA_NEGATIVE 0x40
#define HANDLE 0x80

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

    void init_upper_cmd(void);
    void add_upper_cmd(unsigned int addNum);
    void sub_upper_cmd(unsigned int subNum);
    void sendUpperCmd(double refAngle = 0.0, double refOmega = 0.0);
    void updateUpperCmd(unsigned int *cmd);

private:
    HardwareSerial *upper;
    
    int mapping(int value, int fromLow, int fromHigh, int toLow, int toHigh);

    unsigned int recv_num[4];
    unsigned int sendData[7];
    unsigned int reciveData[4];

    unsigned int upper_cmd, pre_upper_cmd;
};

#endif