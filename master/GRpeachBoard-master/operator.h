#ifndef _OPERATOR_H_INCLUDED
#define _OPERATOR_H_INCLUDED

#include <Arduino.h>
#include "define.h"

#define SENDDATANUM 6
#define RECIVEDATANUM 4

/* 受信で使用 */
#define UPPER_ON 0x01 //上半身マイコンの起動確認
#define UPPER_IS_OK 0x02 //上半身の初期化完了
#define HOLD_HANDLE 0x04 //ハンドル把持完了

/* 送信で使用 */
#define MASTER_ON 0x01 //マスターの起動
#define MASTER_IS_OK 0x02 //マスターの初期化完了
#define TABLE_POSITION 0x04 //テーブル防御体制へ
#define EXPAND 0x08 //展開開始
#define SENDING_TABLE_CMD 0x10 //テーブルの回転速度と回転角度をマスターから指定する //今は回転のONOFFに使用
#define TABLE_POSI_NEGATIVE 0x20 //テーブルの回転角度の正負
#define TABLE_OMEGA_NEGATIVE 0x40 //テーブル回転速度の正負
#define HANDLE 0x80 //ハンドル把持の指令
#define CON_RIGHT 0x0100
#define CON_UP 0x0200
#define CON_LEFT 0x0400

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

    unsigned int upper_cmd;

private:
    HardwareSerial *upper;
    
    int mapping(double value, double fromLow, double fromHigh, double toLow, double toHigh);

    unsigned int recv_num[4];
    unsigned int sendData[7];
    unsigned int reciveData[4];

    unsigned int /*upper_cmd,*/ pre_upper_cmd;
};

#endif