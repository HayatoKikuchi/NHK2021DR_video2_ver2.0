#ifndef _MASTER_H_INCLUDED
#define _MASTER_H_INCLUDED

#include <Arduino.h>
#include "define.h"

#define END_BYTE 0xB4

/* 受信で使用 */
#define UPPER_ON 0x01 //上半身マイコンの起動確認
#define UPPER_IS_OK 0x02 //上半身の初期化完了
#define HOLD_HANDLE 0x04 //ハンドル把持完了

/* 送信で使用 */
#define MASTER_ON 0x01 //マスターの起動
#define MASTER_IS_OK 0x02 //マスターの初期化完了
#define TABLE_POSITION 0x04 //テーブル防御体制へ
#define EXPAND 0x08 //展開開始
#define SENDING_TABLE_CMD 0x10 //テーブルの回転速度と回転角度をマスターから指定する
#define TABLE_POSI_NEGATIVE 0x20 //テーブルの回転角度の正負
#define TABLE_OMEGA_NEGATIVE 0x40 //テーブル回転速度の正負
#define HANDLE 0x80 //ハンドル把持の指令
#define CON_RIGHT 0x0100
#define CON_UP 0x0200
#define CON_LEFT 0x0400

#define SENDDATANUM 4
#define RECIVEDATANUM 6

class Master
{
public:
    Master(HardwareSerial *_master);

    void init_upper_cmd(void);
    void add_upper_cmd(unsigned int addNum);
    void sub_upper_cmd(unsigned int subNum);
    void sendMasterCmd();
    void updateMasterCmd(unsigned int *state, double *refAngle, double *refOmega);

    int mapping(double value, double fromLow, double fromHigh, double toLow, double toHigh);

    unsigned int master_cmd, pre_master_cmd;

private:
    HardwareSerial *master;

    uint8_t sendData[SENDDATANUM];
    uint8_t reciveData[RECIVEDATANUM];

    uint8_t recv_num[RECIVEDATANUM];

};

#endif