#ifndef _MASTER_H_INCLUDED
#define _MASTER_H_INCLUDED

#include <Arduino.h>
#include "define.h"

#define END_BYTE 0xB4

/* 送信で使用 */
#define UPPER_ON 0x01
#define UPPER_IS_OK 0x02
#define HOLD_HANDLE 0x04

/* 受信で使用 */
#define MASTER_ON 0x01
#define MASTER_IS_OK 0x02
#define TABLE_POSITION 0x04
#define EXPAND 0x08
#define SENDING_TABLE_CMD 0x10
#define TABLE_POSI_NEGATIVE 0x20
#define TABLE_OMEGA_NEGATIVE 0x40
#define HANDLE 0x80

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