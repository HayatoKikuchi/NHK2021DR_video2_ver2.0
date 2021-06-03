#ifndef _MASTER_H_INCLUDED
#define _MASTER_H_INCLUDED

#include <Arduino.h>
#include "define.h"

#define END_BYTE 0xB4

#define MSTER_ON 0x01
#define MASTER_OK 0x02
#define INIT_TABLE 0x04
#define HOLD_HANDLE 0x10
#define HOLD_OK 0x20

class Master
{
public:
    Master(HardwareSerial *_master);
    
    void sendMasterCmd();
    void updateMasterCmd(uint8_t *status, coords *posi);

    uint8_t sendData[4];
    uint8_t reciveData[7];

private:
    HardwareSerial *master;

    uint8_t recv_num[7];

};

#endif