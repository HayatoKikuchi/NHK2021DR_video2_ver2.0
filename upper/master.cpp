#include <Arduino.h>
#include "master.h"

Master::Master(HardwareSerial *_master)
{
    master = _master;
}

void Master::sendMasterCmd()
{
    sendData[0] = 1; //ダミー
    sendData[1] = 2; //ダミー
    sendData[2] = (sendData[0] ^ sendData[1]);
    sendData[3] = END_BYTE;

    for (int i = 0; i < 4; i++)
    {
        master->write(sendData[i]);
    }
    
}

void Master::updateMasterCmd(uint8_t *status, coords *posi)
{
    while (master->available())
    {
        uint8_t num = master->read();
        static int loop_num = 0;
        if(num == END_BYTE)
        {
            uint8_t checksum = recv_num[0] ^ recv_num[1];
            for (int i = 2; i < 5; i++)
            {
                checksum ^= recv_num[i];
            }
            
            if(checksum == recv_num[5])
            {
                for (int i = 0; i < 7; i++)
                {
                    reciveData[i] = recv_num[i];
                }
            }

            *status = reciveData[0];
            *posi = {(double)reciveData[1], (double)reciveData[2], (double)reciveData[3]};

            loop_num = 0;
        }
        else
        {
            recv_num[loop_num] = num;
            loop_num++;
        }
    }
    
}