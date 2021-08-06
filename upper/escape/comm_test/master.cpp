#include <Arduino.h>
#include "master.h"

Master::Master(HardwareSerial *_master)
{
    master = _master;
    Master::init_upper_cmd();
}

void Master::init_upper_cmd()
{
  Master::master_cmd = Master::pre_master_cmd = UPPER_ON;
}

void Master::add_upper_cmd(unsigned int addNum)
{
  Master::master_cmd |= addNum;
}

void Master::sub_upper_cmd(unsigned int subNum)
{
    if(Master::master_cmd & subNum) Master::master_cmd -= subNum;
}

void Master::sendMasterCmd()
{
    Master::pre_master_cmd = Master::master_cmd;

    sendData[0] = (uint8_t)master_cmd;
    sendData[1] = 0; //reserved
    uint8_t checksum = 1;
    for (int i = 0; i < (SENDDATANUM - 2); i++) checksum ^= sendData[i];
    sendData[2] = checksum;
    sendData[3] = END_BYTE;

    //if(Master::pre_master_cmd != Master::master_cmd) //コマンドが変更されたときにのみ送信
    //{
        for (int i = 0; i < 4; i++)
        {
            master->write(sendData[i]);
        }
    //}   
    
}

void Master::updateMasterCmd(unsigned int *state, double *refAngle, double *refOmega)
{
    while (master->available())
    {
        uint8_t num = master->read();
        static int loop_num = 0;
        
        if(num == END_BYTE)
        {
            if(loop_num == (RECIVEDATANUM - 1))
            {
                uint8_t checksum = 1;
                for (int i = 0; i < (RECIVEDATANUM - 2); i++) checksum ^= recv_num[i];
                
                if(checksum == recv_num[RECIVEDATANUM - 2])
                {
                    for (int i = 0; i < RECIVEDATANUM; i++) reciveData[i] = recv_num[i];
                    
                    *state = (unsigned int)reciveData[0];

                    double recvAngle, recvOmega;
                    recvAngle = (double)(mapping((double)reciveData[1], 0, 255, 0, 600)) / 10.0; //0~127の値を0~600の範囲で再マップし，1/10の位を出現させる
                    recvOmega = (double)(mapping((double)reciveData[2], 0, 255, 0, 600)) / 10.0; //0~127の値を0~600の範囲で再マップし，1/10の位を出現させる

                    if(reciveData[0] & TABLE_POSI_NEGATIVE) recvAngle *= -1.0;
                    if(reciveData[0] & TABLE_OMEGA_NEGATIVE) recvOmega *= -1.0;

                    *refAngle = recvAngle;
                    *refOmega = recvOmega;
                    
                }

            }
            
            loop_num = 0;
        }
        else
        {
            recv_num[loop_num] = num;
            loop_num++;
        }
        
    }
    
}


int Master::mapping(double value, double fromLow, double fromHigh, double toLow, double toHigh)
{
    return (int)(value / (fromHigh - fromLow) * (toHigh - toLow));
}