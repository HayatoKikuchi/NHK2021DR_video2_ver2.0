#ifndef PIDCLASS_h
#define PIDCLASS_h

#include <Arduino.h>

struct Gain
{
    float p;
    float i;
    float d;
};


class PID
{
public:
    PID(float xKp, float xKi, float xKd, float xint_time);
    float getCmd(float ref, float act, float maxcmd);
    void PIDinit(float ref, float act);
    void setPara(float xKp, float xKi, float xKd);
    Gain getRara(void);

private:
    float preError;
    float intError;
    float Kp;
    float Ki;
    float Kd;
    float int_time;

    Gain K;
    
    boolean init_done;
    
};

#endif