#include "define.h"

FlagChange::FlagChange(double _seconds, double _int_time)
{
    seconds = _seconds;
    int_time = _int_time;
    loop_count = 0;
}

bool FlagChange::change(bool *flag)
{
    if((int)(seconds/INT_TIME) - 1 < loop_count++)
    {
        *flag = !*flag;
        loop_count = 0;
    }
    return *flag;
}

void FlagChange::setSeconds(double setNum)
{
    seconds = setNum;
}