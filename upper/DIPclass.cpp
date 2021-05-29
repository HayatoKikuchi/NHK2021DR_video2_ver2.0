#include "DIPclass.h"
#include "define.h"

DipSW::DipSW()
{
    pinMode(PIN_DIP1,INPUT);
    pinMode(PIN_DIP2,INPUT);
    pinMode(PIN_DIP3,INPUT);
    pinMode(PIN_DIP4,INPUT);
}

int DipSW::getDipState()
{
    dipNum = !digitalRead(PIN_DIP1);
    dipNum |= !digitalRead(PIN_DIP2) << 1;
    dipNum |= !digitalRead(PIN_DIP3) << 2;
    dipNum |= !digitalRead(PIN_DIP4) << 3;

    return dipNum;
}

bool DipSW::getBool(int one_dip, int on_or_off)
{
    bool out;
    switch (on_or_off)
    {
    case ON: out = (dipNum & one_dip); break;
    case OFF: out = (~dipNum & one_dip); break;

    default: out = false; break;
    }
    
    return out;
}