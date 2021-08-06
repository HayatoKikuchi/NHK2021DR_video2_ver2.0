#ifndef MANUALCONTROL_h
#define MANUALCONTROL_h

#include "define.h"

class ManualControl{
public:
    /*********** 変数宣言 ***********/

    /*********** 関数宣言 ***********/
    ManualControl();
    
    int init();
    coords getRefVel(unsigned int, unsigned int, unsigned int);
    coords getLocalVel(coords globalVel, double robotAngle);

private:
    bool anglePIDEnable;
    
};

#endif