#ifndef MANUALCONTROL_h
#define MANUALCONTROL_h

#include "define.h"
#include "PIDclass.h"

class ManualControl{
public:
    /*********** 変数宣言 ***********/

    /*********** 関数宣言 ***********/
    ManualControl(PID* _velx, PID* _vely, PID* _velz);
    
    coords getRawVel(unsigned int, unsigned int, unsigned int);

    /* 速度のPID制御をする場合に使用 */
    coords getRefVel(coords rawVel, coords robotVel, coords maxvel);

private:
    PID *velx, *vely , *velz;
    bool anglePIDEnable;
    
};

#endif