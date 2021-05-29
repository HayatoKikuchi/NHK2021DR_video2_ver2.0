#ifndef PLATFORM_h
#define PLATFORM_h

#include "define.h"

class Platform{
public:
    /*********** 変数宣言 ***********/

    /*********** 関数宣言 ***********/
    Platform(int, int, int, int);
    Platform(int, int, int);
    Platform();
    void platformInit(coords);
    void setPosi(coords);
    coords getPosi(int, int, double);
    void VelocityControl(coords);

private:
    coords Posi;
    int preEncX, preEncY;
    double pre_angle_rad;
    bool init_done;
    int rotateDir[4];
};

#endif