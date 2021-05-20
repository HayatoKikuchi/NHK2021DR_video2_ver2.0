// ゲームコントローラのジョイスティックデータから，ローパスフィルタを用いて指令速度を生成するクラス
// 
// 作成日：2019年12月30日
// 作成者：上野祐樹
// 旋回の調整など細かい動作を追加（菊池隼）

#include "ManualControl.h"
#include "Filter.h"

Filter velX_filter(INT_TIME);
Filter velY_filter(INT_TIME);
Filter velZ_filter(INT_TIME);

ManualControl::ManualControl(PID* _velx, PID* _vely, PID* _velz)
{
    velx = _velx;
    vely = _vely;
    velz = _velz;
    velX_filter.setLowPassPara(0.10, 0.0);//ローパスフィルタのTと初期値を設定
    velY_filter.setLowPassPara(0.10, 0.0);//ローパスフィルタのTと初期値を設定
    velZ_filter.setLowPassPara(0.10, 0.0);//ローパスフィルタのTと初期値を設定

    anglePIDEnable = false;
}



coords ManualControl::getRawVel(unsigned int JoyX, unsigned int JoyY, unsigned int JoyZ)
{
    int joyraw;
    coords rawV, refV;

    // ジョイスティックから指令速度を計算する
    joyraw = (int)JoyX - 127; 
    if(abs(joyraw) >= JOY_DEADBAND){
      if(joyraw >= 0){
        joyraw -= JOY_DEADBAND;
      }else{
        joyraw += JOY_DEADBAND;
      }
      rawV.x = -(joyraw)/(127.0 - (double)JOY_DEADBAND) * JOY_MAXVEL;
    }else{
      rawV.x = 0.0;
    }
    refV.x = velX_filter.LowPassFilter(rawV.x);

    joyraw = (int)JoyY - 127; 
    if(abs(joyraw) >= JOY_DEADBAND){
      if(joyraw >= 0){
        joyraw -= JOY_DEADBAND;
      }else{
        joyraw += JOY_DEADBAND;
      }
      rawV.y = -(joyraw)/(127.0 - (double)JOY_DEADBAND) * JOY_MAXVEL;
    }else{
      rawV.y = 0.0;
    }
    refV.y = velY_filter.LowPassFilter(rawV.y);

    joyraw = (int)JoyZ - 127; 
    if(abs(joyraw) >= JOY_DEADBAND){
      if(joyraw >= 0){
        joyraw -= JOY_DEADBAND;
      }else{
        joyraw += JOY_DEADBAND;
      }
      rawV.z = -(joyraw)/(127.0 - (double)JOY_DEADBAND) * JOY_MAXANGVEL;
    }else{
      rawV.z = 0.0;
    }
    refV.z = velZ_filter.LowPassFilter(rawV.z);

    return refV;
}

coords ManualControl::getRefVel(coords rawVel, coords robotVel, coords maxvel)
{
  coords refvel;
  refvel.x = velx->getCmd(rawVel.x, robotVel.x, maxvel.x);
  refvel.y = vely->getCmd(rawVel.y, robotVel.y, maxvel.y);
  refvel.z = velz->getCmd(rawVel.z, robotVel.z, maxvel.z);

  return refvel;
}