#ifndef _SETTING_H_INCLUDED
#define _SETTING_H_INCLUDED

#include <Arduino.h>
#include "PIDclass.h"
#include "LCDclass.h"

#define DIP1    0x01
#define DIP2    0x02
#define DIP3    0x04
#define DIP4    0x08

#define ON 1
#define OFF 2

#define SET_MAXVELOMEGA 1
#define SETVEL_X 2
#define SETVEL_Y 3
#define SETVEL_Z 4
#define SETPOSI_X 5
#define SETPOSI_Y 6
#define SETPOSI_Z 7

#define SETTINGNUM 7


class DipSW{
  public:
  DipSW();

  /* DipSwitchの状態を取得 */
  int getDipState();

  /*
  * 指定されたDipSwotchがONかOFFを指定し状態の真偽を返す
  *
  * @param one_dip DIP1~DIP4を選択．マクロ名は自分で変更可
  * @param on_or_off DipSwitchがONかOFFを選択
  * 
  */
  bool getBool(int one_dip, int on_or_off);

  private:
  int dipNum;
};


class Encorder{
  public:
  
  Encorder(double _fluctuation = 0.25);

  /* エンコーダのカウント値を返す．デフォルトでは１カチで１カウントする．*/
  int getEncCount();

  /* 
  * エンコーダのカウント値を変更 
  * 
  * @param setnum 設定したいカウント値
  */
  void setEncCount(int setnum);

  /* 
  * カウント値の増分を変更する．（デフォルトでは0.25）
  * 
  * @param setnum 設定したいカウント値の増分
  */
  void setFluctuation(double setnum);

  private:
  int enc_count;
  double count;
  int encA, pre_encA, encB, pre_encB;
  double fluctuation;
};


class PIDsetting
{
public:
    PIDsetting(PID *_pid, myLCDclass *_LCD, Encorder *_encorder);

    /* PID制御のゲイン調整に必要な変数を初期化 */
    void init_variable();

    /*
    * PID制御のゲイン調整を行う．
    *
    * @param flag_display LCDの表示を更新する周期（フラグ処理）
    * @param up オンボードの上ボタン
    * @param down オンボードの下ボタン
    * @param moji LCDに表示させる文字
    * @param flag settingを行うか否か
    */
    void task(bool flag_display,bool up, bool down,char moji[],bool flag);

private:
    PID *pid;
    myLCDclass *LCD;
    Encorder *encorder;
    bool flag_lcd;
};

#endif