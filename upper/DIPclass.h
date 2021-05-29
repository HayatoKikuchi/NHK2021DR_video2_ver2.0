#ifndef _SETTING_H_INCLUDED
#define _SETTING_H_INCLUDED

#include <Arduino.h>
#include "define.h"

#define DIP1    0x01
#define DIP2    0x02
#define DIP3    0x04
#define DIP4    0x08

#define ON 1
#define OFF 2

class DipSW
{
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

#endif