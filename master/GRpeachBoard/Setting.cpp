#include "Setting.h"

Encorder::Encorder(double _fluctuation){
    fluctuation = _fluctuation;

    pinMode(PIN_ENC_A,INPUT);
    pinMode(PIN_ENC_B,INPUT);
    
    pre_encA = digitalRead(PIN_ENC_A);
    pre_encB = digitalRead(PIN_ENC_B);
    count = 0.0;
    enc_count = 0;
}

DipSW::DipSW()
{
  pinMode(PIN_DIP1, INPUT);
  pinMode(PIN_DIP2, INPUT);
  pinMode(PIN_DIP3, INPUT);
  pinMode(PIN_DIP4, INPUT);
  preDip1State = preDip2State = preDip3State = preDip4State = 1;
}

int DipSW::getDipState()
{
    dipNum = 0;
    if(!digitalRead(PIN_DIP1)) dipNum |= 0x01;
    if(!digitalRead(PIN_DIP2)) dipNum |= 0x02;
    if(!digitalRead(PIN_DIP3)) dipNum |= 0x04;
    if(!digitalRead(PIN_DIP4)) dipNum |= 0x08;

    return dipNum;
}

bool DipSW::ChangeToOn(byte pin)
{
  int dipState = digitalRead(pin);
  
  switch (pin)
  {
  case PIN_DIP1:
    return (preDip1State > dipState);
    preDip1State = dipState;
    break;
  case PIN_DIP2:
    return (preDip2State > dipState);
    preDip2State = dipState;
    break;
  case PIN_DIP3:
    return (preDip3State > dipState);
    preDip3State = dipState;
    break;
  case PIN_DIP4:
    return (preDip4State > dipState);
    preDip4State = dipState;
    break;
  default:
    return false;
    break;
  }
}

bool DipSW::ChangeToOff(byte pin)
{
  int dipState = digitalRead(pin);
  
  switch (pin)
  {
  case PIN_DIP1:
    return (preDip1State < dipState);
    preDip1State = dipState;
    break;
  case PIN_DIP2:
    return (preDip2State < dipState);
    preDip2State = dipState;
    break;
  case PIN_DIP3:
    return (preDip3State < dipState);
    preDip3State = dipState;
    break;
  case PIN_DIP4:
    return (preDip4State < dipState);
    preDip4State = dipState;
    break;
  default:
    return false;
    break;
  }
}

PIDsetting::PIDsetting(myLCDclass *_LCD, Encorder *_encorder, int _setting)
{
  LCD = _LCD;
  encorder = _encorder;
  setting = _setting;
  flag_lcd = true;
  init_done = false;
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

int Encorder::getEncCount()
{
    encA = digitalRead(PIN_ENC_A);
    encB = digitalRead(PIN_ENC_B);
    if((encA != pre_encB) && (encB == pre_encA)) count -= fluctuation;
    else if((encA == pre_encB) && (encB != pre_encA)) count += fluctuation;
    pre_encA = encA;
    pre_encB = encB;
    enc_count = (int)count;

    return enc_count;
}

void Encorder::setEncCount(int setnum){
    enc_count = setnum;
    count = (double)setnum;
}

void Encorder::setFluctuation(double setnum)
{
    fluctuation = setnum;
}

void PIDsetting::init(double _kp, double _ki, double _kd, char _moji[])
{
  kp = _kp;
  ki = _ki;
  kd = _kd;
  strcpy(moji,_moji);
  init_done = true;
}

void PIDsetting::task(bool flag_500ms,bool up, bool down, int setting_num)
{
  static int pid_setting_mode;
  static bool init_kp, init_ki, init_kd;
  static double printnum = 0;
  
  if((setting_num == setting) && init_done)
  {
    if(flag_lcd)
    { 
      LCD->clear_display();
      LCD->write_line(moji,LINE_1);
      flag_lcd = false;
      init_kp = true;
      pid_setting_mode = 1;
    }

    if(down)   pid_setting_mode++;
    else if(up) pid_setting_mode--;
    if(pid_setting_mode == 0) pid_setting_mode = 3;
    else if(pid_setting_mode == 4) pid_setting_mode = 1;

    switch (pid_setting_mode)
    {
    case 1:
      init_ki = true;
      init_kd = true;
      if(init_kp)
      { 
        LCD->write_str("Kp ",LINE_3,1); //3コマ使用
        encorder->setEncCount((int)(10.0 * kp));
        init_kp = false;
      }
      kp = printnum = 0.1*(double)encorder->getEncCount();

      break;
    
    case 2:
      init_kp = true;
      init_kd = true;
      if(init_ki)
      {
        LCD->write_str("Ki ",LINE_3,1); //3コマ使用
        encorder->setEncCount((int)(10.0 * ki));
        init_ki = false;
      }
      ki = printnum = 0.1*(double)encorder->getEncCount();

      break;
    
    case 3:
      init_kp = true;
      init_ki = true;
      if(init_kd)
      {
        LCD->write_str("Kd ",LINE_3,1); //3コマ使用
        encorder->setEncCount((int)(10.0 * kd));
        init_kd = false;
      }
      kd = printnum = 0.1*(double)encorder->getEncCount();

      break;

    default:
      break;
    }

    if(flag_500ms)
    {
      LCD->write_str("          ", LINE_3,4);
      LCD->write_double(printnum, LINE_3,4);
    }
  }
  else
  {
    flag_lcd = true;
  }
}

double PIDsetting::getKp(void)
{
  return kp;
}
double PIDsetting::getKi(void)
{
  return ki;
}
double PIDsetting::getKd(void)
{
  return kd;
}