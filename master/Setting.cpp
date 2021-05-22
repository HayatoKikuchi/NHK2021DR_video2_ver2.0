#include "Setting.h"
#include "define.h"

Encorder::Encorder(double _fluctuation){
    fluctuation = _fluctuation;

    pinMode(PIN_ENC_A,INPUT);
    pinMode(PIN_ENC_B,INPUT);
    
    pre_encA = digitalRead(PIN_ENC_A);
    pre_encB = digitalRead(PIN_ENC_B);
    count = 0.0;
    enc_count = 0;
}

DipSW::DipSW(){
    pinMode(PIN_DIP1,INPUT);
    pinMode(PIN_DIP2,INPUT);
    pinMode(PIN_DIP3,INPUT);
    pinMode(PIN_DIP4,INPUT);
}

int DipSW::getDipState(){
    dipNum = 0;
    if(!digitalRead(PIN_DIP1)) dipNum |= 0x01;
    if(!digitalRead(PIN_DIP2)) dipNum |= 0x02;
    if(!digitalRead(PIN_DIP3)) dipNum |= 0x04;
    if(!digitalRead(PIN_DIP4)) dipNum |= 0x08;

    return dipNum;
}

PIDsetting::PIDsetting(PID *_pid, myLCDclass *_LCD, Encorder *_encorder, int _setting)
{
  pid = _pid;
  LCD = _LCD;
  encorder = _encorder;
  setting = _setting;
  flag_lcd = true;
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


void PIDsetting::init_variable()
{
  flag_lcd = true;
}

void PIDsetting::task(bool flag_500ms,bool up, bool down,char moji[], int setting_num)
{
  static int pid_setting_mode;
  static bool init_kp, init_ki, init_kd;
  double printnum;
  Gain K;
  Gain setK;
  
  if(setting_num == setting)
  {
    if(flag_lcd)
    { 
      LCD->clear_display();
      LCD->write_str(moji,LINE_1,1);
      flag_lcd = false;
      init_kp = true;
      K = pid->getRara();
      setK = K;
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
        encorder->setEncCount((int)(10.0 * K.p));
        init_kp = false;
      }
      setK.p = printnum = 0.1*(double)encorder->getEncCount();

      break;
    
    case 2:
      init_kp = true;
      init_kd = true;
      if(init_ki)
      {
        LCD->write_str("Ki ",LINE_3,1); //3コマ使用
        encorder->setEncCount((int)(10.0 * K.i));
        init_ki = false;
      }
      setK.i = printnum = 0.1*(double)encorder->getEncCount();

      break;
    
    case 3:
      init_kp = true;
      init_ki = true;
      if(init_kd)
      {
        LCD->write_str("Kd ",LINE_3,1); //3コマ使用
        encorder->setEncCount((int)(10.0 * K.d));
        init_kd = false;
      }
      setK.d = printnum = 0.1*(double)encorder->getEncCount();
      break;

    default:
      break;
    }

    pid->setPara(setK.p,setK.i,setK.d);

    if(flag_500ms)
    {
      LCD->write_str("          ", LINE_3,4);
      LCD->write_double(printnum, LINE_3,4);
    }
  }
  else
  {
    PIDsetting::init_variable();
  }
}