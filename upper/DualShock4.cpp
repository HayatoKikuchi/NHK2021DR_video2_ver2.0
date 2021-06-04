#include "DualShock4.h"

DualSchok4::DualSchok4(HardwareSerial *_Ser)
{
    Ser = _Ser;

    ButtonState = preButtonState = 0;
    LJoyX = 127;
    LJoyY = 127;
    RJoyX = 127;
    RJoyY = 127;
}

void DualSchok4::begin(int baudrate)
{
    Ser->begin(baudrate);
}

bool DualSchok4::update()
{
  unsigned int checksum;
  preButtonState = ButtonState;
  char c;
  while (Ser->available())
  {
    c = Ser->read();
    if (c == '\n')
    {
      if (recv_num == 10)
      {
        checksum = 0;

        for (int i = 0; i < 9; i++)
        {
          raww_recv_msgs[i] = recv_msgs[i];
          recv_msgs[i] = recv_msgs[i] - 0x20;
          checksum += (unsigned int)recv_msgs[i];
        }

        if ((checksum & 0x3F) == (recv_msgs[9] - 0x20))
        {
          ButtonState = 0;

          ButtonState |= recv_msgs[0] & 0x3F;
          ButtonState |= (recv_msgs[1] & 0x3F) << 6;
          ButtonState |= (recv_msgs[2] & 0x0F) << 12;

          LJoyX = recv_msgs[3];
          LJoyX |= (recv_msgs[4] & 0x03) << 6;

          LJoyY = (recv_msgs[4] & 0x3C) >> 2;
          LJoyY |= (recv_msgs[5] & 0x0F) << 4;

          RJoyX = (recv_msgs[5] & 0x30) >> 4;
          RJoyX |= (recv_msgs[6] & 0x3F) << 2;

          RJoyY = recv_msgs[7];
          RJoyY |= (recv_msgs[8] & 0x03) << 6;
          
          return true;
        }
      }
      recv_num = 0;
    }
    else
    {
      recv_msgs[recv_num++] = c;
    }
  }
  
  if(preButtonState != ButtonState) buttonChanged = true;
  else buttonChanged = false;

  return false;
}

bool DualSchok4::readButton(unsigned int button,int state)
{
  int8_t num = 0;
  if(getButtonState() & button)  num += 2;
  if(getpreButtonState() & button) num -= 1;
  if(num == state) return true;
  else return false;
}

unsigned int DualSchok4::getButtonState() const
{
    return ButtonState;
}
unsigned int DualSchok4::getpreButtonState() const
{
    return preButtonState;
}

/**コントローラの入力に変化があったらtrueを返す**/
bool DualSchok4::getButtonChanged() const
{
    return buttonChanged;
}

unsigned int DualSchok4::readJoy(int joy)
{
  unsigned int out;
  switch (joy)
  {
  case LX: out = LJoyX; break;
  case LY: out = LJoyY; break;
  case RX: out = RJoyX; break;
  case RY: out = RJoyY; break;
  default: out = 127;   break;
  }
  
  return out;
}