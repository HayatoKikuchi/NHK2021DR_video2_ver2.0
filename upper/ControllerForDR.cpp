#include "ControllerForDR.h"

Controller::Controller(HardwareSerial *_Ser)
{
    Ser = _Ser;

    ButtonState = preButtonState = 0;
    LJoyX = 127;
    LJoyY = 127;
    RJoyX = 127;
    RJoyY = 127;
}

void Controller::begin(int baudrate)
{
    Ser->begin(baudrate);
}

bool Controller::update()
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
  return false;
}
 
bool Controller::readButton(unsigned int button,int status)
{
  int8_t num = 0;
  if(getButtonState() & button)  num += 2;
  if(getpreButtonState() & button) num -= 1;
  if(num == status) return true;
  else return false;
}

unsigned int Controller::getButtonState() const
{
    return ButtonState;
}
unsigned int Controller::getpreButtonState() const
{
    return preButtonState;
}

/**コントローラの入力に変化があったらtrueを返す**/
bool Controller::getButtonChanged() const
{
    return buttonChanged;
}

unsigned int Controller::readJoy(int joy)
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