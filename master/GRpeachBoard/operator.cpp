#include "operator.h"

Operator::Operator(HardwareSerial *_upper)
{
  upper = _upper;
  Operator::setup();
  Operator::allOutputLow();
}

void Operator::setup()
{   
  pinMode(PIN_SW, INPUT); // オンボードのスイッチ
  pinMode(PIN_LED_USER, OUTPUT);

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_ENC, OUTPUT);
}

// LEDをチカチカさせるための関数
void Operator::LEDblink(byte pin, int times, int interval)
{
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

void Operator::RGB_led(int period)
{
  // RGB LED を良い感じに光らせるための処理
  static int count = 0;
  count += period; // ここで光る周期を変えられる(はず)
  if(count < 255){
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }
  else if(count < 255 * 2){
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }
  else if(count < 255 * 3){
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }
  else{
    count = 0;
  }
}

void Operator::allOutputLow()
{
  digitalWrite(PIN_LED_1, LOW);
  digitalWrite(PIN_LED_2, LOW);
  digitalWrite(PIN_LED_3, LOW);
  digitalWrite(PIN_LED_4, LOW);
  digitalWrite(PIN_LED_ENC, LOW);
}

void Operator::sendUpperCmd()
{
  sendData[0] = 1; //ダミー
  sendData[1] = 2; //ダミー
  sendData[2] = 3; //ダミー
  sendData[3] = 4; //ダミー
  sendData[4] = 5; //ダミー
  sendData[5] = (sendData[0] ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);
  sendData[6] = END_BYTE;

  for (int i = 0; i < 7; i++)
  {
    upper->write(sendData[i]);
  }
  
}

void Operator::updateUpperCmd(uint8_t *status)
{

  while (upper->available())
  {
    uint8_t num = upper->read();
    static int loop_num = 0;
    if(num == END_BYTE)
    {
      if(recv_num[2] == (recv_num[0] ^ recv_num[1]))
      {
        for (int i = 0; i < 4; i++)
        {
          reciveData[i] = recv_num[i];
        }
      }

      *status = reciveData[0];

      loop_num = 0;
    }
    else
    {
      recv_num[loop_num] = num;
      loop_num++;
    }
  }
  
}