#include "operator.h"

Operator::Operator(HardwareSerial *_upper)
{
  upper = _upper;
  Operator::setup();
  Operator::allOutputLow();
  Operator::init_upper_cmd();
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

void Operator::init_upper_cmd()
{
  Operator::upper_cmd = Operator::pre_upper_cmd = MASTER_ON;
}

void Operator::add_upper_cmd(unsigned int addNum)
{
  Operator::upper_cmd |= addNum;
}

void Operator::sub_upper_cmd(unsigned int subNum)
{
  if(Operator::upper_cmd & subNum) Operator::upper_cmd -= subNum;
}

void Operator::sendUpperCmd(double refAngle, double refOmega)
{
  pre_upper_cmd = Operator::upper_cmd;

  if(refAngle < 0) Operator::add_upper_cmd(TABLE_POSI_NEGATIVE); //変数の正負を情報を格納
  else Operator::sub_upper_cmd(TABLE_POSI_NEGATIVE);
  if(refOmega < 0) Operator::add_upper_cmd(TABLE_OMEGA_NEGATIVE); //変数の正負の情報を格納
  else Operator::sub_upper_cmd(TABLE_OMEGA_NEGATIVE);

  double sendAngle, sendOmega;
  sendAngle = fabs(refAngle*10.0); // 1/10の位まで
  sendOmega = fabs(refOmega*10.0); // 1/10の位まで

  sendData[0] = Operator::upper_cmd;
  sendData[1] = mapping(sendAngle, 0, 600, 0, 255); //0~600の値を0~255の値に再マップ
  sendData[2] = mapping(sendOmega, 0, 600, 0, 255); //0~600の値を0~255の値に再マップ
  sendData[3] = 0; //reserved
  uint8_t checksum = 1;
  for (int i = 0; i < (SENDDATANUM - 2); i++) checksum ^= sendData[i];
  sendData[4] = checksum;
  sendData[5] = END_BYTE;

  //if(Operator::pre_upper_cmd != Operator::upper_cmd) //コマンドが変更しときにだけ送信
  //{
    // for (int i = 0; i < SENDDATANUM; i++) upper->write(sendData[i]);
    for (int i = 0; i < SENDDATANUM; i++) upper->write(sendData[i]);
}

void Operator::updateUpperCmd(unsigned int *cmd)
{

  while (upper->available())
  {

    uint8_t num = upper->read();
    static int loop_num = 0;
    if(num == END_BYTE)
    {
      if(loop_num == (RECIVEDATANUM - 1))
      {
        uint8_t checksum = 1;
        for (int i = 0; i < (RECIVEDATANUM - 2); i++) checksum ^= recv_num[i];

        if(recv_num[RECIVEDATANUM - 2] == checksum)
        {
          for (int i = 0; i < RECIVEDATANUM; i++)
          {
            reciveData[i] = recv_num[i];
          }
          
          *cmd = reciveData[0];
        }

      }

      loop_num = 0;
    }
    else
    {
      recv_num[loop_num] = num;
      loop_num++;
    }
  }
  
}


int Operator::mapping(double value, double fromLow, double fromHigh, double toLow, double toHigh)
{
    return (int)(value / (fromHigh - fromLow) * (toHigh - toLow));
}