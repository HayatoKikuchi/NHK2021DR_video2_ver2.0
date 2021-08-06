#include <Arduino.h>
#include <MsTimer2.h>

#include "Button.h"
#include "define.h"
#include "master.h"

Button sw1(PIN_SW1);
Button sw2(PIN_SW2);
Button swUser(PIN_SW);

Master master(&SERIAL_MASTER);

unsigned int masterCMD;
double refOmega, refAngle;
bool flag_10ms = false, flag_100ms = false;

uint8_t number[3] = {0, 0, 0};

// LEDをチカチカさせるための関数
void LEDblink(byte pin, int times, int interval)
{
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

void RGB_led(int period)
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

void ticker()
{
    flag_10ms = true;
    RGB_led(2);

    static int count_100ms = 0;
    count_100ms++;
    if(count_100ms == 10)
    {
        flag_100ms = true;
        count_100ms = 1;
    }
}

void setup()
{
    delay(1000);
    
    SERIAL_PC.begin(115200);
    SERIAL_MASTER.begin(115200);

    pinMode(PIN_LED_USER, OUTPUT);
    digitalWrite(PIN_LED_USER, LOW);

    LEDblink(PIN_LED_RED, 1 , 100);
    LEDblink(PIN_LED_GREEN, 1 , 100);
    LEDblink(PIN_LED_BLUE, 1 , 100);

    analogWrite(PIN_LED_RED, 255);

    bool ready_to_start = false;

    while (!ready_to_start)
    {
        master.sendMasterCmd();
        master.updateMasterCmd(&masterCMD, &refAngle, &refOmega);

        if(masterCMD & UPPER_ON)
        {
            analogWrite(PIN_LED_RED, 0);
            analogWrite(PIN_LED_BLUE, 255);
        }

        if(masterCMD & UPPER_IS_OK) ready_to_start = true;
        if(swUser.button_fall()) ready_to_start = true;
        
        delay(10);
    }

    LEDblink(PIN_LED_GREEN, 3, 100);

    MsTimer2::set(10, ticker);
    MsTimer2::start();
}

void loop()
{
    if(flag_10ms)
    {
        master.sendMasterCmd();
        master.updateMasterCmd(&masterCMD, &refAngle, &refOmega);
/*
        SERIAL_MASTER.write(22);
        SERIAL_MASTER.write(44);
        SERIAL_MASTER.write(66);
        SERIAL_MASTER.write(END_BYTE);
        // SERIAL_MASTER.println(2);

        while (SERIAL_MASTER.available())
        {
          digitalWrite(PIN_LED_USER, !digitalRead(PIN_LED_USER));

            static int count_loop = 0;
            uint8_t num = SERIAL_MASTER.read();
            static uint8_t buf_num[3];

            if(num == END_BYTE)
            {
              if(count_loop == 3)
              {
                number[0] = buf_num[0];
                number[1] = buf_num[1];
                number[2] = buf_num[2];
              }
              
              count_loop = 0;
            }
            else
            {
                buf_num[count_loop] = num;
                count_loop++;
            }
        }
*/
        flag_10ms = false;
    }

    if(flag_100ms)
    {
        SERIAL_PC.print(masterCMD, BIN);
        SERIAL_PC.print("\t");
        SERIAL_PC.print(refAngle);
        SERIAL_PC.print("\t");
        SERIAL_PC.print(refOmega);
        SERIAL_PC.print("\t");
        SERIAL_PC.println(master.master_cmd, BIN);

        flag_100ms = false;
    }
}