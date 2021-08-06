#include <Arduino.h>
#include <MsTimer2.h>

#include "Controller.h"
#include "define.h"
#include "Filter.h"

Controller Con;

Filter Cvel(INT_TIME);

bool flag_10ms = false, flag_100ms = false;
bool button_up = false, button_down = false, button_right = false, button_left = false; 

void ticker()
{  
    // RGB LED を良い感じに光らせるための処理
    static int count = 0;
    count += 2; // ここで光る周期を変えられる(はず)

    if(count < 255){
        analogWrite(PIN_LED_RED, count);
        analogWrite(PIN_LED_BLUE, 255 - count);
    }else if(count < 255 * 2){
        analogWrite(PIN_LED_GREEN, count - 255);
        analogWrite(PIN_LED_RED, 255 * 2 - count);
    }else if(count < 255 * 3){
        analogWrite(PIN_LED_BLUE, count - 255 * 2);
        analogWrite(PIN_LED_GREEN, 255 * 3 - count);
    }else{
        count = 0;
    }
    flag_10ms = true;
    static int count_100ms = 0;
    if(count_100ms == 10)
    {
        flag_100ms = true;
        count_100ms = 0;
    }
    count_100ms++;
}

void setup()
{
    delay(500);

    SERIAL_PC.begin(115200);
    SERIAL_CON.begin(115200);
    
    pinMode(PIN_SW, INPUT); // オンボードのスイッチ
    pinMode(PIN_XBEERESET, OUTPUT); // XBeeのリセット
    digitalWrite(PIN_XBEERESET, 0);
    delay(10);
    digitalWrite(PIN_XBEERESET,1);
    delay(10);

    analogWrite(PIN_LED_RED, 0); // 消しちゃダメ，ぜったい →　LPMSのために
    analogWrite(PIN_LED_BLUE, 0);
    analogWrite(PIN_LED_GREEN, 0);

    pinMode(PIN_LED_USER, OUTPUT);
    digitalWrite(PIN_LED_USER, LOW);

    Cvel.setSecondOrderPara(22.0, 1.0, 1.0);
    
    delay(100);

    MsTimer2::set(10, ticker);
    MsTimer2::start();
}

void loop()
{
    if(flag_10ms)
    {
        digitalWrite(PIN_LED_USER, Con.update());
        //Con.statePrint();

        static double rawC = 1.0;

        if(Con.readButton(BUTTON_L1) == PUSHE) rawC = 0.3;
        else if(Con.readButton(BUTTON_R1) == PUSHE) rawC = 2.0;
        else rawC = 1.0;
        
        double Cv = Cvel.SecondOrderLag(rawC);

        SERIAL_PC.print(rawC);
        SERIAL_PC.print("\t");
        SERIAL_PC.print(Cv);
        SERIAL_PC.print("\n");

        flag_10ms = false;
    }

    if(flag_100ms)
    {

        flag_100ms = false;
    }
}
