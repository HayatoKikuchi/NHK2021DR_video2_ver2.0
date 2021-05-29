#include <Arduino.h>
#include <MsTimer2.h>
#include "define.h"
#include "DIPclass.h"
#include "ControllerForDR.h"
#include "Button.h"

DipSW dip;
Controller Con(&SERIAL_CON);
Button userSW(PIN_SW);

bool flag_10ms = false;
bool flag_expand = false;
int led_emitting = 0;
int dipState = 0;

/* マイコンボード上の4つのLEDを光らせる関数 */
void UserLED(int emitting_num)
{
    digitalWrite(PIN_LED0, emitting_num & 0x01);
    digitalWrite(PIN_LED1, emitting_num & 0x02);
    digitalWrite(PIN_LED2, emitting_num & 0x04);
    digitalWrite(PIN_LED3, emitting_num & 0x08);
}

/* 基板上の4つのLEDを光らせる関数 */
void BoardLED(int emitting_num)
{
    digitalWrite(PIN_LED_1, emitting_num & 0x01);
    digitalWrite(PIN_LED_2, emitting_num & 0x02);
    digitalWrite(PIN_LED_3, emitting_num & 0x04);
    digitalWrite(PIN_LED_4, emitting_num & 0x08);
}

void timer_warikomi()
{
    flag_10ms = true;

    dipState = dip.getDipState();

    /* 展開の処理 */
    if(flag_expand)
    {
        static int expand_seconds = 0;
        if(expand_seconds++ == 100)
        {
            expand_seconds = 0;
            flag_expand = false;
        }
    }

    /* ボード上のLEDを奇麗に光らせるための処理 */
    static int led_time_count = 0;
    if(led_time_count++ == 100)
    {
        led_time_count = 0;
        if(led_emitting++ == 16) led_emitting = 0;
    }
}

void setup()
{
    delay(1000);

    pinMode(PIN_LED0, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);

    pinMode(PIN_EXPAND, OUTPUT);

    UserLED(LED_OLL_HIGH);
    
    delay(1000);

    SERIAL_PC.begin(115200);
    Con.begin(115200);

    bool ready_to_start = false;
    while (!ready_to_start)
    {
        Con.update();
        if(Con.readButton(BUTTON_PS,PUSHED) || userSW.button_fall())
        {
            ready_to_start = true;
        }

        static bool emitting = false;
        static int led_counts = 0;
        if(led_counts++ == 100)
        {
            emitting = !emitting;
            led_counts = 0;
        }

        UserLED(emitting);
        
        delay(INT_TIME_MS);
    }

    UserLED(LED_OLL_HIGH);

    delay(1000);
    
    MsTimer2::set(INT_TIME_MS, timer_warikomi);
    MsTimer2::start();
}

void loop()
{

    if(flag_10ms)
    {
        Con.update();

        if(Con.readButton(BUTTON_SANKAKU, PUSHED)) flag_expand = true;
        digitalWrite(PIN_EXPAND, flag_expand);


        UserLED(led_emitting);
        BoardLED(dipState);

        flag_10ms = false;
    }

}
