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
int led_emitting = 0;

void timer_warikomi()
{
    flag_10ms = true;

    static int led_time_count = 0;
    if(led_time_count++ == 100)
    {
        led_emitting++;
        led_time_count = 0;
        if(led_emitting == 16) led_emitting = 0;
    }
}

void setup()
{
    delay(1000);

    pinMode(PIN_LED0, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);

    digitalWrite(PIN_LED0, HIGH);
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, HIGH);
    digitalWrite(PIN_LED3, HIGH);
    
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

        digitalWrite(PIN_LED0, emitting);
        digitalWrite(PIN_LED1, emitting);
        digitalWrite(PIN_LED2, emitting);
        digitalWrite(PIN_LED3, emitting);
        
        delay(INT_TIME_MS);
    }

    digitalWrite(PIN_LED0, HIGH);
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, HIGH);
    digitalWrite(PIN_LED3, HIGH);

    delay(1000);
    
    MsTimer2::set(INT_TIME_MS, timer_warikomi);
    MsTimer2::start();
}

void loop()
{
    if(flag_10ms)
    {
        flag_10ms = false;
    }

    digitalWrite(PIN_LED0, led_emitting & 0x01);
    digitalWrite(PIN_LED1, led_emitting & 0x02);
    digitalWrite(PIN_LED2, led_emitting & 0x04);
    digitalWrite(PIN_LED3, led_emitting & 0x08);
}
