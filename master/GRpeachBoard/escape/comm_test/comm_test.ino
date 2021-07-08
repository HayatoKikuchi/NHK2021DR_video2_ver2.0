#include <Arduino.h>
#include <MsTimer2.h>

#include "Button.h"
#include "define.h"
#include "operator.h"

Button swBlack(PIN_SW_BLACK);
Button swRed(PIN_SW_RED);
Button swUser(PIN_SW);

Operator DR(&SERIAL_UPPER);

unsigned int upperCMD;
bool flag_10ms = false, flag_100ms = false;

int number[3] = {0, 0, 0};

void ticker()
{
    flag_10ms = true;
    DR.RGB_led(2);

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
    SERIAL_UPPER.begin(115200);
    DR.setup();

    DR.LEDblink(PIN_LED_RED, 1 , 100);
    DR.LEDblink(PIN_LED_GREEN, 1 , 100);
    DR.LEDblink(PIN_LED_BLUE, 1 , 100);

    analogWrite(PIN_LED_RED, 255);

    bool ready_to_start = false;

    while (!ready_to_start)
    {
        // DR.sendUpperCmd();
        // DR.updateUpperCmd(&upperCMD);

        if(upperCMD & UPPER_ON)
        {
            analogWrite(PIN_LED_RED, 0);
            analogWrite(PIN_LED_BLUE, 255);
        }

        if(upperCMD & UPPER_IS_OK) ready_to_start = true;
        if(swUser.button_fall()) ready_to_start = true;
        
        delay(10);
    }

    DR.LEDblink(PIN_LED_GREEN, 3, 100);

    MsTimer2::set(10, ticker);
    MsTimer2::start();
}

void loop()
{
    if(flag_10ms)
    {
        // DR.updateUpperCmd(&upperCMD);
        // DR.sendUpperCmd();

        SERIAL_UPPER.write(11);
        SERIAL_UPPER.write(33);
        SERIAL_UPPER.write(55);
        SERIAL_UPPER.write(END_BYTE);
    }

    if(flag_100ms)
    {
        // SERIAL_PC.print(upperCMD, BIN);
        // SERIAL_PC.print("\t");
        // SERIAL_PC.println(DR.upper_cmd, BIN);
        while (SERIAL_UPPER.available())
        {
            static int count_loop = 0;
            int num = SERIAL_UPPER.read();
            static int buf_num[3];

            if(num == END_BYTE)
            {
                number[0] = buf_num[0];
                number[1] = buf_num[1];
                number[2] = buf_num[2];

                count_loop = 0;
            }
            else
            {
                buf_num[count_loop] = num;
                count_loop++;
            }
        }
        
        SERIAL_PC.print(number[0]);
        SERIAL_PC.print(number[1]);
        SERIAL_PC.println(number[2]);
    }
}