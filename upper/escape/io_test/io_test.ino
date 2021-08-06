#include "Arduino.h"
#include "define.h"

void setup()
{
    delay(1000);

    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
    pinMode(PIN_LED_3, OUTPUT);
    pinMode(PIN_LED_4, OUTPUT);

    pinMode(PIN_DIP1, INPUT);
    pinMode(PIN_DIP2, INPUT);
    pinMode(PIN_DIP3, INPUT);
    pinMode(PIN_DIP4, INPUT);

    pinMode(PIN_LED_USER,OUTPUT);
    digitalWrite(PIN_LED_USER,HIGH);

    delay(500);
}

void loop()
{
    digitalWrite(PIN_LED_1, !digitalRead(PIN_DIP1));
    digitalWrite(PIN_LED_2, !digitalRead(PIN_DIP2));
    digitalWrite(PIN_LED_3, !digitalRead(PIN_DIP3));
    digitalWrite(PIN_LED_4, !digitalRead(PIN_DIP4));

    delay(10);
}