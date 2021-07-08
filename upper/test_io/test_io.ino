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

    pinMode(PIN_FET_1, OUTPUT);
    pinMode(PIN_FET_2, OUTPUT);
    pinMode(PIN_FET_3, OUTPUT);
    pinMode(PIN_FET_4, OUTPUT);
    
    pinMode(26, OUTPUT);
    pinMode(27, OUTPUT);
    pinMode(28, OUTPUT);
    pinMode(29, OUTPUT);
    pinMode(20, OUTPUT);
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A2, OUTPUT);

    pinMode(36, OUTPUT);
    pinMode(37, OUTPUT);
    pinMode(38, OUTPUT);
    pinMode(39, OUTPUT);
    pinMode(40, OUTPUT);
    pinMode(41, OUTPUT);
    pinMode(42, OUTPUT);
    pinMode(43, OUTPUT);

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
    
    // digitalWrite(PIN_FET_1, LOW);
    // digitalWrite(PIN_FET_2, LOW);
    // digitalWrite(PIN_FET_3, HIGH);
    // digitalWrite(PIN_FET_4, HIGH);
    
    digitalWrite(20, HIGH);
    digitalWrite(A0, HIGH);
    digitalWrite(A1, HIGH);
    digitalWrite(A2, HIGH);
    digitalWrite(26, HIGH);
    digitalWrite(27, HIGH);
    digitalWrite(28, HIGH);
    digitalWrite(29, HIGH);
    
    digitalWrite(36, HIGH);
    digitalWrite(37, HIGH);
    digitalWrite(38, HIGH);
    digitalWrite(39, HIGH);
    digitalWrite(40, HIGH);
    digitalWrite(41, HIGH);
    digitalWrite(42, HIGH);
    digitalWrite(43, HIGH);

    delay(10);
}