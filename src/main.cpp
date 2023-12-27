#include <Arduino.h>

const int frequency = 50; // Hz
const int dutyCycle = 100;   // %

void setup()
{
    pinMode(26, OUTPUT);
}

void loop()
{
    digitalWrite(26, HIGH);
    delay((1000 / frequency) * dutyCycle/100);
    digitalWrite(26, LOW);
    delay((1000 / frequency) * (100 - dutyCycle)/100);
}