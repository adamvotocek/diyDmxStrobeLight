#include <Arduino.h>

//led pwm
const int pwmFrequency = 5000; // Hz
const int pwmChannel = 0;
const int ledPin = 13;
const int resolution = 8; //max resolution of the DMX512 protocol

int dutyCycle = 1;

void setup()
{
    ledcSetup(pwmChannel, pwmFrequency, resolution);
    ledcAttachPin(ledPin, pwmChannel);
    //pinMode(ledPin, OUTPUT);
}

void loop()
{
    ledcWrite(pwmChannel, dutyCycle);
    /*dutyCycle++;
    if (dutyCycle > 255)
    {
        dutyCycle = 0;
    }
    delay(5);*/
}