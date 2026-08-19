#include <Arduino.h>

const int  led_pin = 4;
const int frecuency = 5000;
const int resolution = 10;
const int chanel = 0;

void setup() {

    Serial.begin(115200);
    ledcSetup(chanel, frecuency, resolution);
    ledcAttachPin(led_pin, chanel);
}

void loop() {

    for (int duty_cycle = 0; duty_cycle <= 1023; duty_cycle++)
    {
        ledcWrite(chanel, duty_cycle);
        Serial.printf("increasing: ", duty_cycle);
        delayMicroseconds(4800);
    }
    
    for ( int duty_cycle = 1023; duty_cycle >= 0; duty_cycle--){

        ledcWrite(chanel, duty_cycle);
        Serial.printf("decresing: ", duty_cycle);
        delayMicroseconds(4800);
    
    }
}
