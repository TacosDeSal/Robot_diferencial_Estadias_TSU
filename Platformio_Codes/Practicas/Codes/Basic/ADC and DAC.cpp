#include <Arduino.h>

const int pin_pot = 35;
int ADC_read;
int led = 26;

void setup(){
    analogReadResolution(12);
    Serial.begin(115200);
}

void loop(){
    ADC_read = analogRead(pin_pot);
    int V_out = map(ADC_read,0,4095,0,255);
    dacWrite(led, V_out);

    Serial.printf("Lectura pot: %d", ADC_read);
    Serial.printf(", Salida en led: %d\n",V_out);

}