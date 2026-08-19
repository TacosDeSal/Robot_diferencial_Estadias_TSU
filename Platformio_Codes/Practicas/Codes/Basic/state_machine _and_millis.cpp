#include <Arduino.h>

const int led_1 = 27;
const int led_2 = 14;
const int button = 13;
bool estado = false;
unsigned long reference_time = 0; 
const long duration = 3000;

void setup(){
    Serial.begin(115200);
    pinMode(led_1,OUTPUT);
    pinMode(led_2,OUTPUT);
    pinMode(button,INPUT_PULLDOWN);
    
}

void loop(){
    if (!estado){
        Serial.println("Led apagado");
        digitalWrite(led_1,LOW);
        
        if (digitalRead(button) == HIGH){
            digitalWrite(led_1,HIGH);
            reference_time = millis();
            estado = true;
        }
    }
    else{

        Serial.println("Led escendido");

        if ( millis() - reference_time >= duration){
            estado = false;
        }

    }
}