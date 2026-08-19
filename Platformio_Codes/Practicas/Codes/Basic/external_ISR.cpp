#include <Arduino.h>          //In this practice im gonna use the ISR to interrup a led sequency and call a function to turn on all the leds.

int led_1 = 27;
int led_2 = 14;         
int led_3 = 12;  
int button = 13;

void IRAM_ATTR ISR_all_leds_on (){                //This function is what gonna be call when the interrup occurs. It can have any name.

    digitalWrite(led_1, HIGH);
    digitalWrite(led_2,HIGH);
    digitalWrite(led_3,HIGH);
    
}

void setup(){

    pinMode(led_1,OUTPUT);
    pinMode(led_2,OUTPUT);
    pinMode(led_3,OUTPUT);
    pinMode(button,INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(button),ISR_all_leds_on, RISING); // defining the GPIO as Interrept one, the function to call and the mode of activation
}

void loop(){
    digitalWrite(led_1,HIGH);
    digitalWrite(led_2,LOW);
    digitalWrite(led_3,LOW);
    delay(2000);
    digitalWrite(led_2,HIGH);
    digitalWrite(led_1,LOW);
    digitalWrite(led_3,LOW);
    delay(2000);
    digitalWrite(led_3,HIGH);
    digitalWrite(led_1,LOW);
    digitalWrite(led_2,LOW);
    delay(2000);
}