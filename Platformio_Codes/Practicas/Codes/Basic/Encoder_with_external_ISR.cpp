#include <Arduino.h>

int A_chanel = 18;
int B_chanel = 19;
int button = 13;
volatile int count = 0;
int capture;

void IRAM_ATTR A_chanel_edge_chage(){
    
    if (digitalRead(A_chanel) == digitalRead(B_chanel)) {
        count++;
    }
    else{
        count--;
    }
}

void IRAM_ATTR B_chanel_edge_chage(){
    
    if (digitalRead(A_chanel) == digitalRead(B_chanel)) {
        count--;
    }
    else{
        count++;
    }
}

void IRAM_ATTR count_reset(){
    count = 0;
}


void setup(){
    Serial.begin(115200);

    pinMode(A_chanel,INPUT_PULLUP);
    pinMode(B_chanel,INPUT_PULLUP);
    pinMode(button,INPUT_PULLDOWN);


    attachInterrupt(digitalPinToInterrupt(A_chanel),A_chanel_edge_chage,CHANGE);
    attachInterrupt(digitalPinToInterrupt(B_chanel),B_chanel_edge_chage,CHANGE);
    attachInterrupt(digitalPinToInterrupt(13),count_reset,RISING);
}

void loop (){
    capture = count;
    Serial.println(capture);
}