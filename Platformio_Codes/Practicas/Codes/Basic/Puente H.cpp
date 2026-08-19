#include <Arduino.h>

const int pot = 12;
const int pwm_r = 2;
const int pwm_l = 15;
const int r_en = 5;
const int l_en = 4;
const int resolution = 11;
const int frecuency = 20000;
const int chanel_r = 0;
const int chanel_l = 1;

void setup() {

    Serial.begin(115200);
    analogReadResolution(12);
    pinMode(r_en,OUTPUT);
    pinMode(l_en, OUTPUT);
    ledcSetup(chanel_r, frecuency, resolution);
    ledcAttachPin(pwm_r, chanel_r);
    ledcSetup(chanel_l, frecuency, resolution);
    ledcAttachPin(pwm_l, chanel_l);
    digitalWrite(l_en, HIGH);
    digitalWrite(r_en, HIGH);

}

void loop(){

    int pot_val = analogRead(pot);

    if (pot_val <=  1947) {
        int duty_cicle = map(pot_val,0,1947,2047,0);
        int velocidad_irl = map(duty_cicle,362,2047,0,100);
        if(velocidad_irl <= 0){
            velocidad_irl = 0;   
        }
        ledcWrite(chanel_l, duty_cicle);
        Serial.printf("Pot:%4d | pwm:%4d | velocidad:%4d% | sentido: L\n",pot_val,duty_cicle,velocidad_irl);
        
    }

     else if (pot_val >=  2147) {
        int duty_cicle = map(pot_val,2147,4095,0,2047);
        int velocidad_irl = map(duty_cicle,362,2047,0,100);
        ledcWrite(chanel_r, duty_cicle);
        Serial.printf("Pot:%4d | pwm:%4d | velocidad:%4d% | sentido: R\n",pot_val,duty_cicle,velocidad_irl);
        
    }

    else {
        ledcWrite(chanel_l,0);
        ledcWrite(chanel_l, 0);
        Serial.printf("Gap: %d\n",pot_val);
        
    }

    delay(10);
}