#include <Arduino.h>
#include <ESP32Encoder.h>             //Library

ESP32Encoder encoder_1; 
ESP32Encoder encoder_2;      //the encoder name
//int button = 13;
int last_now = 0;

//void IRAM_ATTR clear_count_encoder(){
//    encoder_sioq.clearCount();
//}

void setup(){
    Serial.begin(921600);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;    // enable weak pull up resistor
    encoder_1.attachFullQuad(4,5);                              // define the GPIO end the resulution for the cuadrature lecture
    encoder_1.clearCount();

    encoder_2.attachFullQuad(19,21);                              // define the GPIO end the resulution for the cuadrature lecture
    encoder_2.clearCount();

    //pinMode(button,INPUT_PULLDOWN);
    //attachInterrupt(digitalPinToInterrupt(button),clear_count_encoder,RISING);

}

void loop(){
    
    int now = millis();
    if (now - last_now > 20){
    Serial.printf("Enc1: %lld | Enc2: %lld\n", encoder_1.getCount(), encoder_2.getCount());
    last_now = now;
    }
}
    

