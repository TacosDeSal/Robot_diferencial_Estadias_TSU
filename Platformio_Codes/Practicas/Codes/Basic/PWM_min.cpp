#include <Arduino.h>

// ── IBT_2  Motor 1 ──────────────────────────────
const int PIN_RPWM = 15;
const int PIN_LPWM =  2;
const int PIN_R_EN =  4;
const int PIN_L_EN =  5;

// ── L298N  Motor 2 ──────────────────────────────
const int PIN_ENA  = 25;
const int PIN_IN1  = 33;
const int PIN_IN2  = 32;

// ── Potenciómetro ───────────────────────────────
const int PIN_POT  = 14;   // cualquier pin ADC; 34/35/36/39 son input-only

// ── LEDC ────────────────────────────────────────
const int FREQ      = 15000;
const int BITS      = 12;
const int CH_R      = 0;   // IBT_2 dirección →
const int CH_L      = 1;   // IBT_2 dirección ←
const int CH_ENA    = 2;   // L298N velocidad

void setup() {
    Serial.begin(115200);
    analogReadResolution(BITS);

    // IBT_2 enables
    pinMode(PIN_R_EN, OUTPUT);
    pinMode(PIN_L_EN, OUTPUT);
    digitalWrite(PIN_R_EN, HIGH);
    digitalWrite(PIN_L_EN, HIGH);

    // L298N dirección fija: → (IN1=H, IN2=L)
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);

    // LEDC
    ledcSetup(CH_R,   FREQ, BITS);
    ledcSetup(CH_L,   FREQ, BITS);
    ledcSetup(CH_ENA, FREQ, BITS);
    ledcAttachPin(PIN_RPWM, CH_R);
    ledcAttachPin(PIN_LPWM, CH_L);
    ledcAttachPin(PIN_ENA,  CH_ENA);
}

void loop() {
    int pot_val    = analogRead(PIN_POT);
    int duty_cycle = pot_val;               // 0..4095, escala directa

    // Motor 1 IBT_2: gira en un solo sentido (RPWM activo, LPWM=0)
    ledcWrite(CH_R, duty_cycle);
    ledcWrite(CH_L, 0);

    // Motor 2 L298N: velocidad por ENA
    ledcWrite(CH_ENA, duty_cycle);

    float vel_pct = duty_cycle * 100.0f / 4095.0f;
    Serial.printf("Pot:%4d | duty:%4d | vel:%.1f%% |\n",
                  pot_val, duty_cycle, vel_pct);

    delay(50);
}