#include <Arduino.h>
#include <ESP32Encoder.h>

// ── Encoder ───────────────────────────────────────────────
#define ENC_A  18
#define ENC_B  19
ESP32Encoder encoder;

// ── Driver BTS7960 ────────────────────────────────────────
#define PIN_RPWM   15
#define PIN_LPWM    2
#define PIN_R_EN    4
#define PIN_L_EN    5
#define PIN_BOTON  13

// ── PWM ───────────────────────────────────────────────────
#define PWM_FREQ       20000
#define PWM_RESOLUTION 11        
#define CH_RPWM        0
#define CH_LPWM        1
#define PWM_MIN        365
#define PWM_MAX        2047  



//
hw_timer_t   *timer          = NULL;
portMUX_TYPE  mux            = portMUX_INITIALIZER_UNLOCKED;
volatile bool control_activo = false;
uint32_t      ultimo_serial  = 0;
int ultimo_buton = 0;
int now = 0;
int pulsos = 0;
float filtered_velocity = 0.0f;
volatile float dbg_velocity = 0; 
int last_count = 0;
float ALPHA = 0.3;
float DT = 0.02;
float dbg_raw_velocity = 0.0f;
int now_2 = 0;
volatile int time_max_vel = 0; 
volatile int dbg_time_max_vel = 0;

void IRAM_ATTR velocity_reader() {


    portENTER_CRITICAL_ISR(&mux);
    int64_t c = encoder.getCount();
    portEXIT_CRITICAL_ISR(&mux);

    //velocidad pulso/s
    float raw_velocity = (float)(c - last_count)/DT;
    last_count = c;

    if (raw_velocity == 4150)
        now_2 =  millis();
        time_max_vel = now_2 - ultimo_buton;

    //filtro EMA
    filtered_velocity = ALPHA * raw_velocity + (1-ALPHA) * filtered_velocity; 


    dbg_velocity = filtered_velocity;
    dbg_raw_velocity = raw_velocity;
    dbg_time_max_vel = time_max_vel;
}
// Botón arranca el control
void IRAM_ATTR boton_isr() {

    now = millis();
    if (now - ultimo_buton >= 250){
        ultimo_buton = now;
        control_activo = !control_activo; }
}


// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    uint32_t alarm_ticks = DT * 1000000; 

    // Driver
    pinMode(PIN_R_EN, OUTPUT); digitalWrite(PIN_R_EN, HIGH);
    pinMode(PIN_L_EN, OUTPUT); digitalWrite(PIN_L_EN, HIGH);

    // PWM
    ledcSetup(CH_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_LPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_RPWM, CH_RPWM);
    ledcAttachPin(PIN_LPWM, CH_LPWM);
    ledcWrite(CH_RPWM, 0);
    ledcWrite(CH_LPWM, 0);

    // Encoder
    encoder.attachFullQuad(ENC_A, ENC_B);
    encoder.setCount(0);

    // Botón
    pinMode(PIN_BOTON, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_BOTON), boton_isr, RISING);
    
    // Timer: 80MHz / prescaler 80 = 1 tick/µs → 1000 ticks = 1ms
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &velocity_reader, true);
    timerAlarmWrite(timer, alarm_ticks, true);
    timerAlarmEnable(timer);
}


void loop() {
    

    if (control_activo) {
        ledcWrite(CH_LPWM, 2047);    
    }

    else{
        encoder.clearCount();
        ledcWrite(CH_LPWM,0);
        time_max_vel = 0;
        dbg_time_max_vel = 0;
    }

    // Serial
    uint32_t ahora = millis();
    if (ahora - ultimo_serial >= 20) {
        ultimo_serial = ahora;
        pulsos = encoder.getCount();
        //Serial.printf(">pulsos: %d\n",pulsos); 
        Serial.printf(">velocidad: %.2f\n",dbg_velocity);
        Serial.printf(">raw_velocidad: %.2f\n",dbg_raw_velocity);
        Serial.printf(">time_max_vel: %.d\n",time_max_vel);
        
    }
}