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

//-Trapezodial parameters─────────────────────────────────────
const float cruiser_vel = 1800.0f;
const float max_acelaration = 1000.0f;
const float distance = 10000.0f;
const float profile_DT = 0.05;         // periodo calculo perfil en s

//-Trapezodial variables─────────────────────────────────────
int A_position = 0;
volatile int now_count = 0;
volatile float profile_time = 0;
volatile float dinamic_setpoint = 0.0f;

//-trapezoidal times──────────────────────────────────────────
volatile float t_1 = 0.0f;
volatile float t_2 = 0.0f;
volatile float t_3 = 0.0f;
volatile bool triangle = false;
volatile float peak_vel = 0.0f;

// ── PID parameters ─────────────────────────────────────────
const float DT    = 0.02f;     // intervalo de control en segundos
const float KP    = 5.0f;      
const float KI    = 3.0f;       
const float KD    = 0.0f;
const float I_MAX = 5000;
const float I_MIN = -5000;

// ── PID variables ───────────────────
static int64_t last_count   = 0;
static float   integral_acc = 0.0f;
static float filtered_velocity = 0.0f;
static float last_raw_velocity = 0.0f;
const float ALPHA = 0.3f;
static float last_velocity = 0.0f;

// ── Debug ISR control ────
volatile float   dbg_velocity = 0.0f;
volatile float   dbg_raw_velocity = 0.0f;
volatile float   dbg_error    = 0.0f;
volatile float   dbg_integral = 0.0f;
volatile float   dbg_output   = 0.0f;
volatile float   dbg_setpoint   = 0.0f;
volatile int32_t dbg_pwm      = 0;

// ── Sincronización ────────────────────────────────────────
hw_timer_t   *timer_1          = NULL;
hw_timer_t   *timer_2          = NULL;
portMUX_TYPE  mux            = portMUX_INITIALIZER_UNLOCKED;

volatile bool trayectoria_activa = false;
int last_button = 0;
uint32_t      ultimo_serial  = 0;


void IRAM_ATTR boton_isr() { 

    int now =millis();
    if (now - last_button >= 200){
        trayectoria_activa = !trayectoria_activa;
        last_button = now; }
}


//____________________________________________________________

void trapezoidal_times(float distance, float cruiser_vel, float max_aceleration) {

    float min_distance = (cruiser_vel * cruiser_vel) / max_aceleration;
    
    if (min_distance < fabsf(distance)) {                  //trapezodial times
        triangle = false;
        t_1 = cruiser_vel / max_aceleration;
        t_2 = fabsf(distance) / cruiser_vel;
        t_3 = t_1 + t_2;
    }
 
    else {                                       //triangle times           

        triangle = true;
        peak_vel = sqrt(fabsf(distance) * max_aceleration);
        t_1 = peak_vel/max_aceleration;
        t_2 = 2 * t_1;

    }

}

void IRAM_ATTR trapezoidal_profile(){

    now_count ++;
    profile_time = (now_count - 1) * profile_DT;
    bool trapecio = !triangle;

    if (trapecio){                                 //trapozoidal_profile

        if (profile_time < t_1) {
            dinamic_setpoint = max_acelaration * profile_time;
        }
        else if ( profile_time >= t_1 && profile_time < t_2){
            dinamic_setpoint = cruiser_vel; 
        }
        else if (profile_time >=t_2 && profile_time <= t_3){
            dinamic_setpoint = cruiser_vel - max_acelaration * (profile_time - t_2);
        }
        else {
            dinamic_setpoint = 0;
        }
    }

    else{

        if(profile_time < t_1){
            dinamic_setpoint = max_acelaration * profile_time;
        }
        else if(profile_time >= t_1 && profile_time <= t_2){
            dinamic_setpoint = peak_vel - max_acelaration * (profile_time - t_1);
        }
        else{
            dinamic_setpoint = 0;
        }
    }

}

//____________________________________________________________
void IRAM_ATTR PI_control() {

    
    portENTER_CRITICAL_ISR(&mux);
    int64_t c = encoder.getCount();
    float setpoint = dinamic_setpoint;
    portEXIT_CRITICAL_ISR(&mux);

    //velocidad pulso/s
    float raw_velocity = (float)(c - last_count) / DT;
    last_count = c;

    //filtro EMA
    filtered_velocity = ALPHA * raw_velocity + (1-ALPHA) * filtered_velocity; 


    //Error en pulsos/s
    float error = setpoint - filtered_velocity;

    //acumulacion de error (integral)
    integral_acc += error * DT; 
    if (integral_acc >  I_MAX) integral_acc =  I_MAX;
    if (integral_acc <  I_MIN) integral_acc =  I_MIN;


    //limpia del acumulado en oversetpoint
    if (fabsf(setpoint) - fabsf(filtered_velocity) < -200) integral_acc = 0;

    //derivativo
    float derivative_term = -(filtered_velocity - last_velocity);
    last_velocity = filtered_velocity;

    //PID 
    float output = (KP * error) + (KI * integral_acc) + (KD * derivative_term);

    //saturacion pwm
    int abs_pwm = (int32_t)fabsf(output);
    if (abs_pwm < PWM_MIN) abs_pwm = PWM_MIN;
    if (abs_pwm > PWM_MAX) abs_pwm = PWM_MAX;

    if (output >= 0) {
        ledcWrite(CH_RPWM,0);
        ledcWrite(CH_LPWM,abs_pwm);
    }
    else {

        ledcWrite(CH_LPWM,0);
        ledcWrite(CH_RPWM, abs_pwm);
    }

    dbg_velocity = filtered_velocity;
    dbg_raw_velocity = raw_velocity;
    dbg_error    = error;
    dbg_integral = integral_acc;
    dbg_output   = output;
    dbg_pwm      = abs_pwm;
    dbg_setpoint   = setpoint;
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
    timer_1 = timerBegin(0, 80, true);
    timerAttachInterrupt(timer_1, &PI_control, true);
    timerAlarmWrite(timer_1, alarm_ticks, true);
    // Arranca desactivado, lo habilita el botón

    timer_2 = timerBegin(1,80,true);
    timerAttachInterrupt(timer_2,&trapezoidal_profile,true);
    timerAlarmWrite(timer_2,profile_DT * 1000000, true);


    trapezoidal_times(distance,cruiser_vel,max_acelaration);
}

void loop() {
    

    if (trayectoria_activa) {
        timerAlarmEnable(timer_1);
        timerAlarmEnable(timer_2);

    }

    else {

        timerAlarmDisable(timer_1);
        //last_count   = encoder.getCount();
        integral_acc = 0.0f;
        filtered_velocity = 0.0f;
        ledcWrite(CH_RPWM,0);
        ledcWrite(CH_LPWM,0);
        timerAlarmDisable(timer_2);
        profile_time = 0;
        now_count = 0;
    }

    // Serial mmonitor
    uint32_t ahora = millis();
    if (ahora - ultimo_serial >= 20) {
        ultimo_serial = ahora;

        Serial.printf(">set:%.2f\n",dbg_setpoint);
        Serial.printf(">raw_velocity:%.2f\n",dbg_raw_velocity);
        Serial.printf(">velocity:%.2f\n",dbg_velocity);

        Serial.printf("SET:%.2f | vel:%.2f | raw vel:%.2f | err:%.2f | I:%.4f | out: %.2f| pwm:%d\n",
                dbg_setpoint, dbg_velocity,dbg_raw_velocity,dbg_error,dbg_integral,dbg_output,dbg_pwm);
    }
}