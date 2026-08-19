#include <Arduino.h>
#include <ESP32Encoder.h>

// ── Encoder ───────────────────────────────────────────────
#define ENC_A  18
#define ENC_B  19
ESP32Encoder encoder;

// ── IBT_2 ────────────────────────────────────────
#define PIN_RPWM   15
#define PIN_LPWM    2
#define PIN_R_EN    4
#define PIN_L_EN    5

//──Componentes ────────────────────────────────────────
#define PIN_BOTON  13
#define PIN_POT_1  14
#define PIN_POT_2  27
#define PIN_POT_3  26
const float pot_DT = 0.25;

// ── PWM ───────────────────────────────────────────────────
#define PWM_FREQ       15000
#define PWM_RESOLUTION 12        
#define CH_RPWM        0
#define CH_LPWM        1
#define PWM_MIN        500  //490 a 12v,
#define PWM_MAX        4095

//-Trapezoidal parameters─────────────────────────────────────
const float cruiser_vel     = 4000.0f;
const float max_acelaration = 2000.0f;
const float distance        = 20000.0f;
const float profile_DT      = 0.01f;

//-Trapezoidal variables─────────────────────────────────────
volatile int   now_count        = 0;
volatile float profile_time     = 0.0f;
volatile float vel_setpoint = 0.0f;
volatile float pos_setpoint = 0.0f;
volatile float x_ac         = 0.0f;


//-Trapezoidal times──────────────────────────────────────────
volatile float t_1      = 0.0f;
volatile float t_2      = 0.0f;
volatile float t_3      = 0.0f;
volatile bool  triangle = false;
volatile float peak_vel = 0.0f;

// ── PID parameters ─────────────────────────────────────────
const float DT    = 0.01f;
float KP    = 0.0f;
float KI    = 0.0f;
float KD    = 0.0f;
const float I_MAX =  5000.0f;
const float I_MIN = -5000.0f;

// ── PID variables ──────────────────────────────────────────
int64_t last_count        = 0;
volatile float   integral_acc      = 0.0f;
volatile float   filtered_velocity = 0.0f;
volatile float   last_velocity     = 0.0f;
volatile float   last_error     = 0.0f;
volatile float   raw_velocity     = 0.0f;
volatile float          derivative_term   = 0.0f;
const  float   ALPHA             = 0.25f;
volatile float c = 0.0f;
volatile float error_pos = 0.0f;
volatile float error_vel = 0.0f;
volatile float output = 0.0f;

// ── Timers ─────────────────────────────────────────────────
hw_timer_t* timer_1 = NULL;
hw_timer_t* timer_2 = NULL;
hw_timer_t* timer_3 = NULL;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ── Estado ─────────────────────────────────────────────────
volatile bool trayectoria_activa = false;
volatile bool boton_flag         = false;  
bool          ultimo_estado      = false;  
uint32_t      last_button        = 0;
uint32_t      ultimo_serial      = 0;
uint32_t      ultimo_pot      = 0;

// ════════════════════════════════════════════════════════════
// ISR Botón — solo bandera, sin millis()
// ════════════════════════════════════════════════════════════
void IRAM_ATTR boton_isr() { 

    int now =millis();
    if (now - last_button >= 500){
        trayectoria_activa = !trayectoria_activa;
        last_button = now; }
}

//void IRAM_ATTR POT_monitor() { 



// ════════════════════════════════════════════════════════════
void trapezoidal_times(float dist, float vel, float acel) {

    float min_distance = (vel * vel) / acel;
    x_ac = (vel * vel) / (2.0f * acel);
    

    if (min_distance < fabsf(dist)) {           // TRAPECIO
        triangle = false;
        t_1 = vel / acel;
        t_2 = fabsf(dist) / vel;
        t_3 = t_1 + t_2;
    }
    else {                                       // TRIÁNGULO
        triangle = true;
        peak_vel = sqrtf(fabsf(dist) * acel);
        t_1 = peak_vel / acel;
        t_2 = 2.0f * t_1;
    }
}

// ════════════════════════════════════════════════════════════
void IRAM_ATTR trapezoidal_profile() {

    now_count++;
    profile_time = (now_count - 1) * profile_DT;

    if (!triangle) {                             // TRAPECIO
        if (profile_time < t_1) {
            vel_setpoint = max_acelaration * profile_time;
            pos_setpoint = max_acelaration/2.0f * (profile_time * profile_time); 
        }
        else if (profile_time < t_2) {
            vel_setpoint = cruiser_vel;
            pos_setpoint = x_ac + cruiser_vel * (profile_time - t_1);
        }
        else if (profile_time <= t_3) {
            vel_setpoint = cruiser_vel - max_acelaration * (profile_time - t_2);
            pos_setpoint = (distance - x_ac) + cruiser_vel * (profile_time - t_2) - (max_acelaration/2.0f) * ((profile_time - t_2) * (profile_time - t_2));
        }
        else {
            vel_setpoint = 0.0f;
            pos_setpoint = distance;
        }
    }
    else {                                       // TRIÁNGULO
        if (profile_time < t_1) {
            vel_setpoint = max_acelaration * profile_time;
            pos_setpoint = max_acelaration/2.0f * (profile_time * profile_time); 
        }
        else if (profile_time < t_2) {
            vel_setpoint = peak_vel - max_acelaration * (profile_time - t_1);
            pos_setpoint = (distance - x_ac) + peak_vel * (profile_time - t_1) - (max_acelaration/2.0f) * ((profile_time - t_1) * (profile_time - t_1));
        }
        else {
            vel_setpoint = 0.0f;
        }
    }
}

// ════════════════════════════════════════════════════════════
void IRAM_ATTR PID_control() {

    //portENTER_CRITICAL_ISR(&mux);
    c      = encoder.getCount();
    //float setpoint = dinamic_setpoint;
    //portEXIT_CRITICAL_ISR(&mux);

    raw_velocity = (float)(c - last_count) / DT;
    last_count = c;

    filtered_velocity = ALPHA * raw_velocity + (1.0f - ALPHA) * filtered_velocity;

    error_vel= vel_setpoint - filtered_velocity;
    error_pos = pos_setpoint - c;


    //integral_acc += error * DT;
    //if (integral_acc >  I_MAX) integral_acc =  I_MAX;
    //if (integral_acc <  I_MIN) integral_acc =  I_MIN;

    //if (fabsf(dinamic_setpoint) - fabsf(filtered_velocity) < -500.0f) integral_acc = 0.0f;

    //derivative_term = -(filtered_velocity - last_velocity) / DT ;
    //last_velocity = filtered_velocity;
    //derivative_term = (error - last_error)/DT;
    //last_error = error;

    output = (KP * error_pos) + (KD * error_vel);

    int32_t abs_pwm = (int32_t)fabsf(output);
    if (abs_pwm < PWM_MIN) abs_pwm = PWM_MIN;
    if (abs_pwm > PWM_MAX) abs_pwm = PWM_MAX;

    if(pos_setpoint == distance && abs(error_pos)<5){
        ledcWrite(CH_LPWM, 0);
        ledcWrite(CH_RPWM, 0); 
    }

    else{
        
        if (output > 0.0f) {
            ledcWrite(CH_RPWM, abs_pwm);
            ledcWrite(CH_LPWM, 0);
        }
        else if(output < 0.0f) {  
            
            ledcWrite(CH_LPWM, abs_pwm);
            ledcWrite(CH_RPWM, 0);    
        }
    }
}
    

// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    pinMode(PIN_R_EN, OUTPUT); digitalWrite(PIN_R_EN, HIGH);
    pinMode(PIN_L_EN, OUTPUT); digitalWrite(PIN_L_EN, HIGH);

    ledcSetup(CH_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_LPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_RPWM, CH_RPWM);
    ledcAttachPin(PIN_LPWM, CH_LPWM);
    ledcWrite(CH_RPWM, 0);
    ledcWrite(CH_LPWM, 0);

    encoder.attachFullQuad(ENC_A, ENC_B);
    encoder.setCount(0);

    pinMode(PIN_BOTON, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_BOTON), boton_isr, RISING);

    uint32_t ticks_control = (uint32_t)(DT * 1000000.0f);
    timer_1 = timerBegin(0, 80, true);
    timerAttachInterrupt(timer_1, &PID_control, true);
    timerAlarmWrite(timer_1, ticks_control, true);
    timerAlarmEnable(timer_1);
    timerStop(timer_1);

    uint32_t ticks_perfil = (uint32_t)(profile_DT * 1000000.0f);
    timer_2 = timerBegin(1, 80, true);
    timerAttachInterrupt(timer_2, &trapezoidal_profile, true);
    timerAlarmWrite(timer_2, ticks_perfil, true);
    timerAlarmEnable(timer_2);
    timerStop(timer_2);

    //uint32_t ticks_pot = (uint32_t)(pot_DT * 1000000.0f);
    //timer_3 = timerBegin(2, 80, true);
    //timerAttachInterrupt(timer_3, &POT_monitor, true);
    //timerAlarmWrite(timer_3, ticks_pot, true);

    trapezoidal_times(distance, cruiser_vel, max_acelaration);
}

// ════════════════════════════════════════════════════════════
void loop() {

    
    if (trayectoria_activa) {
        timerStart(timer_1);
        timerStart(timer_2);

    }

    else {

        timerStop(timer_1);
        timerStop(timer_2);

        
        ledcWrite(CH_RPWM, 0);
        ledcWrite(CH_LPWM, 0);

        // PID
        integral_acc      = 0.0f;
        filtered_velocity = 0.0f;
        last_velocity     = 0.0f;
        last_error        = 0.0f;
        last_count        = 0.0f;
        derivative_term   = 0.0f;
        raw_velocity      = 0.0f;
        error_pos         = 0.0f;
        error_vel         = 0.0f;
        output            = 0.0f;

        // Perfil
        profile_time      = 0.0f;
        now_count         = 0;
        vel_setpoint      = 0.0f;
        c                 = 0;
        pos_setpoint      = 0; 
        encoder.clearCount(); 
    
        KP = map(analogRead(PIN_POT_1), 0, 4095, 0, 100) / 1.0f;
        KI = map(analogRead(PIN_POT_2), 0, 4095, 0, 400) / 2.0f;
        KD = map(analogRead(PIN_POT_3), 0, 4095, 0, 300)/100.0f;

        
    }

    // Serial monitor
    uint32_t ahora = millis();
    if (ahora - ultimo_serial >= 10) {
        ultimo_serial = ahora;

        float dbg_Kp = KP;
        float dbg_Ki = KI;
        float dbg_Kd = KD;
        //Serial.print(">raw_vel:");
        //Serial.println(dbg_raw_velocity);
        Serial.print(">vel:");
        Serial.println(filtered_velocity);
        Serial.print(">position:");
        Serial.println(c);
        Serial.print(">vel_set:");
        Serial.println(vel_setpoint);
        Serial.print(">pos_set:");
        Serial.println(pos_setpoint);
        Serial.print(">Kp:");
        Serial.println(dbg_Kp);
        Serial.print(">Ki:");
        Serial.println(dbg_Ki);
        Serial.print(">Kd:");
        Serial.println(dbg_Kd,5);
        
    }
}