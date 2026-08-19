#include <Arduino.h>
#include <ESP32Encoder.h>

// ── Encoder Motor 1 ────          // ── Encoder Motor 2 ────
const int ENC_M1_A = 18;            const int ENC_M2_A = 21;
const int ENC_M1_B = 19;            const int ENC_M2_B = 22;
ESP32Encoder encoder_m1;            ESP32Encoder encoder_m2;

                
// ── IBT_2 Motor 1 ────          // ── L298N Motor 2 ────
const int PIN_RPWM  = 15;           const int PIN_ENA  = 25;
const int PIN_LPWM  = 2;            const int PIN_IN1  = 33;
const int PIN_R_EN  =  4;           const int PIN_IN2  = 32;
const int PIN_L_EN  =  5;

//── Componentes ────────────────
const int PIN_BOTON      = 13; 
//const int PIN_BOTON_SEL  = 23; 
const int PIN_POT_1      = 14; 
const int PIN_POT_2      = 27; 
const int PIN_POT_3      = 26; 


// ── PWM ───────────────────────────────────────────────────────────────────────────────
const int PWM_FREQ       = 15000;               const int PWM_MIN_m1 =  500;
const int PWM_RESOLUTION =    12;               const int PWM_MAX_m1 =  4095;
const int CH_RPWM = 0; 
const int CH_LPWM = 1;                          const int PWM_MIN_m2 =  1750;  //1530 aparentes 
const int CH_ENA  = 2;                          const int PWM_MAX_m2 = 4095;  


//-Trapezoidal parameters Motor 1 ──────────        //-Trapezoidal parameters Motor 2 ────────
const float cruiser_vel_m1     =  4000.0f;          const float cruiser_vel_m2     =  3000.0f;  
const float max_acelaration_m1 =  4000.0f;          const float max_acelaration_m2 =  1500.0f;  
const float distance_m1        = 25000.0f;          const float distance_m2        = 15000.0f;  
const float DT_1               =     0.01f;         const float DT_2               =     0.01f; 


//-Trapezoidal variables Motor 1 ────────────       //-Trapezoidal variables Motor 2 ────────────────────────────
volatile int   now_count_m1    = 0;                 volatile int   now_count_m2    = 0;
volatile float profile_time_m1 = 0.0f;              volatile float profile_time_m2 = 0.0f;
volatile float vel_setpoint_m1 = 0.0f;              volatile float vel_setpoint_m2 = 0.0f;
volatile float pos_setpoint_m1 = 0.0f;              volatile float pos_setpoint_m2 = 0.0f;
volatile float x_ac_m1         = 0.0f;              volatile float x_ac_m2         = 0.0f;
volatile float t_1_m1 = 0.0f;                       volatile float t_1_m2 = 0.0f, t_2_m2 = 0.0f, t_3_m2 = 0.0f;
volatile float t_2_m1 = 0.0f;                       volatile bool  triangle_m2 = false;
volatile float t_3_m1 = 0.0f;                       volatile float peak_vel_m2 = 0.0f;
volatile bool  triangle_m1 = false;
volatile float peak_vel_m1 = 0.0f;


// ── PID parameters ──────────────────────────
float KP_m1 = 64.0f, KD_m1 = 2.33f;
float KP_m2 = 0.0f, KD_m2 = 0.0f;
const float I_MAX =  5000.0f;
const float I_MIN = -5000.0f;
const float ALPHA =     0.25f;

// ── PID variables Motor 1 ────────────────            // ── PID variables Motor 2 ────────────────────────────────────
int64_t        last_count_m1        = 0;                int64_t        last_count_m2        = 0;
volatile float c_m1                 = 0.0f;             volatile float c_m2                 = 0.0f;
volatile float integral_acc_m1      = 0.0f;             volatile float integral_acc_m2      = 0.0f;
volatile float filtered_velocity_m1 = 0.0f;             volatile float filtered_velocity_m2 = 0.0f;
volatile float last_velocity_m1     = 0.0f;             volatile float last_velocity_m2     = 0.0f;
volatile float last_error_m1        = 0.0f;             volatile float last_error_m2        = 0.0f;
volatile float raw_velocity_m1      = 0.0f;             volatile float raw_velocity_m2      = 0.0f;
volatile float derivative_term_m1   = 0.0f;             volatile float derivative_term_m2   = 0.0f;
volatile float error_pos_m1         = 0.0f;             volatile float error_pos_m2         = 0.0f;
volatile float error_vel_m1         = 0.0f;             volatile float error_vel_m2         = 0.0f;
volatile float output_m1            = 0.0f;             volatile float output_m2            = 0.0f;

// ── Timers ───────────────────────────────────
hw_timer_t* timer_1 = NULL;  
hw_timer_t* timer_2 = NULL; 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ── Estado ──────────────────────────────────
volatile bool trayectoria_activa = false;
volatile bool boton_flag         = false;
volatile int  motor_sel          = 1; 
bool          ultimo_estado      = false;
uint32_t      last_button        = 0;
uint32_t      last_button_sel    = 0;
uint32_t      ultimo_serial      = 0;
uint32_t      ultimo_pot         = 0;


// ════════════════════════════════════════════════════════════
void IRAM_ATTR boton_isr() {
    uint32_t now = millis();
    if (now - last_button >= 500) {
        trayectoria_activa = !trayectoria_activa;
        last_button = now;
    }
}


// ════════════════════════════════════════════════════════════
void trapezoidal_times_m1(float dist, float vel, float acel) {

    float min_distance = (vel * vel) / acel;
    x_ac_m1 = (vel * vel) / (2.0f * acel);

    if (min_distance < fabsf(dist)) {           // TRAPECIO
        triangle_m1 = false;
        t_1_m1 = vel / acel;
        t_2_m1 = fabsf(dist) / vel;
        t_3_m1 = t_1_m1 + t_2_m1;
    }
    else {                                       // TRIÁNGULO
        triangle_m1 = true;
        peak_vel_m1 = sqrtf(fabsf(dist) * acel);
        t_1_m1 = peak_vel_m1 / acel;
        t_2_m1 = 2.0f * t_1_m1;
    }
}

// ════════════════════════════════════════════════════════════
void trapezoidal_times_m2(float dist, float vel, float acel) {

    float min_distance = (vel * vel) / acel;
    x_ac_m2 = (vel * vel) / (2.0f * acel);

    if (min_distance < fabsf(dist)) {           // TRAPECIO
        triangle_m2 = false;
        t_1_m2 = vel / acel;
        t_2_m2 = fabsf(dist) / vel;
        t_3_m2 = t_1_m2 + t_2_m2;
    }
    else {                                       // TRIÁNGULO
        triangle_m2 = true;
        peak_vel_m2 = sqrtf(fabsf(dist) * acel);
        t_1_m2 = peak_vel_m2 / acel;
        t_2_m2 = 2.0f * t_1_m2;
    }
}

// ════════════════════════════════════════════════════════════
void IRAM_ATTR trapezoidal_profile_m1() {

    now_count_m1++;
    profile_time_m1 = (now_count_m1 - 1) * DT_1;

    if (!triangle_m1) {                          // TRAPECIO
        if (profile_time_m1 < t_1_m1) {
            vel_setpoint_m1 = max_acelaration_m1 * profile_time_m1;
            pos_setpoint_m1 = max_acelaration_m1/2.0f * (profile_time_m1 * profile_time_m1);
        }
        else if (profile_time_m1 < t_2_m1) {
            vel_setpoint_m1 = cruiser_vel_m1;
            pos_setpoint_m1 = x_ac_m1 + cruiser_vel_m1 * (profile_time_m1 - t_1_m1);
        }
        else if (profile_time_m1 <= t_3_m1) {
            vel_setpoint_m1 = cruiser_vel_m1 - max_acelaration_m1 * (profile_time_m1 - t_2_m1);
            pos_setpoint_m1 = (distance_m1 - x_ac_m1) + cruiser_vel_m1 * (profile_time_m1 - t_2_m1) - (max_acelaration_m1/2.0f) * ((profile_time_m1 - t_2_m1) * (profile_time_m1 - t_2_m1));
        }
        else {
            vel_setpoint_m1 = 0.0f;
            pos_setpoint_m1 = distance_m1;
        }
    }
    else {                                       // TRIÁNGULO
        if (profile_time_m1 < t_1_m1) {
            vel_setpoint_m1 = max_acelaration_m1 * profile_time_m1;
            pos_setpoint_m1 = max_acelaration_m1/2.0f * (profile_time_m1 * profile_time_m1);
        }
        else if (profile_time_m1 < t_2_m1) {
            vel_setpoint_m1 = peak_vel_m1 - max_acelaration_m1 * (profile_time_m1 - t_1_m1);
            pos_setpoint_m1 = (distance_m1 - x_ac_m1) + peak_vel_m1 * (profile_time_m1 - t_1_m1) - (max_acelaration_m1/2.0f) * ((profile_time_m1 - t_1_m1) * (profile_time_m1 - t_1_m1));
        }
        else {
            vel_setpoint_m1 = 0.0f;
        }
    }
}

// ════════════════════════════════════════════════════════════
void IRAM_ATTR trapezoidal_profile_m2() {

    now_count_m2++;
    profile_time_m2 = (now_count_m2 - 1) * DT_2;

    if (!triangle_m2) {                          // TRAPECIO
        if (profile_time_m2 < t_1_m2) {
            vel_setpoint_m2 = max_acelaration_m2 * profile_time_m2;
            pos_setpoint_m2 = max_acelaration_m2/2.0f * (profile_time_m2 * profile_time_m2);
        }
        else if (profile_time_m2 < t_2_m2) {
            vel_setpoint_m2 = cruiser_vel_m2;
            pos_setpoint_m2 = x_ac_m2 + cruiser_vel_m2 * (profile_time_m2 - t_1_m2);
        }
        else if (profile_time_m2 <= t_3_m2) {
            vel_setpoint_m2 = cruiser_vel_m2 - max_acelaration_m2 * (profile_time_m2 - t_2_m2);
            pos_setpoint_m2 = (distance_m2 - x_ac_m2) + cruiser_vel_m2 * (profile_time_m2 - t_2_m2) - (max_acelaration_m2/2.0f) * ((profile_time_m2 - t_2_m2) * (profile_time_m2 - t_2_m2));
        }
        else {
            vel_setpoint_m2 = 0.0f;
            pos_setpoint_m2 = distance_m2;
        }
    }
    else {                                       // TRIÁNGULO
        if (profile_time_m2 < t_1_m2) {
            vel_setpoint_m2 = max_acelaration_m2 * profile_time_m2;
            pos_setpoint_m2 = max_acelaration_m2/2.0f * (profile_time_m2 * profile_time_m2);
        }
        else if (profile_time_m2 < t_2_m2) {
            vel_setpoint_m2 = peak_vel_m2 - max_acelaration_m2 * (profile_time_m2 - t_1_m2);
            pos_setpoint_m2 = (distance_m2 - x_ac_m2) + peak_vel_m2 * (profile_time_m2 - t_1_m2) - (max_acelaration_m2/2.0f) * ((profile_time_m2 - t_1_m2) * (profile_time_m2 - t_1_m2));
        }
        else {
            vel_setpoint_m2 = 0.0f;
        }
    }
}

// ════════════════════════════════════════════════════════════
void IRAM_ATTR PID_control_m1() {

    c_m1 = encoder_m1.getCount();
    raw_velocity_m1 = (float)(c_m1 - last_count_m1) / DT_1;
    last_count_m1 = c_m1;

    filtered_velocity_m1 = ALPHA * raw_velocity_m1 + (1.0f - ALPHA) * filtered_velocity_m1;

    error_vel_m1 = vel_setpoint_m1 - filtered_velocity_m1;
    error_pos_m1 = pos_setpoint_m1 - c_m1;

    output_m1 = (KP_m1 * error_pos_m1) + (KD_m1 * error_vel_m1);

    int32_t abs_pwm = (int32_t)fabsf(output_m1);
    if (abs_pwm < PWM_MIN_m1) abs_pwm = PWM_MIN_m1;
    if (abs_pwm > PWM_MAX_m1) abs_pwm = PWM_MAX_m1;

    if (pos_setpoint_m1 == distance_m1 && fabsf(error_pos_m1) < 5) {
        ledcWrite(CH_RPWM, 0);
        ledcWrite(CH_LPWM, 0);
    }
    else {
        if (output_m1 > 0.0f) {
            ledcWrite(CH_RPWM, abs_pwm);
            ledcWrite(CH_LPWM, 0);
        }
        else if (output_m1 < 0.0f) {
            ledcWrite(CH_LPWM, abs_pwm);
            ledcWrite(CH_RPWM, 0);
        }
    }
}

// ════════════════════════════════════════════════════════════
void IRAM_ATTR PID_control_m2() {

    c_m2 = encoder_m2.getCount();
    raw_velocity_m2 = (float)(c_m2 - last_count_m2) / DT_2;
    last_count_m2 = c_m2;

    filtered_velocity_m2 = ALPHA * raw_velocity_m2 + (1.0f - ALPHA) * filtered_velocity_m2;

    error_vel_m2 = vel_setpoint_m2 - filtered_velocity_m2;
    error_pos_m2 = pos_setpoint_m2 - c_m2;

    output_m2 = (KP_m2 * error_pos_m2) + (KD_m2 * error_vel_m2);

    int32_t abs_pwm = (int32_t)fabsf(output_m2);
    if (abs_pwm < PWM_MIN_m2) abs_pwm = PWM_MIN_m2;
    if (abs_pwm > PWM_MAX_m2) abs_pwm = PWM_MAX_m2;

    if (pos_setpoint_m2 == distance_m2 && fabsf(error_pos_m2) < 5) {
        ledcWrite(CH_ENA, 0);
    }
    else {
        if (output_m2 > 0.0f) {
            ledcWrite(CH_ENA, abs_pwm);
            digitalWrite(PIN_IN1, HIGH);
            digitalWrite(PIN_IN2, LOW);
        }
        else if (output_m2 < 0.0f) {
            ledcWrite(CH_ENA, abs_pwm);
            digitalWrite(PIN_IN1, LOW);
            digitalWrite(PIN_IN2, HIGH);
        }
    }
}

// ════════════════════════════════════════════════════════════
void IRAM_ATTR motor1_routine() {
    trapezoidal_profile_m1();
    PID_control_m1();
}

void IRAM_ATTR motor2_routine() {
    trapezoidal_profile_m2();
    PID_control_m2();
}

// ════════════════════════════════════════════════════════════
void setup() {
    //Serial.begin(115200);
    Serial.begin(921600);

    pinMode(PIN_R_EN, OUTPUT); digitalWrite(PIN_R_EN, HIGH);
    pinMode(PIN_L_EN, OUTPUT); digitalWrite(PIN_L_EN, HIGH);
    pinMode(PIN_IN1, OUTPUT);  digitalWrite(PIN_IN1, HIGH);
    pinMode(PIN_IN2, OUTPUT);  digitalWrite(PIN_IN2, HIGH);

    ledcSetup(CH_RPWM, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(PIN_RPWM, CH_RPWM); ledcWrite(CH_RPWM, 0);
    ledcSetup(CH_LPWM, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(PIN_LPWM, CH_LPWM); ledcWrite(CH_LPWM, 0);
    ledcSetup(CH_ENA,  PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(PIN_ENA,  CH_ENA);  ledcWrite(CH_ENA, 0);

    encoder_m1.attachFullQuad(ENC_M1_A, ENC_M1_B); encoder_m1.setCount(0);
    encoder_m2.attachFullQuad(ENC_M2_A, ENC_M2_B); encoder_m2.setCount(0);

    pinMode(PIN_BOTON, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_BOTON), boton_isr, RISING);

    uint32_t ticks_m1 = (uint32_t)(DT_1 * 1000000.0f);
    timer_1 = timerBegin(0, 80, true);
    timerAttachInterrupt(timer_1, &motor1_routine, true);
    timerAlarmWrite(timer_1, ticks_m1, true);
    timerAlarmEnable(timer_1);
    timerStop(timer_1);

    uint32_t ticks_m2 = (uint32_t)(DT_2 * 1000000.0f);
    timer_2 = timerBegin(1, 80, true);
    timerAttachInterrupt(timer_2, &motor2_routine, true);
    timerAlarmWrite(timer_2, ticks_m2, true);
    timerAlarmEnable(timer_2);
    timerStop(timer_2);

    trapezoidal_times_m1(distance_m1, cruiser_vel_m1, max_acelaration_m1);
    trapezoidal_times_m2(distance_m2, cruiser_vel_m2, max_acelaration_m2);
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
        ledcWrite(CH_ENA, 0);

        // Motor 1 — PID
        integral_acc_m1      = 0.0f;
        filtered_velocity_m1 = 0.0f;
        last_velocity_m1     = 0.0f;
        last_error_m1        = 0.0f;
        last_count_m1        = 0;
        derivative_term_m1   = 0.0f;
        raw_velocity_m1      = 0.0f;
        error_pos_m1         = 0.0f;
        error_vel_m1         = 0.0f;
        output_m1            = 0.0f;

        // Motor 1 — Perfil
        profile_time_m1 = 0.0f;
        now_count_m1    = 0;
        vel_setpoint_m1 = 0.0f;
        c_m1            = 0;
        pos_setpoint_m1 = 0;
        encoder_m1.clearCount();

        // Motor 2 — PID
        integral_acc_m2      = 0.0f;
        filtered_velocity_m2 = 0.0f;
        last_velocity_m2     = 0.0f;
        last_error_m2        = 0.0f;
        last_count_m2        = 0;
        derivative_term_m2   = 0.0f;
        raw_velocity_m2      = 0.0f;
        error_pos_m2         = 0.0f;
        error_vel_m2         = 0.0f;
        output_m2            = 0.0f;

        // Motor 2 — Perfil
        profile_time_m2 = 0.0f;
        now_count_m2    = 0;
        vel_setpoint_m2 = 0.0f;
        c_m2            = 0;
        pos_setpoint_m2 = 0;
        encoder_m2.clearCount();

        KP_m2 = map(analogRead(PIN_POT_1), 0, 4095, 0, 200) / 1.0f;
        KD_m2 = map(analogRead(PIN_POT_3), 0, 4095, 0, 200) / 10.0f;
        
    }

    // Serial monitor
    // ════════════════════════════════════════════════════════════

    // Serial monitor
    uint32_t ahora = millis();
    if (ahora - ultimo_serial >= 10) {
        ultimo_serial = ahora;

        
        Serial.print(">vel_m1:");      Serial.println(filtered_velocity_m1);
        Serial.print(">position_m1:"); Serial.println(c_m1);
        Serial.print(">vel_set_m1:");  Serial.println(vel_setpoint_m1);
        Serial.print(">pos_set_m1:");  Serial.println(pos_setpoint_m1);
        Serial.print(">Kp_m1:");       Serial.println(KP_m1);
        Serial.print(">Kd_m1:");       Serial.println(KD_m1, 5);

        Serial.print(">vel_m2:");      Serial.println(filtered_velocity_m2);
        Serial.print(">position_m2:"); Serial.println(c_m2);
        Serial.print(">vel_set_m2:");  Serial.println(vel_setpoint_m2);
        Serial.print(">pos_set_m2:");  Serial.println(pos_setpoint_m2);
        Serial.print(">Kp_m2:");       Serial.println(KP_m2);
        Serial.print(">Kd_m2:");       Serial.println(KD_m2, 5);
    }
}
