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

// ── Parámetros PI ─────────────────────────────────────────
const float SETPOINT_PPS = 4100.0f;     // pulsos/s 

const float DT    = 0.02f;     // intervalo de control en segundos
const float KP    = 4.5f;      
const float KI    = 3.5f;       
const float KD    = 0.035f;

// Anti-windup:
//const float I_MAX =  (float)PWM_MAX / KI;
//const float I_MIN = -(float)PWM_MAX / KI;
const float I_MAX = 5000;
const float I_MIN = -5000;

// ── Variables internas del controlador ───────────────────
static int64_t last_count   = 0;
static float   integral_acc = 0.0f;
static float filtered_velocity = 0.0f;
static float last_raw_velocity = 0.0f;
const float ALPHA = 0.3f;
static float last_velocity = 0.0f;
volatile float last_error = 0.0f;

// ── Debug (escritura en ISR, lectura en loop) ────
volatile float   dbg_velocity = 0.0f;
volatile float   dbg_raw_velocity = 0.0f;
volatile float   dbg_error    = 0.0f;
volatile float   dbg_integral = 0.0f;
volatile float   dbg_derivative_term = 0.0f;
volatile float   dbg_output   = 0.0f;
volatile int32_t dbg_pwm      = 0;

// ── Sincronización ────────────────────────────────────────
hw_timer_t   *timer          = NULL;
portMUX_TYPE  mux            = portMUX_INITIALIZER_UNLOCKED;
volatile bool control_activo = false;
uint32_t      ultimo_serial  = 0;
volatile int last_button     = 0;



// ════════════════════════════════════════════════════════════
void IRAM_ATTR PID_control() {

    
    portENTER_CRITICAL_ISR(&mux);
    int64_t c = encoder.getCount();
    portEXIT_CRITICAL_ISR(&mux);

    //velocidad pulso/s
    float raw_velocity = (float)(c - last_count) / DT;
    last_count = c;

    //filtro EMA
    filtered_velocity = ALPHA * raw_velocity + (1-ALPHA) * filtered_velocity; 


    //Error en pulsos/s
    float error = SETPOINT_PPS - filtered_velocity;

    //acumulacion de error (integral)
    integral_acc += error * DT; 
    if (integral_acc >  I_MAX) integral_acc =  I_MAX;
    if (integral_acc <  I_MIN) integral_acc =  I_MIN;


    //limpia del acumulado en oversetpoint
    if (fabsf(SETPOINT_PPS) - fabsf(filtered_velocity) < -200) integral_acc = 0;

    //derivativo
    float derivative_term = (error - last_error)/DT;
    last_error = error;
    //float derivative_term = -(filtered_velocity - last_velocity)/DT;
    //last_velocity = filtered_velocity;


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

    dbg_derivative_term = derivative_term;
    dbg_velocity = filtered_velocity;
    dbg_raw_velocity = raw_velocity;
    dbg_error    = error;
    dbg_integral = integral_acc;
    dbg_output   = output;
    dbg_pwm      = abs_pwm;
}

// Botón arranca el control
void IRAM_ATTR boton_isr() { 
    
    int now =millis();
    if (now - last_button >= 200){
        control_activo = !control_activo;
        last_button = now; } }


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
    timerAttachInterrupt(timer, &PID_control, true);
    timerAlarmWrite(timer, alarm_ticks, true);
    // Arranca desactivado, lo habilita el botón
}

void loop() {
   
    if (control_activo) {
        timerAlarmEnable(timer);
    }

    else{

        timerAlarmDisable(timer);
        integral_acc = 0.0f;
        filtered_velocity = 0.0f;
        ledcWrite(CH_LPWM,0);
        ledcWrite(CH_RPWM,0);
        last_count = 0;
        encoder.clearCount();
        dbg_raw_velocity = 0;
    }

    // Serial
    uint32_t ahora = millis();
    if (ahora - ultimo_serial >= 20) {
        ultimo_serial = ahora;

        Serial.print(">set:");
        Serial.println(SETPOINT_PPS);
        Serial.print(">raw_vel:");
        Serial.println(dbg_raw_velocity);
        Serial.print(">vel:");
        Serial.println(filtered_velocity);
         Serial.print(">derivative_term:");
        Serial.println(dbg_derivative_term);
        //Serial.printf(">derivative_term:%.2f\n",dbg_derivative_term);
        //Serial.printf(">raw_velocity:%.2f\n",dbg_raw_velocity);
        //Serial.printf(">velocity:%.2f\n",dbg_velocity);

        Serial.printf("SET:%.2f | vel:%.2f | raw vel:%.2f | err:%.2f | I:%.4f | out: %.2f| pwm:%d\n",
                SETPOINT_PPS, dbg_velocity,dbg_raw_velocity,dbg_error,dbg_integral,dbg_output,dbg_pwm);
    }
}