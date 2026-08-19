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
#define SETPOINT_PPS   1895     // pulsos/s 
const float DT    = 0.1f;     // intervalo de control en segundos
const float KP    = 1.0f;      
const float KI    = 1.0f;      

// Parametros para la ISR PI (parametros/DT)
volatile float ISR_set = 0;            //Conversion de parametros/s a parametros/DT
volatile float ISR_Kp = 0;
volatile float ISR_Ki = 0;

// Variables persistentes de la ISR PI
static int64_t last_count   = 0;
static float   integral_acc = 0.0f;

// Anti-windup: el término I solo no puede saturar el PWM
//const float I_MAX =  (float)PWM_MAX / KI;
//const float I_MIN = -(float)PWM_MAX / KI;
const float I_MAX = 2047;
const float I_MIN = -2047;

// ── Debug (escritura atómica en ISR, lectura en loop) ────
volatile float   dbg_velocity = 0.0f;
volatile float   dbg_error    = 0.0f;
volatile float   dbg_integral = 0.0f;
volatile int32_t dbg_pwm      = 0;


// ── Sincronización ────────────────────────────────────────
hw_timer_t   *timer          = NULL;
portMUX_TYPE  mux            = portMUX_INITIALIZER_UNLOCKED;
volatile bool control_activo = false;
uint32_t      ultimo_serial  = 0;


// ════════════════════════════════════════════════════════════
void IRAM_ATTR PI_control() {

    // Lectura atómica del encoder
    portENTER_CRITICAL_ISR(&mux);
    int64_t c = encoder.getCount();
    portEXIT_CRITICAL_ISR(&mux);

    
    int velocity = c - last_count;      //velocidad en pulsos/DT
    last_count = c;

    
    float error = ISR_set - float(velocity); // error/DT
    integral_acc += error;
    //integral_acc = constrain(integral_acc,I_MIN,I_MAX);
    if (integral_acc >  I_MAX) integral_acc =  I_MAX;    
    if (integral_acc <  I_MIN) integral_acc =  I_MIN;

    // Ley PI — salida en unidades PWM
    int pwm = int(ISR_Kp * error + ISR_Ki * integral_acc);  //ley PI
    //pwm = constrain(pwm,PWM_MIN,PWM_MAX);
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (pwm > PWM_MAX) pwm = PWM_MAX;

    ledcWrite(CH_LPWM, (uint32_t)pwm);

    // Debug — float de 4 bytes, escritura atómica en ESP32
    dbg_velocity = velocity;
    dbg_error    = error;
    dbg_integral = integral_acc;
    dbg_pwm      = pwm;
}

// Botón arranca el control
void IRAM_ATTR boton_isr() { control_activo = true; }


// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    
    ISR_set = SETPOINT_PPS * DT;            //Conversion de parametros/s a parametros/DT
    ISR_Kp = KP/DT;
    ISR_Ki = KI/KI;

    uint32_t alarm_ticks = DT * 1000000;             //ajuste de la alarma en base a DT

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

    //Encoder
    encoder.attachFullQuad(ENC_A, ENC_B);
    encoder.setCount(0);

    // Botón
    pinMode(PIN_BOTON, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_BOTON), boton_isr, RISING);

    // Timer: 80MHz / prescaler 80 = 1 tick/µs → 1000 ticks = 1ms
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &PI_control, true);
    timerAlarmWrite(timer, alarm_ticks, true);
    // Arranca desactivado, lo habilita el botón
}

void loop() {
    // Botón presionado: limpia estado y arranca el control
    if (control_activo) {
        integral_acc = 0.0f;
        last_count   = encoder.getCount();
        timerAlarmEnable(timer);
        control_activo = false;
    }

    
    uint32_t ahora = millis();
    if (ahora - ultimo_serial >= 10) {
        ultimo_serial = ahora;
        float vel_pss = dbg_velocity/DT;
        float error_pss = dbg_error/DT;

        Serial.printf("SET:%d | vel:%.2f | err:%.2f | I:%.4f | pwm:%d\n",
                      SETPOINT_PPS,vel_pss,error_pss,dbg_integral,dbg_pwm);
    }
}