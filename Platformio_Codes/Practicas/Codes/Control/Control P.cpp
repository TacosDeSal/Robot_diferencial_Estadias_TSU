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
#define PWM_MIN        370
#define PWM_MAX        2047

// ── Parámetros P ──────────────────────────────────────────
const float SETPOINT_PULSOS = 10000.0f;   // posición destino en pulsos

const float DT = 0.02f;
const float KP = 5.0f;

// ── Variables internas ────────────────────────────────────
volatile float   dbg_position = 0.0f;
volatile float   dbg_error    = 0.0f;
volatile float   dbg_output   = 0.0f;
volatile int32_t dbg_pwm      = 0;

// ── Sincronización ────────────────────────────────────────
hw_timer_t   *timer          = NULL;
portMUX_TYPE  mux            = portMUX_INITIALIZER_UNLOCKED;
volatile bool control_activo = false;
uint32_t      ultimo_serial  = 0;


// ════════════════════════════════════════════════════════════
void IRAM_ATTR P_control() {

    portENTER_CRITICAL_ISR(&mux);
    int64_t c = encoder.getCount();
    portEXIT_CRITICAL_ISR(&mux);

    float position = (float)c;
    float error    = SETPOINT_PULSOS - position;

    // P
    float output = KP * error;

    // Saturación PWM
    int abs_pwm = (int32_t)fabsf(output);
    if (abs_pwm < PWM_MIN) abs_pwm = PWM_MIN;
    if (abs_pwm > PWM_MAX) abs_pwm = PWM_MAX;
    
    //Determinar sentido
    if (output > 0) {
        ledcWrite(CH_RPWM, 0);
        ledcWrite(CH_LPWM, abs_pwm);
    }
    else if (output < 0) {
        ledcWrite(CH_LPWM, 0);
        ledcWrite(CH_RPWM, abs_pwm);
    }
    else {
        ledcWrite(CH_LPWM, 0);
        ledcWrite(CH_RPWM, 0);
    }

    dbg_position = position;
    dbg_error    = error;
    dbg_output   = output;
    dbg_pwm      = abs_pwm;
}

void IRAM_ATTR boton_isr() { control_activo = true; }


// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    uint32_t alarm_ticks = DT * 1000000;

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

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &P_control, true);
    timerAlarmWrite(timer, alarm_ticks, true);
}

void loop() {
    if (control_activo) {
        encoder.setCount(0);           // reinicia posición al presionar
        timerAlarmEnable(timer);
        control_activo = false;
    }

    uint32_t ahora = millis();
    if (ahora - ultimo_serial >= 20) {
        ultimo_serial = ahora;

        Serial.printf(">set:%.2f\n", SETPOINT_PULSOS);
        Serial.printf(">position:%.2f\n", dbg_position);

        Serial.printf("SET:%.2f | pos:%.2f | err:%.2f | out:%.2f | pwm:%d\n",
                SETPOINT_PULSOS, dbg_position, dbg_error, dbg_output, dbg_pwm);
    }
}