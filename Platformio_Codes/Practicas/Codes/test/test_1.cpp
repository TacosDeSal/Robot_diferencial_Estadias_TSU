#include <Arduino.h>
#include "soc/gpio_reg.h"
#include "soc/ledc_reg.h"

// ── Encoder ───────────────────────────────────────────────
#define ENC_A  18
#define ENC_B  19
// int32_t — lectura/escritura atómica en Xtensa LX6 (32-bit aligned)
// int64_t requiere dos instrucciones = no atómica = fuente de crash
volatile int32_t enc_count = 0;

// ── BTS7960 ───────────────────────────────────────────────
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
#define SETPOINT_PPS  1895
const float DT    = 0.100f;
const float KP    = 0.08f;
const float KI    = 2.5f;
const float I_MAX =  2000.0f;
const float I_MIN = -2000.0f;

// ── Estado del controlador (exclusivo de la ISR del PI) ───
// Nadie más los toca → sin necesidad de lock
static int32_t last_count   = 0;
static float   integral_acc = 0.0f;

// ── Flags inter-ISR ───────────────────────────────────────
// bool = 1 byte, escritura/lectura atómica en Xtensa
volatile bool control_habilitado = false;
volatile bool reset_pendiente    = false;

// ── Debug (escritos en ISR, leídos en loop) ───────────────
// float y int32_t: 4 bytes alineados = lectura atómica en ESP32
volatile float   dbg_vel = 0.0f;
volatile float   dbg_err = 0.0f;
volatile float   dbg_int = 0.0f;
volatile int32_t dbg_pwm = 0;

hw_timer_t *timer        = NULL;
uint32_t    ultimo_serial = 0;


// ════════════════════════════════════════════════════════════
//  LEDC — escritura directa a registros hardware
//
//  Mapa canales High-Speed (0-7):
//    base = LEDC_HSCH0_CONF0_REG + canal * 0x14
//    +0x00  CONF0  bit2  → sig_out_en
//    +0x08  DUTY         → valor en Q10.4 (duty << 4)
//    +0x0C  CONF1  bit31 → duty_start (auto-clear tras ciclo)
//
//  REG_WRITE / REG_SET_BIT son macros de ESP-IDF que expanden
//  a un store de 32 bits directo — nunca tocan Flash.
// ════════════════════════════════════════════════════════════
static inline void IRAM_ATTR ledc_write_isr(uint8_t canal, uint32_t duty) {
    const uint32_t base = LEDC_HSCH0_CONF0_REG + canal * 0x14u;
    REG_WRITE  (base + 0x08u, duty << 4);   // carga duty
    REG_SET_BIT(base,         BIT(2));       // sig_out_en
    REG_SET_BIT(base + 0x0Cu, BIT(31));      // dispara actualización
}


// ════════════════════════════════════════════════════════════
//  ISRs del encoder
// ════════════════════════════════════════════════════════════
void IRAM_ATTR enc_A_isr() {
    const uint32_t gpio = REG_READ(GPIO_IN_REG);
    const bool a = (gpio >> ENC_A) & 1u;
    const bool b = (gpio >> ENC_B) & 1u;
    enc_count += (a == b) ? 1 : -1;
}

void IRAM_ATTR enc_B_isr() {
    const uint32_t gpio = REG_READ(GPIO_IN_REG);
    const bool a = (gpio >> ENC_A) & 1u;
    const bool b = (gpio >> ENC_B) & 1u;
    enc_count += (a == b) ? -1 : 1;
}


// ════════════════════════════════════════════════════════════
//  ISR del PI — 100 ms
//  El loop NO interviene en el camino de control
// ════════════════════════════════════════════════════════════
void IRAM_ATTR PI_control() {

    // Reset pedido por botón — se maneja aquí, no en loop
    if (reset_pendiente) {
        integral_acc      = 0.0f;
        last_count        = enc_count;
        control_habilitado = true;
        reset_pendiente    = false;
    }

    if (!control_habilitado) return;

    // enc_count es int32_t → lectura en una sola instrucción (atómica)
    const int32_t c   = enc_count;
    const float   vel = (float)(c - last_count) / DT;
    last_count = c;

    const float error = (float)SETPOINT_PPS - vel;

    integral_acc += error * DT;
    if (integral_acc >  I_MAX) integral_acc =  I_MAX;
    if (integral_acc <  I_MIN) integral_acc =  I_MIN;

    const float output = KP * error + KI * integral_acc;

    int32_t pwm = (int32_t)output;
    if (pwm < PWM_MIN) pwm = PWM_MIN;
    if (pwm > PWM_MAX) pwm = PWM_MAX;

    ledc_write_isr(CH_LPWM, (uint32_t)pwm);

    // Debug — floats de 4 bytes alineados: escritura atómica en LX6
    dbg_vel = vel;
    dbg_err = error;
    dbg_int = integral_acc;
    dbg_pwm = pwm;
}


// ════════════════════════════════════════════════════════════
//  ISR del botón — solo pone el flag, PI_control hace el resto
// ════════════════════════════════════════════════════════════
void IRAM_ATTR boton_isr() {
    reset_pendiente = true;
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

    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A), enc_A_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_B), enc_B_isr, CHANGE);

    pinMode(PIN_BOTON, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_BOTON), boton_isr, RISING);

    // Timer siempre activo — PI_control decide internamente si actúa
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &PI_control, true);
    timerAlarmWrite(timer, 100000, true);
    timerAlarmEnable(timer);
}


// ════════════════════════════════════════════════════════════
//  Loop — cero lógica de control, solo telemetría
// ════════════════════════════════════════════════════════════
void loop() {
    const uint32_t ahora = millis();
    if (ahora - ultimo_serial >= 100) {
        ultimo_serial = ahora;

        // float/int32_t alineados = lecturas atómicas, no necesitan lock
        Serial.printf("SET:%d | vel:%.2f | err:%.2f | I:%.4f | pwm:%d\n",
                      SETPOINT_PPS,
                      (float)dbg_vel,
                      (float)dbg_err,
                      (float)dbg_int,
                      (int)dbg_pwm);
    }
}