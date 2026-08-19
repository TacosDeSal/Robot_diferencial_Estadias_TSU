#include <Arduino.h>
#include "driver/pcnt.h"    // driver de bajo nivel de Espressif para el módulo PCNT

// ─────────────────────────────────────────────────────────────
// El módulo PCNT del ESP32 tiene:
//   • 8 unidades  (PCNT_UNIT_0 … PCNT_UNIT_7)
//   • 2 canales por unidad (PCNT_CHANNEL_0, PCNT_CHANNEL_1)
//   • Contador interno de 16 bits con signo  → rango: -32768 a +32767
//   • Cada canal tiene:
//       - pulse_gpio  : pin que genera los pulsos a contar
//       - ctrl_gpio   : pin de control que puede invertir/detener el conteo
// ─────────────────────────────────────────────────────────────

#define ENC_A  19   // canal A del encoder → pulse_gpio del canal 0
#define ENC_B  18   // canal B del encoder → pulse_gpio del canal 1

// Unidad PCNT que vamos a usar (hay 8 disponibles: PCNT_UNIT_0 … PCNT_UNIT_7)
#define PCNT_UNIT   PCNT_UNIT_0


volatile int64_t count = 0;
uint32_t last_print = 0;

// ── Cuadratura completa (x4) ──────────────────────────────────
// En cuadratura completa se cuentan los 4 flancos por ciclo:
//
//   Canal A:  ─┐ ┌─┐ ┌─    Canal B:  ──┐ ┌─┐ ┌─
//              └─┘ └─┘                  └─┘ └─┘
//   Flancos:  ↑↓↑↓  (A)  +  ↑↓↑↓  (B)  = 4 conteos por ciclo
//
// Para lograr esto se configuran DOS canales en la misma unidad:
//   Canal 0: cuenta flancos de A, controlado por B
//   Canal 1: cuenta flancos de B, controlado por A
// ─────────────────────────────────────────────────────────────

void setup_pcnt() {

    // ── Configuración del Canal 0 ─────────────────────────────
    // pcnt_config_t es un struct de ESP-IDF con todos los parámetros del canal
    pcnt_config_t canal0 = {

        .pulse_gpio_num = ENC_A,
        // GPIO que se monitorea para contar pulsos (canal A del encoder)

        .ctrl_gpio_num  = ENC_B,
        // GPIO de control: su estado (HIGH/LOW) decide si el contador
        // sube o baja cuando llega un flanco en pulse_gpio

        .lctrl_mode = PCNT_MODE_REVERSE,
        // Cuando ctrl_gpio está en LOW → invierte la dirección del conteo
        // Es decir: si normalmente sube, ahora baja (y viceversa)
        // Esto es lo que decodifica la dirección de giro del encoder

        .hctrl_mode = PCNT_MODE_KEEP,
        // Cuando ctrl_gpio está en HIGH → mantiene la dirección normal
        // PCNT_MODE_KEEP  = no cambia nada
        // PCNT_MODE_REVERSE = invierte
        // PCNT_MODE_DISABLE = no cuenta

        .pos_mode = PCNT_COUNT_INC,
        // Qué hacer en flanco POSITIVO (LOW→HIGH) de pulse_gpio
        // PCNT_COUNT_INC  = incrementa el contador
        // PCNT_COUNT_DEC  = decrementa
        // PCNT_COUNT_DIS  = ignora este flanco

        .neg_mode = PCNT_COUNT_DEC,
        // Qué hacer en flanco NEGATIVO (HIGH→LOW) de pulse_gpio
        // Contando ambos flancos (INC + DEC con ctrl) = cuadratura x2 por canal
        // Combinado con el canal 1 = cuadratura x4 total

        .counter_h_lim =  32767,
        // Límite superior del contador de 16 bits
        // Si llega aquí dispara un evento y puede reiniciar (overflow)
        // 32767 = máximo positivo de int16_t

        .counter_l_lim = -32768,
        // Límite inferior del contador
        // -32768 = mínimo negativo de int16_t (underflow)

        .unit    = PCNT_UNIT,
        // A qué unidad PCNT pertenece esta configuración (PCNT_UNIT_0)

        .channel = PCNT_CHANNEL_0,
        // Canal dentro de la unidad (canal 0 de 2 disponibles)
    };

    pcnt_unit_config(&canal0);
    // Aplica la configuración del canal 0 al hardware PCNT
    // Internamente escribe los registros del módulo PCNT en el ESP32


    // ── Configuración del Canal 1 ─────────────────────────────
    // Canal 1 hace lo mismo pero con A y B intercambiados
    // Esto permite contar los flancos de B controlado por A
    // Resultado: 4 conteos por ciclo mecánico completo (x4)

    pcnt_config_t canal1 = {

        .pulse_gpio_num = ENC_B,
        // Ahora monitoreamos el canal B para los pulsos

        .ctrl_gpio_num  = ENC_A,
        // Y usamos A como control de dirección

        .lctrl_mode = PCNT_MODE_KEEP,
        // Cuando A está en LOW → mantiene dirección normal
        // (invertido respecto al canal 0 para que ambos cuenten en la misma dirección)

        .hctrl_mode = PCNT_MODE_REVERSE,
        // Cuando A está en HIGH → invierte la dirección
        // La lógica invertida respecto al canal 0 es intencional
        // para que los dos canales siempre sumen en la misma dirección

        .pos_mode = PCNT_COUNT_INC,
        // Flanco positivo de B → incrementa

        .neg_mode = PCNT_COUNT_DEC,
        // Flanco negativo de B → decrementa

        .counter_h_lim =  32767,
        .counter_l_lim = -32768,

        .unit    = PCNT_UNIT,
        // Misma unidad que el canal 0 — comparten el mismo contador físico

        .channel = PCNT_CHANNEL_1,
        // Segundo canal de la unidad
    };

    pcnt_unit_config(&canal1);
    // Aplica la configuración del canal 1


    // ── Filtro de rebote ──────────────────────────────────────
    pcnt_set_filter_value(PCNT_UNIT, 100);
    // Establece el filtro de glitch en ciclos de clock (80MHz)
    // 100 ciclos = 100 / 80,000,000 = 1.25 µs
    // Pulsos más cortos que esto se ignoran → elimina rebotes y ruido eléctrico
    // Rango válido: 0 a 1023 ciclos
    // Para encoders lentos puedes subir a 1000 (12.5µs)

    pcnt_filter_enable(PCNT_UNIT);
    // Activa el filtro — sin esta línea pcnt_set_filter_value no tiene efecto

    // Registrar ISR de overflow
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, pcnt_overflow_handler, NULL);

    // Habilitar eventos en los límites
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);  // dispara en +32,767
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);  // dispara en -32,768

    // ── Inicializar y arrancar ────────────────────────────────
    pcnt_counter_pause(PCNT_UNIT);
    // Pausa el contador antes de limpiarlo — buena práctica para evitar
    // leer valores basura durante la inicialización

    pcnt_counter_clear(PCNT_UNIT);
    // Pone el contador a 0
    // Equivalente a encoder.setCount(0) de ESP32Encoder

    pcnt_counter_resume(PCNT_UNIT);
    // Arranca el contador — a partir de aquí empieza a contar pulsos
}


// ── Leer el contador — seguro desde ISR ──────────────────────
// Esta función SÍ puede llamarse desde una ISR con IRAM_ATTR
// porque pcnt_get_counter_value() accede directo al registro de hardware

void IRAM_ATTR pcnt_overflow_handler(void *arg) {
    int16_t valor = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &valor);
    count += (int64_t)valor;   // acumula aquí — único lugar
    pcnt_counter_clear(PCNT_UNIT_0);
}


void setup() {
    Serial.begin(115200);
    setup_pcnt();
}


void loop() {

    uint32_t now = millis();
    if (now - last_print >= 10) {
        last_print = now;
        Serial.printf("pos: %lld\n", count);
    }
}