#include <Arduino.h>


hw_timer_t * timer = NULL;
const int ledPin = 2; 
volatile bool estadoLed = false;


void IRAM_ATTR onTimer() {
  estadoLed = !estadoLed;
  digitalWrite(ledPin, estadoLed);
}

void setup() {
  pinMode(ledPin, OUTPUT);

  /* 1. Iniciar timer 
     0 = número de timer (0 a 3)
     80 = prescaler (80MHz / 80 = 1 tick por microsegundo)
     true = contar hacia arriba
  */
  timer = timerBegin(0, 80, true);

  /* 2. Adjuntar la función onTimer a nuestro timer */
  timerAttachInterrupt(timer, &onTimer, true);

  /* 3. Configurar la alarma
     1,000,000 ticks = 1 segundo
     true = que se repita (autoreload)
  */
  timerAlarmWrite(timer, 500000, true); // 500ms para el efecto metrónomo

  /* 4. Activar la alarma */
  timerAlarmEnable(timer);
}

void loop() {

}