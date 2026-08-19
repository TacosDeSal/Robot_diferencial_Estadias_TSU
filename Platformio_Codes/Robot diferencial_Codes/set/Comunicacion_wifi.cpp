#include <WiFi.h>
#include <ArduinoOTA.h>

const char* ssid = "TU_RED";
const char* password = "TU_PASSWORD";

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname("robot-esp32");

  ArduinoOTA.onStart([]() {
    Serial.println("Iniciando OTA...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA completo");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progreso: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]\n", error);
  });

  ArduinoOTA.begin();
  Serial.println("OTA listo");
}

void loop() {
  ArduinoOTA.handle();
}