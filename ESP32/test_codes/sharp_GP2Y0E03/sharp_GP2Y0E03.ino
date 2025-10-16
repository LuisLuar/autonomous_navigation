#include <GP2Y0E03_ESP32.h>

#define SHARP_PIN 33

GP2Y0E03 sensor;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  sensor.init(SHARP_PIN);

  // Calibración personalizada (valores ejemplo)
  // valor_ADC_máx, valor_ADC_mín, distancia_mín, distancia_máx
  Serial.println("Escribe 'auto' para calibrar automáticamente o cualquier otra tecla para usar calibración manual.");
  while (!Serial.available()) delay(10);
  String entrada = Serial.readStringUntil('\n');

  if (entrada.startsWith("auto")) {
    sensor.autoCalibrate();
  } else {
    // Calibración manual (valores de ejemplo)
    sensor.calibrateAnalog(2500, 400, 4, 50);
  }
  Serial.println("Sensor Sharp GP2Y0E03 listo (modo analógico)"); 
  
}

void loop() {
  Serial.print("Distancia (cm): ");
  Serial.println(sensor.distAnalog());  
  delay(1000);
}
