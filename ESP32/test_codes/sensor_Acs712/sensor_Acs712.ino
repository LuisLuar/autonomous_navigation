#define SENSOR_PIN 34    // Entrada analógica de la ESP32 (32, 33, 34, 35, 36 o 39)
float current = 0.0;
float offset = 0;  // Voltaje de salida en reposo (sin corriente)

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  delay(1000);

  offset = calibrateSensorCurrent(SENSOR_PIN);
  
}

void loop() {
  current = readCurrent(SENSOR_PIN, offset);

  

  delay(500);
}
