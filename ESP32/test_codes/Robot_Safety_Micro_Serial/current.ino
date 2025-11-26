//current.ino
#define VREF 3.3         // Voltaje de referencia de la ESP32
#define ADC_RESOLUTION 4095.0

// Configura según tu modelo
#define SENSITIVITY 0.185  // Sensibilidad en V/A (0.185 para 5A, 0.100 para 20A, 0.066 para 30A)

// Promediar lecturas para mayor estabilidad
#define NUM_MUESTRAS 100



float readCurrent(int GPIO, float offs) {
  float voltage = reedAverage(GPIO);
  float voltageDiff = voltage - offs;  // Diferencia respecto al punto medio
  float current = voltageDiff / SENSITIVITY;  // Corriente en amperios

  /*Serial.print("Voltaje leido: ");
  Serial.print(voltage, 3);
  Serial.print(" V\t Corriente: ");
  Serial.print(current, 3);
  Serial.println(" A");*/

  return current;
}

float reedAverage(int pin) {
  long sum = 0;
  for (int i = 0; i < NUM_MUESTRAS; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  float promedio = (float)sum / NUM_MUESTRAS;
  return (promedio / ADC_RESOLUTION) * VREF;
}

float calibrateSensorCurrent(int GPIO) {
  // Calcula el punto medio (sin corriente)
  //Serial.println("Calibrando ACS712... No conectes ninguna carga.");
  float offs = reedAverage(GPIO);
  /*Serial.print("Voltaje de offset: ");
  Serial.print(offs, 3);
  Serial.println(" V");*/

  return offs;
}
