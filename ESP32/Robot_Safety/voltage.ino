//voltage.ino
#define ADC_RESOLUTION 4095.0
#define VREF 3.3         // Voltaje de referencia de la ESP32
// El módulo FZ0430 tiene una relación de 5:1
// Es decir, 25V de entrada → 5V de salida
// pero como la ESP32 solo soporta hasta 3.3V, el rango útil es ~16.5V máx.
#define DIVIDER_RATIO (25.0 / 5.0)  // = 5.0

float readVoltage(int GPIO, float alpha , float &anterior){
  int rawValue = analogRead(GPIO);
  float voltageOut = (rawValue / ADC_RESOLUTION) * VREF;     // Voltaje que ve la ESP32 (0–3.3V)
  float SinFilter_voltageIn = voltageOut * DIVIDER_RATIO;              // Voltaje real medido (0–16.5V aprox)

  
  float voltage = alpha * SinFilter_voltageIn + (1-alpha) * anterior;
  anterior = voltage;

  /*Serial.print("ADC: ");
  Serial.print(rawValue);
  Serial.print("\t Voltaje sensor (salida): ");
  Serial.print(voltageOut, 3);
  Serial.print(" V\t Voltaje real: ");
  Serial.print(voltageIn, 2);
  Serial.println(" V");*/
  return voltage;
}
