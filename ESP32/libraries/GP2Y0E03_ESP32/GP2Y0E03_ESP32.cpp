#include "GP2Y0E03_ESP32.h"

GP2Y0E03::GP2Y0E03(uint8_t address) : _addr(address) {}

void GP2Y0E03::init(int vout) {
  Wire.begin();
  _init_sensor(vout);
}

#if defined(ESP32) || defined(ESP8266)
void GP2Y0E03::init(int vout, int sda, int scl) {
  Wire.begin(sda, scl);
  _init_sensor(vout);
}

void GP2Y0E03::init(int sda, int scl) {
  Wire.begin(sda, scl);
  _init_sensor(-1);
}
#endif

void GP2Y0E03::_init_sensor(int vout) {
  _vout = vout;
  calibrateAnalog(448, 289, 3, 30); // valores por defecto
  if (_vout != -1) {
    pinMode(_vout, INPUT);
  }
}

int GP2Y0E03::vout() {
  return analogRead(_vout);
}

void GP2Y0E03::calibrateAnalog(int voutMin, int voutMax, int distMin, int distMax) {
  _cal[0] = voutMin;
  _cal[1] = voutMax;
  _cal[2] = distMin;
  _cal[3] = distMax;

  // 🔹 Mostrar valores al calibrar
  Serial.println(F("\n--- Calibración manual ---"));
  Serial.print(F("ADC Mínimo: ")); Serial.println(voutMin);
  Serial.print(F("ADC Máximo: ")); Serial.println(voutMax);
  Serial.print(F("Distancia Mínima: ")); Serial.print(distMin); Serial.println(F(" cm"));
  Serial.print(F("Distancia Máxima: ")); Serial.print(distMax); Serial.println(F(" cm"));
  Serial.println(F("--------------------------\n"));
}

int GP2Y0E03::distAnalog() {
  if (_vout == -1) return -1;
  int lectura = analogRead(_vout);
  int dist = map(lectura, _cal[0], _cal[1], _cal[2], _cal[3]);
  dist = constrain(dist, _cal[2], _cal[3]);
  return dist;
}

void GP2Y0E03::autoCalibrate() {
  if (_vout == -1) {
    Serial.println(F("⚠️ No hay pin analógico configurado para el sensor."));
    return;
  }

  Serial.println(F("\n=== AUTO-CALIBRACIÓN DEL SENSOR ==="));
  Serial.println(F("1️⃣ Coloca un objeto a 4 cm del sensor y presiona Enter."));
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read(); // limpiar buffer
  delay(500);

  int lecturaCorta = analogRead(_vout);
  Serial.print(F("Lectura a 4 cm: "));
  Serial.println(lecturaCorta);

  Serial.println(F("\n2️⃣ Coloca el objeto a 50 cm del sensor y presiona Enter."));
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();
  delay(500);

  int lecturaLarga = analogRead(_vout);
  Serial.print(F("Lectura a 50 cm: "));
  Serial.println(lecturaLarga);

  // Guardar calibración
  calibrateAnalog(lecturaLarga, lecturaCorta, 50, 4);

  Serial.println(F("\n✅ Calibración completada exitosamente."));
  Serial.println(F("--------------------------------------\n"));
}

