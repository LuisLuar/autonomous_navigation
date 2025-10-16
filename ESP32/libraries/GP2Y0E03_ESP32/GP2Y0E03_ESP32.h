#ifndef GP2Y0E03_ESP32_H
#define GP2Y0E03_ESP32_H

#include <Arduino.h>
#include <Wire.h>

class GP2Y0E03 {
public:
  GP2Y0E03(uint8_t address = 0x40);
  void init(int vout = -1);

#if defined(ESP32) || defined(ESP8266)
  void init(int vout, int sda, int scl);
  void init(int sda, int scl);
#endif

  int distAnalog();
  int vout();
  void calibrateAnalog(int voutMin, int voutMax, int distMin, int distMax);
  void autoCalibrate(); // 🔹 Nueva función

private:
  int _addr;
  int _vout;
  int _cal[4];
  void _init_sensor(int vout);
};

#endif

