#include "SabertoothSimplified.h"

SabertoothSimplified::SabertoothSimplified(Stream &port) : serial(port) {}

void SabertoothSimplified::motor(int motor, int power) {
  if (motor < 1 || motor > 2) return;
  power = constrain(power, -127, 127);

  byte command;
  if (motor == 1)
    command = (power >= 0) ? 0 : 1; // 0 = M1 forward, 1 = M1 reverse
  else
    command = (power >= 0) ? 4 : 5; // 4 = M2 forward, 5 = M2 reverse

  byte speed = abs(power);
  sendByte(command + speed);
}

void SabertoothSimplified::sendByte(byte value) {
  serial.write(value);
}

