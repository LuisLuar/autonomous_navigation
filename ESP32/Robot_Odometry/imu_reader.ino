//imu_reader.ino
/*#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 imu;

// Inicializa offsets a 0 (los calcularás)
float accX_offset = 0.0;
float accY_offset = 0.0;
float accZ_offset = 0.0;

float gyroX_offset = 0.0;
float gyroY_offset = 0.0;
float gyroZ_offset = 0.0;

const int samples = 500;

bool IMU_begin() {
  Wire.begin();
  Wire.setClock(400000);

  // Intentar inicializar el IMU varias veces
  const int maxAttempts = 5;
  int attempts = 0;
  while (!imu.begin()) {
    attempts++;
    if (attempts >= maxAttempts) {
      Serial.println("IMU: init failed after multiple attempts.");
      return false;
    }
    Serial.println("IMU: init failed, retrying...");
    delay(200);
  }

  imu.setAccelerometerRange(MPU6050_RANGE_2_G);
  imu.setGyroRange(MPU6050_RANGE_250_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Calibración: promediar N muestras en reposo
  Serial.println("Calibrando IMU... No muevas el sensor.");
  double sumAx = 0.0, sumAy = 0.0, sumAz = 0.0;
  double sumGx = 0.0, sumGy = 0.0, sumGz = 0.0;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    imu.getEvent(&a, &g, &temp);
    sumAx += a.acceleration.x;
    sumAy += a.acceleration.y;
    sumAz += a.acceleration.z; // incluye gravedad en Z si el sensor está plano

    sumGx += g.gyro.x;
    sumGy += g.gyro.y;
    sumGz += g.gyro.z;

    delay(5); // pequeño retardo para no saturar bus I2C
  }

  accX_offset = sumAx / samples;
  accY_offset = sumAy / samples;
  accZ_offset = sumAz / samples;

  gyroX_offset = sumGx / samples;
  gyroY_offset = sumGy / samples;
  gyroZ_offset = sumGz / samples;

  // Si quieres que accZ_offset considere gravedad -> si tu algoritmo espera 0 m/s^2 en Z,
  // debes restar 9.80665 m/s^2 en función de la orientación del sensor. Si el sensor está plano:
  // accZ_offset -= 9.80665;

  Serial.println("Calibracion IMU completada.");
  return true;
}

bool IMU_update() {
  sensors_event_t a, g, temp;
  imu.getEvent(&a, &g, &temp);

  // Resta offsets (medidos) para centrar lecturas
  ax = a.acceleration.x - accX_offset;
  ay = a.acceleration.y - accY_offset;
  az = a.acceleration.z - accZ_offset;

  gx = g.gyro.x - gyroX_offset;
  gy = g.gyro.y - gyroY_offset;
  gz = g.gyro.z - gyroZ_offset;

  // Filtro sencillo deadzone
  if (abs(ax) < 0.07) ax = 0;
  if (abs(ay) < 0.07) ay = 0;
  if (abs(az) < 0.07) az = 0;

  if (abs(gx) < 0.3) gx = 0;
  if (abs(gy) < 0.3) gy = 0;
  if (abs(gz) < 0.05) gz = 0;

  // Integración simple (ten en cuenta acumulación de drift)
  roll_imu  += gx * (dt_imu / 1000.0);
  pitch_imu += gy * (dt_imu / 1000.0);
  yaw_imu   += gz * (dt_imu / 1000.0);

  return true;
}*/
