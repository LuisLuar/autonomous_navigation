//imu_reader.ino

// ---- Valores de calibración del magnetómetro (segunda calibración, en mG) ----
const float MAG_BIAS_X = 295.85f;
const float MAG_BIAS_Y = 537.51f;
const float MAG_BIAS_Z = 199.37f;

const float MAG_SCALE_X = 1.23f;
const float MAG_SCALE_Y = 0.67f;
const float MAG_SCALE_Z = 1.43f;

// Declinación magnética
// -5° 4'  => -5.0666667 grados (negativa = WEST)
const float MAG_DECLINATION_DEG = -5.0666667f;

// conversion constants
const float G_TO_MS2 = 9.80665f;


bool IMU_begin() {
  Wire.begin();
  delay(2000);
  //Wire.setClock(400000);

  // ---- Configuración sugerida para robot móvil en vias (recomendación) ----
  // Explicación: para robots en vía, no esperamos G's enormes ni giros extremadamente rápidos.
  // Elegimos sensibilidad moderada para buena resolución y filtros DLPF para estabilidad.
  MPU9250Setting setting;
  // Recomendación: +-8g para aceleración (suficiente para baches fuertes) o +-4g si quieres mayor resolución.
  setting.accel_fs_sel = ACCEL_FS_SEL::A8G;        // <-- mejor resolución/ruido vs A16G
  setting.gyro_fs_sel  = GYRO_FS_SEL::G500DPS;    // <-- 500 dps es conservador; G250DPS es más sensible si no esperas giros fuertes
  setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;    // amortigua ruido en giros rápidos
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;  // suaviza vibraciones, mantiene respuesta decente

  // Intentar inicializar el IMU varias veces
  const int maxAttempts = 5;
  int attempts = 0;
  while (!mpu.setup(0x68, setting)) {
    attempts++;
    if (attempts >= maxAttempts) {
      //Serial.println("IMU: init failed after multiple attempts.");
      return false;
    }
    //Serial.println("IMU: init failed, retrying...");
    delay(200);
  }

  // Habilitar mensajes verbosos en calibración
  mpu.verbose(true);

  // ---- Calibración de ACCEL + GYRO en el arranque (se ejecuta SIEMPRE) ----
  //Serial.println("Accel/Gyro calibration will start in 5 sec. Keep the device still.");
  delay(500);
  //Serial.println("Calibrating accel+gyro ... (mantener el dispositivo quieto)");
  mpu.calibrateAccelGyro(); // bloquea hasta terminar
  //Serial.println("Accel+Gyro calibration finished.");

  // ---- Aplicar valores preestablecidos del MAGNETÓMETRO (no recalibramos cada inicio) ----
  //Serial.println("Applying preset magnetometer bias/scale from saved calibration...");
  mpu.setMagBias(MAG_BIAS_X, MAG_BIAS_Y, MAG_BIAS_Z);   // unidades: mG (tal como tu salida)
  mpu.setMagScale(MAG_SCALE_X, MAG_SCALE_Y, MAG_SCALE_Z);

  // ---- Declinación magnética (para convertir heading magnético -> verdadero) ----
  mpu.setMagneticDeclination(MAG_DECLINATION_DEG);

  // ---- Configurar filtro de orientación (usar cuaternión AHRS) ----
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(10); // sugerido por ti
  mpu.ahrs(true);              // asegurar que AHRS está activo

  // Desactivar verbose para normal operation
  mpu.verbose(false);

  //Serial.println("Calibracion IMU completada.");
  return true;
}

void IMU_update() {

  /*float magx_raw = mpu.getMagX();
  float magy_raw = mpu.getMagY();
  float magz_raw = mpu.getMagZ();*/

  //______________ACELERACION m/s2____________
  // Aceleración (ya escalada según fullscale): getAccX/Y/Z()
  ax = mpu.getAccX() * G_TO_MS2;
  ay = mpu.getAccY() * G_TO_MS2;
  az = mpu.getAccZ() * G_TO_MS2;

  /*if (abs(ax) < 0.07) ax = 0;
    if (abs(ay) < 0.2) ay = 0;
    if (abs(az) < 0.07) az = 0;*/

  //accX = alpha * ax_uf + (1.0 - alpha) * accX;

  //__________VELOCIDAD ANGULAR en rad/s_____________
  gx = mpu.getGyroX() * DEG_TO_RAD;
  gy  = mpu.getGyroY() * DEG_TO_RAD;
  gz  = mpu.getGyroZ() * DEG_TO_RAD;

  /*if (abs(gx) < 0.3) gx = 0;
    if (abs(gy) < 0.3) gy = 0;
    if (abs(gz) < 0.05) gz = 0;*/

  //__________ORIENTACION (Ángulos de Euler) rad_____________
  float yaw_deg = (-mpu.getYaw() + 90.0f); // ya con declinación aplicada

  // normalizar a [-180,180)
  while (yaw_deg >= 180.0f) yaw_deg -= 360.0f;
  while (yaw_deg < -180.0f) yaw_deg += 360.0f;

  roll_imu  = -mpu.getRoll() * DEG_TO_RAD;
  pitch_imu = -mpu.getPitch() * DEG_TO_RAD;
  yaw_imu = yaw_deg * DEG_TO_RAD;

  //Serial.printf("Acc [m/s^2]: %.4f, %.4f, %.4f\n", ax, ay, az);
  //Serial.printf("Mag: %.4f, %.4f, %.4f\n", magx_raw, magy_raw, magz_raw);
  //Serial.printf("Gyro [rad/s]: %.4f, %.4f, %.4f\n", gx, gy, gz);
  //Serial.printf("roll %.2f pitch %.2f yaw %.2f\n", roll_imu, pitch_imu, yaw_imu);
}
