//imu_reader.ino

// ---- Valores de calibración del magnetómetro (segunda calibración, en mG) ----
const float MAG_BIAS_X = 205.65; //203.85; //137.1f;
const float MAG_BIAS_Y = 334.81; //342.05; //327.58f;
const float MAG_BIAS_Z = -577.31; //-575.58; //-572.11;

const float MAG_SCALE_X = 1.81; //1.83; //2.53f;
const float MAG_SCALE_Y = 1.11; //1.09; //1.06f;
const float MAG_SCALE_Z = 0.65; //0.6f;

// Declinación magnética
// -5° 4'  => -5.0666667 grados (negativa = WEST)
const float MAG_DECLINATION_DEG = -5.0666667f;

// conversion constants
const float G_TO_MS2 = 9.80665f;
bool first = false;
float yaw_gyro = 0.0;
float yaw_integral = 0.0;
float error = 0.0;
static float yaw_bias = 0.0f;
static uint32_t last_correction = 0;
float k = 0.0;

// Si el pitch es negativo (robot inclinado hacia adelante) y estamos en el sentido contrario,
// ajustar el yaw en 180°
float threshold_pitch = 0.1f; // radianes, ajusta según tu robot

// Solo aplicar esta corrección al inicio
static bool first_orientation_set = false;

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

  delay(500);
  mpu.calibrateMag();
  // ---- Aplicar valores preestablecidos del MAGNETÓMETRO (no recalibramos cada inicio) ----
  //Serial.println("Applying preset magnetometer bias/scale from saved calibration...");
  mpu.setMagBias(MAG_BIAS_X, MAG_BIAS_Y, MAG_BIAS_Z);   // unidades: mG (tal como tu salida)
  mpu.setMagScale(MAG_SCALE_X, MAG_SCALE_Y, MAG_SCALE_Z);

  // ---- Declinación magnética (para convertir heading magnético -> verdadero) ----
  //mpu.setMagneticDeclination(MAG_DECLINATION_DEG);

  // ---- Configurar filtro de orientación (usar cuaternión AHRS) ----
  mpu.selectFilter(QuatFilterSel::MADGWICK);
  mpu.setFilterIterations(5);
  mpu.ahrs(false);              // asegurar que AHRS está activo


  // Desactivar verbose para normal operation
  mpu.verbose(false);

  //Serial.println("Calibracion IMU completada.");
  return true;
}

void IMU_update() {

  float magx_raw = mpu.getMagX();
  float magy_raw = mpu.getMagY();
  float magz_raw = mpu.getMagZ();

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

  if (abs(gx) < 0.0025) gx = 0;
  if (abs(gy) < 0.0025) gy = 0;
  if (abs(gz) < 0.0025) gz = 0;

  //__________ORIENTACION (Ángulos de Euler) rad_____________
  yaw_integral += gz * dt_imu / 1000;
  yaw_gyro = yaw_integral;
  while (yaw_gyro >= PI) yaw_gyro -= 2 * PI; //3.1415926535f
  while (yaw_gyro < -PI) yaw_gyro += 2 * PI;


  float yaw_mag = atan2(-magy_raw, magx_raw) + MAG_DECLINATION_DEG * DEG_TO_RAD; //- PI/2 ;

  float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az));
  
  if (!first_orientation_set && abs(pitch_acc) > threshold_pitch) {
    // Determinar la dirección basada en el signo del pitch
    if (pitch_acc > 0) {
      // Robot inclinado hacia atrás
      yaw_mag += PI; // Ajustar 180 grados
    }
    first_orientation_set = true;
  }

  while (yaw_mag >  PI) yaw_mag -= 2 * PI;
  while (yaw_mag < -PI) yaw_mag += 2 * PI;

  float mag_norm = sqrt(magx_raw * magx_raw + magy_raw * magy_raw + magz_raw * magz_raw);
  bool mag_ok = (mag_norm > 300 && mag_norm < 600);

  if (mag_ok && (millis() - last_correction) > 500)
  {
    if (abs(vx) < 0.05 && abs(gz) < 0.05) {
      k = 0.009f;
    } else {
      k = 0.007f;
    }

    error = yaw_mag - yaw_imu;

    // normalizar error
    while (error >  PI) error -= 2 * PI;
    while (error < -PI) error += 2 * PI;

    yaw_bias += k * error;   // 🔑 corrección MUY suave
    last_correction = millis();

    if (!first) {
      yaw_integral = yaw_mag;   // sincronizar estado continuo
      yaw_bias = 0.0f;
      first = true;
    }
  }

  yaw_imu = yaw_gyro + yaw_bias;
  while (yaw_imu >  PI) yaw_imu -= 2 * PI;
  while (yaw_imu < -PI) yaw_imu += 2 * PI;

  roll_imu  = -mpu.getRoll() * DEG_TO_RAD;
  pitch_imu = -mpu.getPitch() * DEG_TO_RAD;
  //yaw_imu = yaw_deg * DEG_TO_RAD;
}
