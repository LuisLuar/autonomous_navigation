// Robot_Odometry.ino (PRINCIPAL)
#include <imu_publisher.h>
#include <odometry.h>
#include <ESP32Encoder.h>
#include <SabertoothSimplified.h>
#include <motorControl.h>
#include <GP2Y0E03_ESP32.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <Wire.h>
#include "MPU9250.h"

MPU9250 mpu;
SemaphoreHandle_t i2cMutex;



//___________ENCODER DERECHO_____________
#define RightA 27
#define RightB 14
#define RightZ 26
//_______________________________________

//__________ENCODER LeftUIERDO____________
#define LeftA 5
#define LeftB 18
#define LeftZ 19
//_______________________________________

//______________SENSORES SHARP_____________
#define SHARP_PIN1 33
#define SHARP_PIN2 34
#define SHARP_PIN3 35
//_______________________________________

//__________________CONTROL DE MOTORES______________________
#define SABERTOOTH_TX 17   // GPIO17 (TX2)
HardwareSerial SabertoothSerial(2);
SabertoothSimplified ST(SabertoothSerial); //El rango de velocidad es de -63 a +63, donde:
//_______________________________________

//_____________cariables para seguridad de microros y serial______________
unsigned long last_serial_activity = 0;
const unsigned long SERIAL_TIMEOUT_MS = 2000; // 2 segundos sin actividad serial
bool serial_connected = false;
bool reset = false;
//_____________________________________________________________________

//___________Parametros del robot_____________
float L = 0.84; // distancia entre ruedas metros
float D = 0.33; // Diametro de la rueda en metros
//_______________________________________________

//_______VARIABLES IMU_________________________________
const int sampleTime_imu = 50;  // Tiempo de muestreo milisegundos
unsigned long lastTime_imu = 0, dt_imu = 0;

float ax = 0.0, ay = 0.0, az = 0.0;
float gx = 0.0, gy = 0.0, gz = 0.0;
float roll_imu = 0.0, pitch_imu = 0.0, yaw_imu = 0.0;
float offset = 0.0;
//______________________________________________________

//________CONFIGURACION ENCODER__________
const int sampleTime_enc = 50;  // Tiempo de muestreo milisegundos
unsigned long lastTime_enc = 0, dt_enc = 0;;

float vx = 0, wz = 0; //Velocidad lineal y angular del robot [m/s] y [rad/s]
float x_pos = 0.0, y_pos = 0.0; //Posición estimada por odometría METROS
float x_pose = 0.0, y_pose = 0.0;
float yaw_enc = 0.0; //Orientación estimada por odometría RADIANES
//_______________________________________

//________RANGO DE DISTANCIAS_____________
const int sampleTime_range = 200;
unsigned long lastTime_range = 0;
float range_front = 0.0, range_left = 0.0, range_right = 0.0;
//_______________________________________

//________TIEMPO SERIAL___________________
const int sampleTime_serial = 50;  // Tiempo de muestreo milisegundos
unsigned long lastTime_serial = 0;
//_______________________________________

//______________ESTRUCTURA PARA DATOS MOTORES___________________________
struct DatosMotores {
  double w = 0.0;
  double wAnterior = 0.0;
  double wRef = 0.0;
  int outValue = 0;
  double wSinFilter = 0.0;
  double v = 0.0;
};
//_________________________________________________________

//____________creación de instancias__________________________
DatosMotores Left;
DatosMotores Right;

Odometry odometry;
ImuPublisher imu_pub;

ESP32Encoder encoderL;
ESP32Encoder encoderR;

GP2Y0E03 sensor1;
GP2Y0E03 sensor2;
GP2Y0E03 sensor3;

motorControl motorR(sampleTime_enc);
motorControl motorL(sampleTime_enc);
//_________________________________________________________

//_______variables microros___________
rclc_executor_t executor;
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ error_loop(); }}
const int led_error = 2;
//_______________________________________________________

bool beginMicroros(); // prototype from microros_configuration.ino
void enviarDatos(); // prototype from serial_configuration.ino
void recibirDatos();
bool agent_connected = false;
bool microros_operational = false;

void setup() {
  i2cMutex = xSemaphoreCreateMutex();

  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(led_error, OUTPUT);
  digitalWrite(led_error, HIGH);

  //_____________________MOTOR______________________
  SabertoothSerial.begin(9600, SERIAL_8N1, -1, SABERTOOTH_TX);
  ST.motor(1, 0); ST.motor(2, 0);

  // Opcional: espera Serial (solo si necesitas debugging tempranero)
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) {
    delay(10);
  }

  //_____________________ENCODER______________________
  beginEncoder(LeftA, LeftB, LeftZ, RightA, RightB, RightZ);

  //____________IMU______________
  //while (!Serial) {}

  /*if (!IMU_begin()) {
    Serial.println("Error al inicializar el IMU");
    while (1);
  }*/
  //_______________________________

  //___________SENSOR SHARP____________________
  /*sensor1.init(SHARP_PIN1);
    sensor2.init(SHARP_PIN2);
    sensor3.init(SHARP_PIN3);
    sensor1.calibrateAnalog(2500, 400, 4, 50);
    sensor2.calibrateAnalog(2500, 400, 4, 50);
    sensor3.calibrateAnalog(2500, 400, 4, 50);*/

  //_____________CONTROL PID_______________
  beginPid();

  //__________Tarea para control de velocidad y comunicacion serial, en Core 0_____________________
  xTaskCreatePinnedToCore(
    ControlLoop,
    "ControlLoopTask",
    4096,
    NULL,
    1,
    NULL,
    0
  );
  //______________________________________________________________________
  digitalWrite(led_error, LOW);

  // Inicializar micro-ROS (puede fallar) (inicia la red y los publishers/subscribers)
  if (!beginMicroros()) {
    //Serial.println("⚠️ micro-ROS no pudo inicializarse, funcionando en modo serial");
    agent_connected = false;
    microros_operational = false;
    digitalWrite(led_error, HIGH);
  } else {
    //Serial.println("✅ micro-ROS inicializado correctamente");
    agent_connected = true;
    microros_operational = true;
    digitalWrite(led_error, LOW);
  }
  //Serial.println("ESP32 iniciada y lista para recibir comandos");
}

void loop() {
  // watchdog revisa micro-ROS y actualiza microros_operational (no reinicia)
  watchdog();

  // Solo hacer spin si micro-ROS está operacional
  if (microros_operational && agent_connected) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
      static int executor_errors = 0;
      executor_errors++;

      if (executor_errors > 3) {
        //Serial.println("⚠️ Errores en executor, micro-ROS no operacional");
        microros_operational = false;
        agent_connected = false;
        executor_errors = 0;
      }
    }
  }
  delay(25);


}

void ControlLoop(void *parameter) {
  //___________Inicializaciones de tiempo_____________
  lastTime_enc = millis();
  lastTime_imu = millis();
  lastTime_range = millis();
  lastTime_serial = millis();

  while (1) {
    /*dt_imu = millis() - lastTime_imu;

    if (mpu.update()) {
      if (dt_imu >= sampleTime_imu) {
        lastTime_imu = millis();
        IMU_update();
      }
    }*/

    dt_enc = millis() - lastTime_enc;
    if (dt_enc >= sampleTime_enc) {
      lastTime_enc = millis();
      encoderPID();
    }

    /*if (millis() - lastTime_range >= sampleTime_range) {
      lastTime_range = millis();
      range_front = sensor1.distAnalog() / 100.0;
      range_left = sensor2.distAnalog() / 100.0;
      range_right = sensor3.distAnalog() / 100.0;
      }*/

    if (millis() - lastTime_serial > sampleTime_serial) {
      lastTime_serial = millis();
      enviarDatos();
      recibirDatos();
    }
    delay(10);

  }
}
