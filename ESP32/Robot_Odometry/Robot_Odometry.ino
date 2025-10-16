//Robot_Odometry.ino (PRINCIPAL)
#include <imu_publisher.h>
#include <odometry.h>
#include <ESP32Encoder.h>
#include <SabertoothSimplified.h>
#include <motorControl.h>

/*Motor controller using micro_ros serial set_microros_transports*/
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <GP2Y0E03_ESP32.h>

//___________ENCODER DERECHO_____________
#define RightA 13
#define RightB 25
#define RightZ 14
//_______________________________________

//__________ENCODER LeftUIERDO____________
#define LeftA 18
#define LeftB 19
#define LeftZ 5
//_______________________________________

//______________SENSORES SHARP_____________
#define SHARP_PIN1 33
#define SHARP_PIN2 34
#define SHARP_PIN3 35


//_______variables microros___________
rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const int led_error = 2;


//_______________________________________________________

//____________BIBLIOTECAS PROPIAS___________________________
Odometry odometry;
ImuPublisher imu_pub;
//_________________________________________________________



//_______VARIABLES IMU_________________________________
float ax = 0.0, ay = 0.0, az = 0.0;
float gx = 0.0, gy = 0.0, gz = 0.0;

unsigned long lastTime_imu = 0, dt_imu = 0;
float yaw_imu = 0.0, roll_imu = 0.0, pitch_imu = 0.0;
//______________________________________________________

//___________Parametros del robot_____________
float L = 0.196; // distancia entre ruedas metros
float D = 0.0853; // Diametro de la rueda en metros
//_______________________________________________

//===========CONTROL DE MOTORES===================
#define SABERTOOTH_TX 17   // GPIO17 (TX2)
HardwareSerial SabertoothSerial(2);
SabertoothSimplified ST(SabertoothSerial); //El rango de velocidad es de -127 a +127, donde:
//=======================================================

//==========ESTRUCTURA PARA DATOS MOTORES================
struct DatosMotores {
  double w = 0.0;  // Revoluciones por minuto calculadas.
  double wAnterior = 0.0;
  double wRef = 0.0;  // Velocidad angular en rad/s.
  int outValue = 0; //Variable de control (+-127)
  double wSinFilter = 0.0;     // Velocidad angular en rad/s.
  double v = 0.0; //Velocidad lineal de cada rueda [m/s]
};

DatosMotores Left;
DatosMotores Right;
//======================================================

//____________creación de instancias___________________________
ESP32Encoder encoderL;
ESP32Encoder encoderR;
//_______________________________________________

//________CONFIGURACION ENCODER__________
unsigned long lastTime_enc = 0, dt_enc = 0;;      // Tiempo anterior
const int sampleTime_enc = 100;  // Tiempo de muestreo milisegundos
float yaw_enc = 0.0;
//_______________________________________

//____________CONFIGURACION PID__________________
motorControl motorR(sampleTime_enc);
motorControl motorL(sampleTime_enc);
//_______________________________________________

//___________Parametros del robot_____________
float v = 0, w = 0;
float x_pos = 0.0, delta_x = 0.0; //Posición estimada por odometría METROS
float y_pos = 0.0,  delta_y = 0.0;
//____________________________________________

GP2Y0E03 sensor1;
GP2Y0E03 sensor2;
GP2Y0E03 sensor3;

void setup() {
  Serial.begin(115200);  // Inicializa el puerto serial a 9600 bps
  Serial.println("ESP32 iniciada y lista para recibir comandos");
  analogReadResolution(12);

  //_____________________MICROROS________________________________
  beginMicroros();

  // Opcional: espera Serial (solo si necesitas debugging tempranero)
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) { delay(10); } // espera max 2s


  if (!IMU_begin()) {
    Serial.println("Error al inicializar el IMU");
    while (1);
  }

  //_____________________MOTOR______________________
  SabertoothSerial.begin(9600, SERIAL_8N1, -1, SABERTOOTH_TX);
  Serial.println("Sabertooth listo");

  //_____________________ENCODER______________________
  beginEncoder(LeftA, LeftB, LeftZ, RightA, RightB, RightZ);

  //___________SENSOR SHARP____________________
  sensor1.init(SHARP_PIN1);
  sensor2.init(SHARP_PIN2);
  sensor3.init(SHARP_PIN3);

  sensor1.calibrateAnalog(2500, 400, 4, 50);
  sensor2.calibrateAnalog(2500, 400, 4, 50);
  sensor3.calibrateAnalog(2500, 400, 4, 50);
  Serial.println("Sensor Sharp GP2Y0E03 listo (modo analógico)"); 

  //_____________CONTROL PID_______________
  beginPid();

  //__________Tarea para control de velocidad, en Core 0
  xTaskCreatePinnedToCore(
    ControlLoop,        // Función
    "ControlLoopTask",  // Nombre
    4096,               // Stack size
    NULL,               // Parametros
    1,                  // Prioridad
    NULL,               // Handle
    0                   // Core 0
  );
  Serial.print("setup() running on core ");

  digitalWrite(led_error, LOW);
  
  lastTime_enc = millis();

  lastTime_imu = millis();
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(40)));
}

//_______FUNCION PARA EJECUTAR EN EL SEGUNDO NUCLEO_____________-
void ControlLoop(void *parameter) {
  Serial.print("ControlLoop() running on core ");
  Serial.println(xPortGetCoreID());
  
  while (1) {

    dt_imu = millis() - lastTime_imu;
    if (dt_imu >= 50) {
      lastTime_imu = millis();
      IMU_update();
    }

    dt_enc = millis() - lastTime_enc; //milisegundos
    if (dt_enc >= sampleTime_enc) {// Se actualiza cada tiempo de muestreo
      lastTime_enc = millis();  // Almacenamos el tiempo actual.
      encoderPID(); // actualiza V y W del robot Y X,Y posicion y orientacion del encoder, y manda la señal PID
    }

    delay(10);
    
  }
  
}
