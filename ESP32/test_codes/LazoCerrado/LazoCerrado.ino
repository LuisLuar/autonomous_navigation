//LazoCerrado.ino (PRINCIPAL)
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

//___________ENCODER DERECHO_____________
#define RightA 27
#define RightB 14
#define RightZ 26
//_______________________________________

//__________ENCODER IZQUIERDO____________
#define LeftA 5
#define LeftB 18
#define LeftZ 19
//_______________________________________

//________CONFIGURACION ENCODER__________
unsigned long lastTime_enc = 0, dt_enc = 0;;      // Tiempo anterior
const int sampleTime_enc = 100;  // Tiempo de muestreo milisegundos
//_______________________________________

//_______variables microros___________
rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const int led_error = 2;
//_______________________________________________________

//____________creación de instancias___________________________
Odometry odometry;

ESP32Encoder encoderL;
ESP32Encoder encoderR;

motorControl motorR(sampleTime_enc);
motorControl motorL(sampleTime_enc);
//_________________________________________________________


//===========CONTROL DE MOTORES===================
#define SABERTOOTH_TX 17   // GPIO17 (TX2)
HardwareSerial SabertoothSerial(2);
SabertoothSimplified ST(SabertoothSerial); //El rango de velocidad es de -63 a +63, donde:
//=======================================================

//==========ESTRUCTURA PARA DATOS MOTORES================
struct DatosMotores {
  double w = 0.0;  // Velocidad angular en rad/s.
  double wAnterior = 0.0;
  double wSinFilter = 0.0;     // Velocidad angular cruda en rad/s.
  double wRef = 0.0;  // Velocidad angular de referencia en rad/s.
  int outValue = 0; // Variable de control (+-63)
  double v = 0.0; // Velocidad lineal de cada rueda [m/s]
};

DatosMotores Left;
DatosMotores Right;
//======================================================


void setup() {
  Serial.begin(115200);  // Inicializa el puerto serial a 9600 bps
  analogReadResolution(12);
  pinMode(led_error, OUTPUT);
  digitalWrite(led_error, HIGH);

  //_____________________MOTOR______________________
  SabertoothSerial.begin(9600, SERIAL_8N1, -1, SABERTOOTH_TX);
  Serial.println("Sabertooth listo");
  ST.motor(1, 0);
  ST.motor(2, 0);

  //_____________________MICROROS________________________________
  beginMicroros();

  // Opcional: espera Serial (solo si necesitas debugging tempranero)
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) {
    delay(10);  // espera max 2s
  }
  
  //_____________________ENCODER______________________
  beginEncoder(LeftA, LeftB, LeftZ, RightA, RightB, RightZ);

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

  Serial.println("ESP32 iniciada y lista para recibir comandos");
}

void loop() {
  watchdog();
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(40)));
  delay(10);
}

//_______FUNCION PARA EJECUTAR EN EL SEGUNDO NUCLEO_____________-
void ControlLoop(void *parameter) {
  Serial.print("ControlLoop() running on core ");
  Serial.println(xPortGetCoreID());

  while (1) {

    dt_enc = millis() - lastTime_enc; //milisegundos
    if (dt_enc >= sampleTime_enc) {// Se actualiza cada tiempo de muestreo
      lastTime_enc = millis();  // Almacenamos el tiempo actual.
      encoderPID(); // actualiza
    }

    delay(10);

  }

}
