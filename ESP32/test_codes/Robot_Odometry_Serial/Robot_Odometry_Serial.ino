//Robot_Odometry.ino (PRINCIPAL)
#include <ESP32Encoder.h>
#include <SabertoothSimplified.h>
#include <motorControl.h>


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
//_______________________________________

//__________________CONTROL DE MOTORES______________________
#define SABERTOOTH_TX 17   // GPIO17 (TX2)
HardwareSerial SabertoothSerial(2);
SabertoothSimplified ST(SabertoothSerial); //El rango de velocidad es de -63 a +63, donde:
//_______________________________________

//___________Parametros del robot_____________
float L = 0.196; // distancia entre ruedas metros
float D = 0.0853; // Diametro de la rueda en metros
//_______________________________________________

//_______VARIABLES IMU_________________________________
const int sampleTime_imu = 50;  // Tiempo de muestreo milisegundos
unsigned long lastTime_imu = 0, dt_imu = 0;

float ax = 0.0, ay = 0.0, az = 0.0;
float gx = 0.0, gy = 0.0, gz = 0.0;
float roll_imu = 0.0, pitch_imu = 0.0, yaw_imu = 0.0;
//______________________________________________________

//________CONFIGURACION ENCODER__________
const int sampleTime_enc = 50;  // Tiempo de muestreo milisegundos
unsigned long lastTime_enc = 0, dt_enc = 0;     

float vx = 0, wz = 0; //Velocidad lineal y angular del robot [m/s] y [rad/s]
float x_pos = 0.0, y_pos = 0.0,  delta_y = 0.0, delta_x = 0.0; //Posición estimada por odometría METROS
float yaw_enc = 0.0; //Orientación estimada por odometría RADIANES
//_______________________________________

bool reset = false;
const int sampleTime_range = 200;  // Tiempo de muestreo milisegundos
unsigned long lastTime_range = 0, dt_range = 0; 
float range_front = 0.0, range_left = 0.0, range_right = 0.0;

//_______variables microros___________

const int led_error = 2;
//_______________________________________________________


//______________ESTRUCTURA PARA DATOS MOTORES___________________________
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
//_________________________________________________________

//____________creación de instancias__________________________

ESP32Encoder encoderL;
ESP32Encoder encoderR;

GP2Y0E03 sensor1;
GP2Y0E03 sensor2;
GP2Y0E03 sensor3;

motorControl motorR(sampleTime_enc);
motorControl motorL(sampleTime_enc);
//_________________________________________________________


void setup() {
  Serial.begin(115200);  // Inicializa el puerto serial a 9600 bps
  analogReadResolution(12);
  pinMode(led_error, OUTPUT);
  digitalWrite(led_error, HIGH);

  //_____________________MOTOR______________________
  SabertoothSerial.begin(9600, SERIAL_8N1, -1, SABERTOOTH_TX);
  //Serial.println("Sabertooth listo");
  ST.motor(1, 0);
  ST.motor(2, 0);


  // Opcional: espera Serial (solo si necesitas debugging tempranero)
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) { delay(10); } // espera max 2s


  /*if (!IMU_begin()) {
    Serial.println("Error al inicializar el IMU");
    while (1);
  }*/


  //_____________________ENCODER______________________
  beginEncoder(LeftA, LeftB, LeftZ, RightA, RightB, RightZ);

  //___________SENSOR SHARP____________________
  sensor1.init(SHARP_PIN1);
  sensor2.init(SHARP_PIN2);
  sensor3.init(SHARP_PIN3);

  sensor1.calibrateAnalog(2500, 400, 4, 50);
  sensor2.calibrateAnalog(2500, 400, 4, 50);
  sensor3.calibrateAnalog(2500, 400, 4, 50);
  //Serial.println("Sensor Sharp GP2Y0E03 listo (modo analógico)"); 

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
  //Serial.print("setup() running on core ");

  digitalWrite(led_error, LOW);
  
  lastTime_enc = millis();
  lastTime_imu = millis();
  lastTime_range = millis();
  
  //Serial.println("ESP32 iniciada y lista para recibir comandos");
}

void loop() {
   // Enviar datos a Python
  enviarDatos();
  
  // Recibir datos de Python
  recibirDatos();
  
  delay(50); // Pequeño delay para estabilidad
}

//_______FUNCION PARA EJECUTAR EN EL SEGUNDO NUCLEO_____________-
void ControlLoop(void *parameter) {
  //Serial.print("ControlLoop() running on core ");
  //Serial.println(xPortGetCoreID());
  
  while (1) {

    /*dt_imu = millis() - lastTime_imu;
    if (dt_imu >= sampleTime_imu) {
      lastTime_imu = millis();
      IMU_update();
    }*/

    dt_enc = millis() - lastTime_enc; //milisegundos
    if (dt_enc >= sampleTime_enc) {// Se actualiza cada tiempo de muestreo
      lastTime_enc = millis();  // Almacenamos el tiempo actual.
      encoderPID(); // actualiza V y W del robot Y X,Y posicion y orientacion del encoder, y manda la señal PID
    }

    dt_range = millis() - lastTime_range; //milisegundos
    if (dt_range >= sampleTime_range) {// Se actualiza cada tiempo de muestreo
      lastTime_range = millis();  // Almacenamos el tiempo actual.
      range_front = sensor1.distAnalog() / 100.0;
      range_left = sensor2.distAnalog() / 100.0;
      range_right = sensor3.distAnalog() / 100.0;
    }


    delay(10);
    
  }
  
}
