// Robot_Safety.ino (principal)
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <micro_ros_utilities/string_utilities.h>


//______DEFINICION DE GPIO________________________________-
//#define GPIO_CURRENT_M_LEFT 34    // Entrada analógica de la ESP32 (32, 33, 34, 35, 36 o 39)
//#define GPIO_CURRENT_M_RIGHT 35
#define GPIO_VOLTAJE_12V 32
//#define GPIO_VOLTAJE_5V 32

//#define GPIO_RELE_M_LEFT 14
#define GPIO_RELE_M_RIGHT 33
/*#define GPIO_RELE_STOP 34
#define GPIO_RELE_LEFT 19//36
#define GPIO_RELE_RIGHT 18//39
#define GPIO_RELE_SAFETY 35*/
//___________________________________________


//_____________VARIABLES_________________________________________
volatile float current_Mleft = 0.0;
volatile float offset_Mleft = 0.0;  // Voltaje de salida en reposo (sin corriente)

volatile float current_Mright = 0.0;
volatile float offset_Mright = 0.0;  // Voltaje de salida en reposo (sin corriente)

volatile float voltage_12V = 0.0;
volatile float voltage_5V = 0.0;
float anterior_12v = 11.0;
float anterior_5v = 4.0;

SemaphoreHandle_t sensor_mutex = NULL;
//___________________________________________

//_____________cariables para seguridad de microros y serial______________
unsigned long last_serial_activity = 0;
const unsigned long SERIAL_TIMEOUT_MS = 2000; // 2 segundos sin actividad serial
bool serial_connected = false;
bool reset = false;

bool agent_connected = false;
bool microros_operational = false;

//_____________________________________________________________________

//________TIEMPO SERIAL___________________
const int sampleTime_serial = 50;  // Tiempo de muestreo milisegundos
unsigned long lastTime_serial = 0;
//_______________________________________

//________________VARIABLES GLOBALES DE MICROROS__________________
rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const int led_error = 2;
//_______________________________________________________

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  delay(500);

  // Crear mutex
  sensor_mutex = xSemaphoreCreateMutex();
  if (sensor_mutex == NULL) {
    Serial.println("Fallo creando mutex sensors. Abortando.");
    while (1) {delay(1000);}
  }

  //offset_Mleft = calibrateSensorCurrent(GPIO_CURRENT_M_LEFT);
  //offset_Mright = calibrateSensorCurrent(GPIO_CURRENT_M_RIGHT);

  //___________RELES___________________
  //pinMode(GPIO_RELE_M_LEFT, OUTPUT);
  pinMode(GPIO_RELE_M_RIGHT, OUTPUT);
  /*pinMode(GPIO_RELE_STOP, OUTPUT);
  pinMode(GPIO_RELE_LEFT, OUTPUT);
  pinMode(GPIO_RELE_RIGHT, OUTPUT);
  pinMode(GPIO_RELE_SAFETY, OUTPUT);*/


  // Opcional: espera Serial (solo si necesitas debugging tempranero)
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) {
    delay(10);  // espera max 2s
  }

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

  // Inicializar micro-ROS (puede fallar) (inicia la red y los publishers/subscribers)
  if (!beginMicroros()) {
    agent_connected = false;
    microros_operational = false;
    digitalWrite(led_error, HIGH);
  } else {
    agent_connected = true;
    microros_operational = true;
    digitalWrite(led_error, LOW);
  }

}

void loop() {
  watchdog();
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


//_______FUNCION PARA EJECUTAR EN EL SEGUNDO NUCLEO_____________-
void ControlLoop(void *parameter) {
  //___________Inicializaciones de tiempo_____________
  lastTime_serial = millis();

  while (1) {
    //float left = readCurrent(GPIO_CURRENT_M_LEFT, offset_Mleft);
    //float right = readCurrent(GPIO_CURRENT_M_RIGHT, offset_Mright);
    float v12 = readVoltage(GPIO_VOLTAJE_12V, 0.05, anterior_12v);
    //float v5  = readVoltage(GPIO_VOLTAJE_5V, 0.5, anterior_5v);

    // Protegemos la escritura
    if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      //current_Mleft = left;
      //current_Mright = right;
      voltage_12V = v12;
      //voltage_5V = v5;
      xSemaphoreGive(sensor_mutex);
    } 
    vTaskDelay(pdMS_TO_TICKS(500)); // espera 500 ms (no bloquear otros hilos)

    if (millis() - lastTime_serial > sampleTime_serial) {
      lastTime_serial = millis();
      enviarDatos(0, 0, v12, 0);
      recibirDatos();
    }

    delay(50);
  }
}
