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
#define GPIO_CURRENT_M_LEFT 34    // Entrada analógica de la ESP32 (32, 33, 34, 35, 36 o 39)
#define GPIO_CURRENT_M_RIGHT 35
#define GPIO_VOLTAJE_12V 33
#define GPIO_VOLTAJE_5V 32

#define GPIO_RELE_M_LEFT 14
#define GPIO_RELE_M_RIGHT 17
#define GPIO_RELE_STOP 23
#define GPIO_RELE_LEFT 25
#define GPIO_RELE_RIGHT 26
#define GPIO_RELE_SAFETY 27
//___________________________________________


//_____________VARIABLES_________________________________________
volatile float current_Mleft = 0.0;
volatile float offset_Mleft = 0.0;  // Voltaje de salida en reposo (sin corriente)

volatile float current_Mright = 0.0;
volatile float offset_Mright = 0.0;  // Voltaje de salida en reposo (sin corriente)

volatile float voltage_12V = 0.0;
volatile float voltage_5V = 0.0;

SemaphoreHandle_t sensor_mutex = NULL;
//___________________________________________

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

  offset_Mleft = calibrateSensorCurrent(GPIO_CURRENT_M_LEFT);
  offset_Mright = calibrateSensorCurrent(GPIO_CURRENT_M_RIGHT);

  //___________RELES___________________
  pinMode(GPIO_RELE_M_LEFT, OUTPUT);
  pinMode(GPIO_RELE_M_RIGHT, OUTPUT);
  pinMode(GPIO_RELE_STOP, OUTPUT);
  pinMode(GPIO_RELE_LEFT, OUTPUT);
  pinMode(GPIO_RELE_RIGHT, OUTPUT);
  pinMode(GPIO_RELE_SAFETY, OUTPUT);

  //_____________________MICROROS________________________________
  beginMicroros();

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
  Serial.print("setup() running on core ");

}

void loop() {
  watchdog();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(40)));
  delay(10);
}


//_______FUNCION PARA EJECUTAR EN EL SEGUNDO NUCLEO_____________-
void ControlLoop(void *parameter) {
  Serial.print("ControlLoop() running on core ");
  Serial.println(xPortGetCoreID());

  while (1) {
    float left = readCurrent(GPIO_CURRENT_M_LEFT, offset_Mleft);
    float right = readCurrent(GPIO_CURRENT_M_RIGHT, offset_Mright);
    float v12 = readVoltage(GPIO_VOLTAJE_12V);
    float v5  = readVoltage(GPIO_VOLTAJE_5V);

    // Protegemos la escritura
    if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      current_Mleft = left;
      current_Mright = right;
      voltage_12V = v12;
      voltage_5V = v5;
      xSemaphoreGive(sensor_mutex);
    } else {
      Serial.println("❌ Error: No se pudo actualizar sensores");
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // espera 500 ms (no bloquear otros hilos)
  }
}
