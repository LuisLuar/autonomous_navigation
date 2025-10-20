//microros_configuration.ino
/*Motor controller using micro_ros serial set_microros_transports*/
#include <WiFi.h>  // ← Nueva librería

//___________variables de microros____________________
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t rele_subscriber;

std_msgs__msg__UInt8 rele_msg;

std_msgs__msg__Float32MultiArray battery_array_msg;
std_msgs__msg__Float32MultiArray motors_array_msg;

rcl_timer_t sync_timer;
rcl_timer_t publish_timer;

rcl_publisher_t battery_array_pub;
rcl_publisher_t motors_array_pub;

//__________VARIABLES PARA SEGURIDAD_________________________-
unsigned long last_agent_check = 0;
const unsigned long AGENT_TIMEOUT_MS = 10000;  // 10s sin respuesta → error
bool agent_connected = true;
//_________________________________________________

//____________INTERNET_________________
/*const char* ssid = "OMEN";
const char* password = "12345678";
const char* agent_ip = "10.42.0.1";  // ← IP de tu PC (donde corre el agent)*/

const char* ssid = "Fastnett-Fibra-ConstructoraVasqu";
const char* password = "1706312434";
const char* agent_ip = "192.168.100.167";

const uint32_t agent_port = 8888;
//___________________________m

unsigned long long time_offset = 0;

struct timespec getTime() {
  struct timespec tp = { 0 };
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

void error_loop() {
  unsigned long long last_error_time = millis();
  while (1) {

    digitalWrite(led_error, !digitalRead(led_error));
    Serial.println("error..");
    // Desactivar todos los relés
    digitalWrite(GPIO_RELE_M_LEFT, LOW);
    digitalWrite(GPIO_RELE_M_RIGHT, LOW);
    digitalWrite(GPIO_RELE_STOP, LOW);
    digitalWrite(GPIO_RELE_LEFT, LOW);
    digitalWrite(GPIO_RELE_RIGHT, LOW);
    digitalWrite(GPIO_RELE_SAFETY, LOW);
    delay(500);

    if (millis() - last_error_time > 3000) ESP.restart();
  }
}

void beginMicroros() {
  //______MICROROS_________
  //set_microros_transports();
  //delay(2000);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(200);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nAdvertencia: No se pudo conectar a WiFi en 10s. Intentando continuar.");
  } else {
    Serial.println("\nConectado a WiFi");
  }

  //Configurar transporte micro-ROS por UDP
  set_microros_wifi_transports(
    (char*)ssid,          // Conversión explícita a char*
    (char*)password,      // Conversión explícita a char*
    (char*)agent_ip,      // Conversión explícita a char*
    agent_port            // uint32_t
  );

  delay(1000);

  allocator = rcl_get_default_allocator();

  //create init_options, node
  Serial.println("INICIANDO");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "Robot_Safety", "", &support));

  // Configurar array de batería (2 elementos: voltage_12v, voltage_5v)
  setup_float32_multiarray(&battery_array_msg, 2, "battery_array");
  setup_float32_multiarray(&motors_array_msg, 2, "motors_array");

  RCCHECK(rclc_publisher_init_default( &battery_array_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "battery/array"));
  RCCHECK(rclc_publisher_init_default( &motors_array_pub,  &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motors/array"));

  Serial.println("CREANDO SUSCRIPTORES");
  RCCHECK(rclc_subscription_init_default( &rele_subscriber,  &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "rele/control"));

  Serial.println("CREANDO TIMER");
  RCCHECK(rclc_timer_init_default( &sync_timer,     &support, RCL_MS_TO_NS(120000), sync_timer_callback));
  RCCHECK(rclc_timer_init_default( &publish_timer,  &support, RCL_MS_TO_NS(250), publish_all));

  // create executor
  Serial.println("CREANDO EXECUTOR");
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &rele_subscriber, &rele_msg, &Rele_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &publish_timer));
  delay(1000);

  syncTime();
}

//____________________MICROROS_________________________
void syncTime() {
  unsigned long now = millis();
  Serial.println("Sincronizando tiempo con ROS 2...");

  for (int i = 0; i < 5; i++) {
    rcl_ret_t ret = rmw_uros_sync_session(10);
    if (ret == RCL_RET_OK) {
      unsigned long long ros_time_ms = rmw_uros_epoch_millis();
      time_offset = ros_time_ms - now;
      Serial.println("¡Sincronización exitosa!");
      return;
    }
    Serial.println("Reintentando sincronización...");
    delay(500);
  }

  Serial.println("¡Fallo al sincronizar tiempo! Continuando sin sync...");

  //RCCHECK(rmw_uros_sync_session(10));
}

//___________________SINCRONIZACIÓN DEL TIEMPO________________________________________
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  syncTime();
}

//__________________activación de reles______________________________________________-
void Rele_callback(const void *msg_in)  {
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msg_in;
  uint8_t data = msg->data;

  // Cada bit controla un relé
  /*Serial.println(data & (1 << 0) ? HIGH : LOW);
  Serial.println(data & (1 << 1) ? HIGH : LOW);
  Serial.println(data & (1 << 2) ? HIGH : LOW);
  Serial.println(data & (1 << 3) ? HIGH : LOW);
  Serial.println(data & (1 << 4) ? HIGH : LOW);
  Serial.println(data & (1 << 5) ? HIGH : LOW);
  Serial.println("");*/
  
  digitalWrite(GPIO_RELE_M_LEFT,   data & (1 << 0) ? HIGH : LOW);  
  digitalWrite(GPIO_RELE_M_RIGHT,  data & (1 << 1) ? HIGH : LOW);  
  digitalWrite(GPIO_RELE_STOP,     data & (1 << 2) ? HIGH : LOW);  
  digitalWrite(GPIO_RELE_LEFT,     data & (1 << 3) ? HIGH : LOW);  
  digitalWrite(GPIO_RELE_RIGHT,    data & (1 << 4) ? HIGH : LOW);  
  digitalWrite(GPIO_RELE_SAFETY,   data & (1 << 5) ? HIGH : LOW);  
}
//_________________________________________________

//___________PUBLICACIÓN DE MENSAJES________________
void publish_all(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  
  extern SemaphoreHandle_t sensor_mutex;
  extern volatile float voltage_12V, voltage_5V, current_Mleft, current_Mright;
  
  if (xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Copiar valores a variables temporales (rápido)
    battery_array_msg.data.data[0] = voltage_12V;
    battery_array_msg.data.data[1] = voltage_5V;
    motors_array_msg.data.data[0] = current_Mleft;
    motors_array_msg.data.data[1] = current_Mright;

    xSemaphoreGive(sensor_mutex);  // 🔓 Liberar ASAP  

    RCSOFTCHECK(rcl_publish(&battery_array_pub, &battery_array_msg, NULL));
    RCSOFTCHECK(rcl_publish(&motors_array_pub, &motors_array_msg, NULL));
  } else {
    Serial.println("⚠️ Timeout leyendo sensores");
  }
}
//__________________________________________________

//________________Función auxiliar para configurar arrays________________________________
void setup_float32_multiarray(std_msgs__msg__Float32MultiArray * msg,
                              size_t size, const char* label)
{
  // Limpia primero
  memset(msg, 0, sizeof(std_msgs__msg__Float32MultiArray));

  // Asigna 1 dimensión
  msg->layout.dim.capacity = 1;
  msg->layout.dim.size = 1;
  msg->layout.dim.data = (std_msgs__msg__MultiArrayDimension*)
      allocator.allocate(sizeof(std_msgs__msg__MultiArrayDimension), allocator.state);

  if (msg->layout.dim.data == NULL) {
    Serial.println("Error: no se pudo asignar memoria para dim.data");
    return;
  }

  msg->layout.dim.data[0].size = size;
  msg->layout.dim.data[0].stride = size;
  msg->layout.dim.data[0].label = micro_ros_string_utilities_init(label);

  msg->data.capacity = size;
  msg->data.size = size;
  msg->data.data = (float*) allocator.allocate(sizeof(float) * size, allocator.state);

  if (msg->data.data == NULL) {
    Serial.println("Error: no se pudo asignar memoria para data.data");
    return;
  }

  for (size_t i = 0; i < size; i++) {
    msg->data.data[i] = 0.0f;
  }
}

//_________________________________________________________
void watchdog(){
  // Verificación periódica del agente micro-ROS
  if (millis() - last_agent_check > 3000) {
    last_agent_check = millis();
    rcl_ret_t ret = rmw_uros_ping_agent(100, 1);  // Espera 100ms respuesta
    if (ret != RCL_RET_OK) {
      if (agent_connected) {
        Serial.println("⚠️ Conexión con agente perdida. Esperando reconexión...");
        agent_connected = false;
      }
    } else {
      if (!agent_connected) {
        Serial.println("✅ Conexión con agente restaurada.");
      }
      agent_connected = true;
    }

    // Si se perdió la conexión por más de 10s → entra en modo seguro
    static unsigned long disconnected_since = 0;
    if (!agent_connected) {
      if (disconnected_since == 0)
        disconnected_since = millis();

      if (millis() - disconnected_since > AGENT_TIMEOUT_MS) {
        Serial.println("🚨 Conexión perdida por más de 10s. Activando modo seguro.");
        error_loop();  // Se queda parpadeando y reinicia tras unos segundos
      }
    } else {
      disconnected_since = 0;
    }
  }
}
