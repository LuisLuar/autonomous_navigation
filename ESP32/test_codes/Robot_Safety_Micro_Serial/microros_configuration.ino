//microros_configuration.ino

//____________INTERNET_________________
const char* ssid = "OMEN";
const char* password = "12345678";
const char* agent_ip = "10.42.0.1";  // ← IP de tu PC (donde corre el agent)

/*const char* ssid = "Fastnett-Fibra-ConstructoraVasqu";
const char* password = "1706312434";
const char* agent_ip = "192.168.100.98";*/

const uint32_t agent_port = 8888;
//___________________________

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
static int consecutive_failures = 0;
static unsigned long last_serial_check = 0;
//_________________________________________________


unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

struct timespec getTime() {
  struct timespec tp = { 0 };
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

// Verificar si ambas comunicaciones están caídas
bool both_comms_failed() {
  bool microros_down = !agent_connected || !microros_operational;
  bool serial_down = (millis() - last_serial_activity) > SERIAL_TIMEOUT_MS;

  return microros_down && serial_down;
}

// Reinicio agresivo para casos críticos ambas comunicaciones perdidads
void aggressive_recovery() {
  // Parar motores por seguridad absoluta
  //digitalWrite(GPIO_RELE_M_LEFT, LOW);
  digitalWrite(GPIO_RELE_M_RIGHT, LOW);
  /*digitalWrite(GPIO_RELE_STOP, LOW);
  digitalWrite(GPIO_RELE_LEFT, LOW);
  digitalWrite(GPIO_RELE_RIGHT, LOW);
  digitalWrite(GPIO_RELE_SAFETY, LOW);*/

  // Intentar reconexión rápida primero
  for (int i = 0; i < 3; i++) {
    //Serial.println("🔁 Intento rápido de recuperación " + String(i + 1) + "/3");
    cleanup_microros();
    delay(200);

    if (reconnect_microros()) {
      //Serial.println("✅ Recuperación agresiva exitosa");
      return;
    }
    delay(200);
  }

  // Si falla, reiniciar ESP32
  delay(500);
  ESP.restart();
}

void error_loop() {
  unsigned long last_reconnect_attempt = millis();
  const unsigned long RECONNECT_INTERVAL = 500;
  int reconnect_attempts = 0;
  const int MAX_RECONNECT_ATTEMPTS = 4;

  // Limpiar recursos micro-ROS
  cleanup_microros();

  while (1) {
    // SOLO DETENER MOTORES SI AMBAS COMUNICACIONES FALLAN
    if (both_comms_failed()) {
      //Serial.println("🚨 CRÍTICO: Ambas comunicaciones perdidas - deteniendo motores");
      //digitalWrite(GPIO_RELE_M_LEFT, LOW);
      digitalWrite(GPIO_RELE_M_RIGHT, LOW);
      /*digitalWrite(GPIO_RELE_STOP, LOW);
      digitalWrite(GPIO_RELE_LEFT, LOW);
      digitalWrite(GPIO_RELE_RIGHT, LOW);
      digitalWrite(GPIO_RELE_SAFETY, LOW);*/

      // INICIAR RECUPERACIÓN AGRESIVA
      aggressive_recovery();
      return;
    }

    // LED de error parpadeante
    digitalWrite(led_error, !digitalRead(led_error));

    // Intentar reconexión periódicamente
    if (millis() - last_reconnect_attempt > RECONNECT_INTERVAL) {
      reconnect_attempts++;
      //Serial.printf("🔄 Reconexión %d/%d...\n", reconnect_attempts, MAX_RECONNECT_ATTEMPTS);
      last_reconnect_attempt = millis();

      // Intentar reconexión RÁPIDA
      if (reconnect_microros()) {
        digitalWrite(led_error, LOW);
        return;
      } else {

        if (reconnect_attempts >= MAX_RECONNECT_ATTEMPTS) {

          // Verificar si tenemos comunicación serial
          if ((millis() - last_serial_activity) <= SERIAL_TIMEOUT_MS) {
            digitalWrite(led_error, HIGH); // LED fijo indicando modo serial
            return;
          } else {
            //Serial.println("🚨 Sin comunicaciones - iniciando recuperación agresiva");
            aggressive_recovery();
            return;
          }
        }
      }
    }

    delay(250);
  }
}

// Reconexión micro-ROS RÁPIDA
bool reconnect_microros() {

  // Configurar transporte rápido
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
  delay(250);

  // Búsqueda rápida de agente
  bool agent_found = false;

  for (int i = 0; i < 4; i++) {
    rcl_ret_t ret = rmw_uros_ping_agent(250, 2); // 🔥 REDUCIDO: 1.5 segundos
    if (ret == RCL_RET_OK) {
      agent_found = true;
      //Serial.println("✅ Agente encontrado en intento " + String(i + 1));
      break;
    }
    //Serial.print(".");
    delay(250);
  }

  if (!agent_found) {
    // Último intento con configuración alternativa
    set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
    delay(1000);

    rcl_ret_t final_attempt = rmw_uros_ping_agent(500, 3);
    if (final_attempt == RCL_RET_OK) {
      agent_found = true;
    }
  }

  if (!agent_found) {
    //Serial.println("❌ Agente no disponible");
    return false;
  }


  // Reinicialización rápida
  if (!beginMicroros()) {
    //Serial.println("❌ Falló inicialización rápida");
    return false;
  }

  // Verificación final rápida
  rcl_ret_t final_check = rmw_uros_ping_agent(1000, 1);
  if (final_check == RCL_RET_OK) {
    agent_connected = true;
    microros_operational = true;
    //Serial.println("✅ Reconexión RÁPIDA completada");
    return true;
  }

  return false;
}

bool beginMicroros() {

  //Configurar transporte micro-ROS por UDP
  set_microros_wifi_transports(
    (char*)ssid,          // Conversión explícita a char*
    (char*)password,      // Conversión explícita a char*
    (char*)agent_ip,      // Conversión explícita a char*
    agent_port            // uint32_t
  );

  delay(500);

  // Verificar agente antes de inicializar
  rcl_ret_t agent_check = rmw_uros_ping_agent(1000, 3);
  if (agent_check != RCL_RET_OK) {
    //Serial.println("❌ Agente micro-ROS no disponible");
    return false;
  }

  allocator = rcl_get_default_allocator();


  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)  return false;
  if (rclc_node_init_default(&node, "Robot_Safety", "", &support) != RCL_RET_OK)  return false;

  // Configurar array de batería (2 elementos: voltage_12v, voltage_5v)
  setup_float32_multiarray(&battery_array_msg, 2, "battery_array");
  setup_float32_multiarray(&motors_array_msg, 2, "motors_array");

  if (rclc_publisher_init_default( &battery_array_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/battery_array/microros") != RCL_RET_OK)  return false;
  if (rclc_publisher_init_default( &motors_array_pub,  &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "/motors_array/microros") != RCL_RET_OK)  return false;


  if (rclc_subscription_init_default( &rele_subscriber,  &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "rele_control") != RCL_RET_OK)  return false;

  if (rclc_timer_init_default( &sync_timer,     &support, RCL_MS_TO_NS(120000), sync_timer_callback) != RCL_RET_OK)  return false;
  if (rclc_timer_init_default( &publish_timer,  &support, RCL_MS_TO_NS(100), publish_all) != RCL_RET_OK)  return false;


  if (rclc_executor_init(&executor, &support.context, 4, &allocator));
  if (rclc_executor_add_subscription(&executor, &rele_subscriber, &rele_msg, &Rele_callback, ON_NEW_DATA) != RCL_RET_OK)  return false;
  if (rclc_executor_add_timer(&executor, &sync_timer) != RCL_RET_OK)  return false;
  if (rclc_executor_add_timer(&executor, &publish_timer) != RCL_RET_OK)  return false;

  syncTime();

  return true;
}

//____________________MICROROS_________________________
void syncTime() {
  unsigned long now = millis();

  for (int i = 0; i < 5; i++) {
    rcl_ret_t ret = rmw_uros_sync_session(10);
    if (ret == RCL_RET_OK) {
      unsigned long long ros_time_ms = rmw_uros_epoch_millis();
      time_offset = ros_time_ms - now;
      //Serial.println("¡Sincronización exitosa!");
      return;
    }
    delay(500);
  }
}

//___________________SINCRONIZACIÓN DEL TIEMPO________________________________________
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  syncTime();
}

//__________________activación de reles______________________________________________-
void Rele_callback(const void *msg_in)  {
  prev_cmd_time = millis();
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msg_in;
  uint8_t data = msg->data;

  // Cada bit controla un relé
  /*//Serial.println(data & (1 << 0) ? HIGH : LOW);
    //Serial.println(data & (1 << 1) ? HIGH : LOW);
    //Serial.println(data & (1 << 2) ? HIGH : LOW);
    //Serial.println(data & (1 << 3) ? HIGH : LOW);
    //Serial.println(data & (1 << 4) ? HIGH : LOW);
    //Serial.println(data & (1 << 5) ? HIGH : LOW);
    //Serial.println("");*/

  //digitalWrite(GPIO_RELE_M_LEFT,   data & (1 << 0) ? HIGH : LOW);
  digitalWrite(GPIO_RELE_M_RIGHT,  data & (1 << 1) ? HIGH : LOW);
  /*digitalWrite(GPIO_RELE_STOP,     data & (1 << 2) ? HIGH : LOW);
  digitalWrite(GPIO_RELE_LEFT,     data & (1 << 3) ? HIGH : LOW);
  digitalWrite(GPIO_RELE_RIGHT,    data & (1 << 4) ? HIGH : LOW);
  digitalWrite(GPIO_RELE_SAFETY,   data & (1 << 5) ? HIGH : LOW);*/
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
    //Serial.println("⚠️ Timeout leyendo sensores");
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
    //Serial.println("Error: no se pudo asignar memoria para dim.data");
    return;
  }

  msg->layout.dim.data[0].size = size;
  msg->layout.dim.data[0].stride = size;
  msg->layout.dim.data[0].label = micro_ros_string_utilities_init(label);

  msg->data.capacity = size;
  msg->data.size = size;
  msg->data.data = (float*) allocator.allocate(sizeof(float) * size, allocator.state);

  if (msg->data.data == NULL) {
    //Serial.println("Error: no se pudo asignar memoria para data.data");
    return;
  }

  for (size_t i = 0; i < size; i++) {
    msg->data.data[i] = 0.0f;
  }
}

//_________________________________________________________
void watchdog() {
  // Verificación periódica del agente micro-ROS

  if (millis() - last_agent_check > 1000) {
    //Serial.println("Iiciando watchdog");
    last_agent_check = millis();

    const int MAX_CONSECUTIVE_FAILURES = 2; // 2 fallos consecutivos


    if (millis() - prev_cmd_time > 3000) {
      //Serial.println(millis() - prev_cmd_time);
      rcl_ret_t ret = rmw_uros_ping_agent(800, 1);  // Espera 100ms respuesta
      if (ret != RCL_RET_OK) {
        consecutive_failures++;
        //Serial.println("Error ping");

        if (agent_connected) {
          if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
            agent_connected = false;
            microros_operational = false;
            consecutive_failures = 0;

            // Entrar en recuperación
            error_loop();
          }
        }
      } else {
        consecutive_failures = 0;

        if (!agent_connected) {
          cleanup_microros();
          reconnect_microros();
          //Serial.println("✅ micro-ROS reconectado");
          agent_connected = true;
          microros_operational = true;
        }
      }
    } else {
      agent_connected = true;
      microros_operational = true;
    }
  }

  // Si se perdió la conexión por más de 10s → entra en modo seguro
  if (millis() - last_serial_check > 1000) {
    last_serial_check = millis();

    bool was_serial_connected = serial_connected;
    serial_connected = (millis() - last_serial_activity) <= SERIAL_TIMEOUT_MS;

    /*if (was_serial_connected && !serial_connected) {
      //Serial.println("⚠️ Comunicación serial perdida");
      } else if (!was_serial_connected && serial_connected) {
      //Serial.println("✅ Comunicación serial recuperada");
      }*/

  }
}

void cleanup_microros() {

  // limpiar todo sin verificar estados complejos

  // Limpiar executor
  rclc_executor_fini(&executor);

  // Limpiar timers
  if (publish_timer.impl) RCSOFTCHECK(rcl_timer_fini(&publish_timer));
  if (sync_timer.impl) RCSOFTCHECK(rcl_timer_fini(&sync_timer));

  // Limpiar publishers
  if (battery_array_pub.impl) RCSOFTCHECK(rcl_publisher_fini(&battery_array_pub, &node));
  if (motors_array_pub.impl) RCSOFTCHECK(rcl_publisher_fini(&motors_array_pub, &node));

  // Limpiar subscription
  if (rele_subscriber.impl) RCSOFTCHECK(rcl_subscription_fini(&rele_subscriber, &node));

  // Limpiar nodo
  if (node.impl) RCSOFTCHECK(rcl_node_fini(&node));

  // Limpiar support - no verificar context.initialized
  rclc_support_fini(&support);

  // Resetear variables de estado
  agent_connected = false;
  microros_operational = false;

  delay(250);
  //Serial.println("✅ Limpieza de micro-ROS completada");
}
