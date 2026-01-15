// microros_configuration.ino
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/set_bool.h>

//_________CONFIGURACIÓN WIFI Y AGENTE MICRO-ROS________________
const char* ssid = "OMEN";
const char* password = "12345678";
const char* agent_ip = "10.42.0.1";  // ← IP de tu PC (donde corre el agent)


/*const char* ssid = "Fastnett-Fibra-ConstructoraVasqu";
  const char* password = "1706312434";
  const char* agent_ip = "192.168.100.175";*/
const uint32_t agent_port = 8888;
//_____________________________________________________________


#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// seguridad
unsigned long last_agent_check = 0;
const unsigned long AGENT_TIMEOUT_MS = 10000;

static int consecutive_failures = 0;
// Verificar actividad serial periódicamente
static unsigned long last_serial_check = 0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_service_t set_bool_service;

rcl_subscription_t subscriber;
rcl_publisher_t odom_publisher;
//rcl_publisher_t pose_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t range_front_publisher;
rcl_publisher_t range_left_publisher;
rcl_publisher_t range_right_publisher;

std_srvs__srv__SetBool_Request set_bool_req;
std_srvs__srv__SetBool_Response set_bool_res;

geometry_msgs__msg__Twist msg;
nav_msgs__msg__Odometry odom_msg;
//nav_msgs__msg__Odometry pose_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Range range_front_msg;
sensor_msgs__msg__Range range_left_msg;
sensor_msgs__msg__Range range_right_msg;

rcl_timer_t odom_timer;
rcl_timer_t range_timer;
rcl_timer_t sync_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

// getTime
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
  ST.motor(1, 0);
  ST.motor(2, 0);

  // Intentar reconexión rápida primero
  for (int i = 0; i < 3; i++) {
    //Serial.println("🔁 Intento rápido de recuperación " + String(i+1) + "/3");
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
      ST.motor(1, 0);
      ST.motor(2, 0);

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
      //Serial.println("✅ Agente encontrado en intento " + String(i+1));
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

  // Configurar transporte
  set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
  delay(500);

  // Verificar agente antes de inicializar
  rcl_ret_t agent_check = rmw_uros_ping_agent(1000, 3);
  if (agent_check != RCL_RET_OK) {
    ////Serial.println("❌ Agente micro-ROS no disponible");
    return false;
  }

  // Inicializar micro-ROS
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)  return false;

  if (rclc_node_init_default(&node, "Robot_Control", "", &support) != RCL_RET_OK) return false;

  // servicio de reset micro-ROS
  if (rclc_service_init_default(&set_bool_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/robot_control_reset_microros") != RCL_RET_OK) return false;

  // Subscription
  if (rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK)return false;

  // Publishers
  if (rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/microros") != RCL_RET_OK) return false;
  //if (rclc_publisher_init_default(&pose_publisher,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),"robot/pose")!= RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/microros") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&range_front_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "range/front/microros") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&range_left_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "range/left/microros") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&range_right_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "range/right/microros") != RCL_RET_OK) return false;

  // Timers
  if (rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(50), publish_odom) != RCL_RET_OK) return false;
  if (rclc_timer_init_default(&range_timer, &support, RCL_MS_TO_NS(200), publish_ranges_all) != RCL_RET_OK) return false;
  if (rclc_timer_init_default(&sync_timer, &support, RCL_MS_TO_NS(120000), sync_timer_callback) != RCL_RET_OK) return false;

  // Executor
  if (rclc_executor_init(&executor, &support.context, 6, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &odom_timer) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &range_timer) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &sync_timer) != RCL_RET_OK) return false;

  if (rclc_executor_add_service(&executor, &set_bool_service, &set_bool_req, &set_bool_res, set_bool_callback) != RCL_RET_OK) return false;

  // Sincronizar tiempo
  syncTime();

  // Range fixed config
  micro_ros_string_utilities_set(range_front_msg.header.frame_id, "range_front_link");
  range_front_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range_front_msg.field_of_view = 0.52;
  range_front_msg.min_range = 0.04;
  range_front_msg.max_range = 0.5;

  micro_ros_string_utilities_set(range_left_msg.header.frame_id, "range_left_link");
  range_left_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range_left_msg.field_of_view = 0.52;
  range_left_msg.min_range = 0.04;
  range_left_msg.max_range = 0.5;

  micro_ros_string_utilities_set(range_right_msg.header.frame_id, "range_right_link");
  range_right_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range_right_msg.field_of_view = 0.52;
  range_right_msg.min_range = 0.04;
  range_right_msg.max_range = 0.5;

  return true;
}

void syncTime() {
  unsigned long now = millis();
  for (int i = 0; i < 5; i++) {
    rcl_ret_t ret = rmw_uros_sync_session(10);
    if (ret == RCL_RET_OK) {
      unsigned long long ros_time_ms = rmw_uros_epoch_millis();
      time_offset = ros_time_ms - now;
      return;
    }
    delay(500);
  }
}

void publish_odom(rcl_timer_t* timer, int64_t last_call_time) {
  (void)timer; (void)last_call_time;
  struct timespec time_stamp = getTime();
  odometry.update(vx, wz, x_pos, y_pos, yaw_enc);
  imu_pub.update(ax, ay, az, gx, gy, gz, roll_imu, pitch_imu, yaw_imu);

  odom_msg = odometry.getData();
  imu_msg = imu_pub.getData();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

  /*odometry.update(0.0, 0.0, x_pose, y_pose, yaw_enc);
  pose_msg = odometry.getData();
  pose_msg.header.stamp.sec = time_stamp.tv_sec;
  pose_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&pose_publisher, &pose_msg, NULL));*/
}

void publish_ranges_all(rcl_timer_t * timer, int64_t last_call_time) {
  (void)timer; (void)last_call_time;
  struct timespec time_stamp = getTime();
  range_front_msg.header.stamp.sec = time_stamp.tv_sec;
  range_front_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  range_front_msg.range = range_front;
  RCSOFTCHECK(rcl_publish(&range_front_publisher, &range_front_msg, NULL));

  time_stamp = getTime();
  range_left_msg.header.stamp.sec = time_stamp.tv_sec;
  range_left_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  range_left_msg.range = range_left;
  RCSOFTCHECK(rcl_publish(&range_left_publisher, &range_left_msg, NULL));

  time_stamp = getTime();
  range_right_msg.header.stamp.sec = time_stamp.tv_sec;
  range_right_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  range_right_msg.range = range_right;
  RCSOFTCHECK(rcl_publish(&range_right_publisher, &range_right_msg, NULL));
}

void subscription_callback(const void* msgin) {
  prev_cmd_time = millis();
  const geometry_msgs__msg__Twist * incoming = (const geometry_msgs__msg__Twist *)msgin;
  float linear = incoming->linear.x;
  float angular = incoming->angular.z;

  float vI = linear - (angular * L) / 2;
  float vD = linear + (angular * L) / 2;

  Left.wRef = (2 * vI) / D;
  Right.wRef = (2 * vD) / D;
}

void set_bool_callback(const void * req, void * res) {
  std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) req;
  std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) res;

  bool requested_state = req_in->data;
  if (requested_state) {
    roll_imu = 0; pitch_imu = 0; yaw_imu = 0;
    x_pos = 0.0; y_pos = 0.0; yaw_enc = 0.0;

    res_in->success = true;
    static char success_msg1[] = "Estado activado exitosamente (microros)";
    res_in->message.data = success_msg1;
    res_in->message.size = strlen(res_in->message.data);
  } else {
    res_in->success = true;
    static char success_msg2[] = "Estado desactivado exitosamente (microros)";
    res_in->message.data = success_msg2;
    res_in->message.size = strlen(res_in->message.data);
  }
}


void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  syncTime();
}


void watchdog() {
  if (millis() - last_agent_check > 1000) { // 1.5 segundos
    last_agent_check = millis();
    const int MAX_CONSECUTIVE_FAILURES = 2; // 2 fallos consecutivos

    if (millis() - prev_cmd_time > 3000) {
      Serial.println(millis() - prev_cmd_time);
      rcl_ret_t ret = rmw_uros_ping_agent(800, 1);  // Espera 100ms respuesta
      if (ret != RCL_RET_OK) {
        consecutive_failures++;
        Serial.println("Error ping");

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
          Serial.println("✅ micro-ROS reconectado");
          agent_connected = true;
          microros_operational = true;
        }
      }
    } else {
      agent_connected = true;
      microros_operational = true;
    }
  }


  if (millis() - last_serial_check > 1000) {
    last_serial_check = millis();

    bool was_serial_connected = serial_connected;
    serial_connected = (millis() - last_serial_activity) <= SERIAL_TIMEOUT_MS;

    /*if (was_serial_connected && !serial_connected) {
      Serial.println("⚠️ Comunicación serial perdida");
      } else if (!was_serial_connected && serial_connected) {
      Serial.println("✅ Comunicación serial recuperada");
      }*/
  }
}

void cleanup_microros() {

  // limpiar todo sin verificar estados complejos

  // Limpiar executor
  rclc_executor_fini(&executor);

  // Limpiar timers
  if (odom_timer.impl) RCSOFTCHECK(rcl_timer_fini(&odom_timer));
  if (range_timer.impl) RCSOFTCHECK(rcl_timer_fini(&range_timer));
  if (sync_timer.impl) RCSOFTCHECK(rcl_timer_fini(&sync_timer));

  // Limpiar publishers
  if (odom_publisher.impl) RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
  if (imu_publisher.impl) RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
  if (range_front_publisher.impl) RCSOFTCHECK(rcl_publisher_fini(&range_front_publisher, &node));
  if (range_left_publisher.impl) RCSOFTCHECK(rcl_publisher_fini(&range_left_publisher, &node));
  if (range_right_publisher.impl) RCSOFTCHECK(rcl_publisher_fini(&range_right_publisher, &node));

  // Limpiar subscription
  if (subscriber.impl) RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));

  // Limpiar servicio
  if (set_bool_service.impl) RCSOFTCHECK(rcl_service_fini(&set_bool_service, &node));

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
