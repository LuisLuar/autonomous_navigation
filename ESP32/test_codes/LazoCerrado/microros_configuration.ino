//microros_configuration.ino
#include <WiFi.h>  // ← Nueva librería

//____________INTERNET_________________
const char* ssid = "OMEN";
const char* password = "12345678";
const char* agent_ip = "10.42.0.1";  // ← IP de tu PC (donde corre el agent)

/*const char* ssid = "Fastnett-Fibra-ConstructoraVasqu";
  const char* password = "1706312434";
  const char* agent_ip = "192.168.100.98";*/

const uint32_t agent_port = 8888;
//___________________________

//___________variables de seguridad_______________
unsigned long last_agent_check = 0;
const unsigned long AGENT_TIMEOUT_MS = 10000;  // 10 s sin comunicación
bool agent_connected = true;
//_________________________________________________


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t subscriber;
rcl_publisher_t odom_publisher;

geometry_msgs__msg__Twist msg;
nav_msgs__msg__Odometry odom_msg;

rcl_timer_t odom_timer;
rcl_timer_t sync_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;


// add time difference between uC time and ROS time to
// synchronize time with ROS
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

    ST.motor(1, 0);
    ST.motor(2, 0);

    digitalWrite(led_error, !digitalRead(led_error));
    Serial.println("error..");
    delay(500);


    if (millis() - last_error_time > 3000) ESP.restart();
  }
}


void beginMicroros() {
  //______MICROROS_________
  //set_microros_transports();
  //delay(2000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado a WiFi");

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
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "Robot_Control", "", &support));

  // 4. Crear suscriptor para el servicio
  RCCHECK(rclc_subscription_init_default(&subscriber,       &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_publisher_init_default(&odom_publisher,  &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered"));

  RCCHECK(rclc_timer_init_default(&odom_timer,  &support, RCL_MS_TO_NS(50), publish_odom));
  RCCHECK(rclc_timer_init_default(&sync_timer,  &support, RCL_MS_TO_NS(120000), sync_timer_callback));


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));
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


//___________________ODOMETRIA________________________________________
void publish_odom(rcl_timer_t* timer, int64_t last_call_time) {
  struct timespec time_stamp = getTime();

  //odometry.update(v,w,x_pos,y_pos,yaw_enc);
  //(Velocidad en X, Velocidad angular en Z, Posición en X, Posición en Y, Yaw desde encoder)bienderech
  odometry.update(Right.w, Left.w, 0, 0, 0);

  odom_msg = odometry.getData();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}


//__________________VELOCIDAD DE CADA RUEDA_______________---
void subscription_callback(const void* msgin) {
  prev_cmd_time = millis();
  int control_right = (int)msg.linear.x;
  int control_left = (int)msg.angular.z;

  Right.wRef = control_right; //rad/s
  Left.wRef = control_left; //rad/s
}

//___________________SINCRONIZACIÓN DEL TIEMPO________________________________________
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  syncTime();
}

//_________________________________________________________
void watchdog() {
  if (millis() - last_agent_check > 3000) {
    last_agent_check = millis();
    rcl_ret_t ret = rmw_uros_ping_agent(100, 1);
    if (ret != RCL_RET_OK) {
      if (agent_connected) {
        Serial.println("⚠️ Agente micro-ROS no responde.");
        agent_connected = false;
      }
    } else {
      if (!agent_connected) {
        Serial.println("✅ Conexión con agente restaurada.");
      }
      agent_connected = true;
    }

    // Si se pierde por más de 10s → modo seguro
    static unsigned long disconnected_since = 0;
    if (!agent_connected) {
      if (disconnected_since == 0)
        disconnected_since = millis();

      if (millis() - disconnected_since > AGENT_TIMEOUT_MS) {
        Serial.println("🚨 Conexión perdida >10s. Activando modo seguro...");
        error_loop();
      }
    } else {
      disconnected_since = 0;
    }
  }
}
