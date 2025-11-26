//microros_configuration.ino
#include <WiFi.h>  // ← Nueva librería

//____________INTERNET_________________
/*const char* ssid = "OMEN";
const char* password = "12345678";
const char* agent_ip = "10.42.0.1";  // ← IP de tu PC (donde corre el agent)*/

const char* ssid = "Fastnett-Fibra-ConstructoraVasqu";
const char* password = "1706312434";
const char* agent_ip = "192.168.100.169";
const uint32_t agent_port = 8888;
//___________________________

//___________variables de seguridad_______________
unsigned long last_agent_check = 0;
const unsigned long AGENT_TIMEOUT_MS = 2000;  // 10 s sin comunicación
bool agent_connected = true;
//_________________________________________________


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_service_t set_bool_service;

rcl_subscription_t subscriber;
//rcl_subscription_t reset_subscriber;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t range_front_publisher;
rcl_publisher_t range_left_publisher;
rcl_publisher_t range_right_publisher;
//rcl_publisher_t reset_publisher;
std_srvs__srv__SetBool_Request set_bool_req;
std_srvs__srv__SetBool_Response set_bool_res;

geometry_msgs__msg__Twist msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;

sensor_msgs__msg__Range range_front_msg;
sensor_msgs__msg__Range range_left_msg;
sensor_msgs__msg__Range range_right_msg;

//std_msgs__msg__Bool sub_msg;
//std_msgs__msg__Bool pub_msg;

rcl_timer_t odom_timer;
rcl_timer_t range_timer;
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
  RCCHECK(rclc_node_init_default(&node, "Robot_Control","", &support));

  // 4. Crear suscriptor para el servicio
  // Crear servicio booleano
  RCCHECK(rclc_service_init_default(&set_bool_service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "/robot_control_reset"));
  //RCCHECK(rclc_subscription_init_default(&reset_subscriber, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),       "reset/request_srv"));
  RCCHECK(rclc_subscription_init_default(&subscriber,       &node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  
  //RCCHECK(rclc_publisher_init_default(&reset_publisher, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),     "reset/response_srv"));
  RCCHECK(rclc_publisher_init_default(&odom_publisher,  &node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered"));
  RCCHECK(rclc_publisher_init_default(&imu_publisher,   &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),   "imu/unfiltered"));

  //create a range_front publisher
  RCCHECK(rclc_publisher_init_default(&range_front_publisher,  &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),"range/front"));
  RCCHECK(rclc_publisher_init_default(&range_left_publisher,  &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),"range/left"));
  RCCHECK(rclc_publisher_init_default(&range_right_publisher,  &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),"range/right"));
         
  RCCHECK(rclc_timer_init_default(&odom_timer,  &support,RCL_MS_TO_NS(50),publish_odom));
  RCCHECK(rclc_timer_init_default(&range_timer, &support,RCL_MS_TO_NS(200),publish_ranges_all));
  RCCHECK(rclc_timer_init_default(&sync_timer,  &support,RCL_MS_TO_NS(120000),sync_timer_callback));

  

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  //RCCHECK(rclc_executor_add_subscription(&executor, &reset_subscriber, &sub_msg, &reset_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &range_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));
  RCCHECK(rclc_executor_add_service(&executor, &set_bool_service, &set_bool_req, &set_bool_res, set_bool_callback));

  delay(1000);
  syncTime();

  //________mensaje fijo______________
  // Mensaje fijo range 1
  range_front_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range_front_msg.field_of_view = 0.52;
  range_front_msg.min_range = 0.04;
  range_front_msg.max_range = 0.5;
  micro_ros_string_utilities_set(range_front_msg.header.frame_id, "range_front_link");

   // Mensaje fijo range 1
  range_left_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range_left_msg.field_of_view = 0.52;
  range_left_msg.min_range = 0.04;
  range_left_msg.max_range = 0.5;
  micro_ros_string_utilities_set(range_left_msg.header.frame_id, "range_left_link");

   // Mensaje fijo range 1
  range_right_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range_right_msg.field_of_view = 0.52;
  range_right_msg.min_range = 0.04;
  range_right_msg.max_range = 0.5;
  micro_ros_string_utilities_set(range_right_msg.header.frame_id, "range_right_link");
  //_________________________________________________________________

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
  
  odometry.update(vx,wz,x_pos,y_pos,yaw_enc);
  imu_pub.update(ax, ay, az, gx, gy, gz, roll_imu, pitch_imu, yaw_imu);

  odom_msg = odometry.getData();
  imu_msg = imu_pub.getData();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

  imu_msg.header.stamp.sec = time_stamp.tv_sec;
  imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

//____________________RANGE______________________________________________-
void publish_ranges_all(rcl_timer_t * timer, int64_t last_call_time) {
 
  // range_front
  struct timespec time_stamp = getTime();
  range_front_msg.header.stamp.sec = time_stamp.tv_sec;
  range_front_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  range_front_msg.range = sensor1.distAnalog() / 100.0;
  RCSOFTCHECK(rcl_publish(&range_front_publisher, &range_front_msg, NULL));
  
  // range_left
  time_stamp = getTime();
  range_left_msg.header.stamp.sec = time_stamp.tv_sec;
  range_left_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  range_left_msg.range = sensor2.distAnalog() / 100.0;
  RCSOFTCHECK(rcl_publish(&range_left_publisher, &range_left_msg, NULL));

  // range_right
  time_stamp = getTime();
  range_right_msg.header.stamp.sec = time_stamp.tv_sec;
  range_right_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  range_right_msg.range = sensor3.distAnalog() / 100.0;
  RCSOFTCHECK(rcl_publish(&range_right_publisher, &range_right_msg, NULL));
}


//__________________VELOCIDAD LINEAL Y ANGULAR DEL ROBOT_______________---
void subscription_callback(const void* msgin) {
  prev_cmd_time = millis();
  float linear = msg.linear.x;
  float angular = msg.angular.z;

  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  float vI = linear - (angular * L) / 2;
  float vD = linear + (angular * L) / 2;

  Left.wRef = (2 * vI) / D;
  Right.wRef = (2 * vD) / D;
}

//___________________SINCRONIZACIÓN DEL TIEMPO________________________________________
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  syncTime();
}

//__________________RESET DE VALORES INTEGRADOS______________________________________________-
/*void reset_callback(const void *msg_in)
{
  const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msg_in;
   // Imprimir el valor real
  Serial.printf("Booleano recibido: %s\n", msg->data ? "true" : "false");
  if(msg->data){
    roll_imu = 0, pitch_imu = 0, yaw_imu = 0; //datos de la imu
    x_pos = 0.0, y_pos = 0.0, yaw_enc= 0.0; //datos del encoder
    v = 0, w = 0; //datos del robot
    publish_bool(true);
  }else{
    Serial.println("Error al resetear: valor falso recibido");
    publish_bool(false);
  }
}


// Función para publicar el booleano
void publish_bool(bool value) {
  pub_msg.data = value;
  //RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  //rcl_publish(&reset_publisher, &pub_msg, NULL);

  // Cambia por:
  rcl_ret_t ret = rcl_publish(&reset_publisher, &pub_msg, NULL);
  if (ret != RCL_RET_OK) {
      // Manejar el error según necesites
      Serial.printf("Error publishing bool: %d\n", ret);
  }
  
  Serial.printf("Booleano publicado: %s\n", value ? "true" : "false");
}*/
// Callback del servicio
void set_bool_callback(const void * req, void * res) {
  std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) req;
  std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) res;
  
  // Aquí procesas la solicitud booleana
  bool requested_state = req_in->data;
  Serial.println(requested_state);
  
  if (requested_state) {
    // Acción cuando es true
    roll_imu = 0, pitch_imu = 0, yaw_imu = 0; //datos de la imu
    x_pos = 0.0, y_pos = 0.0, yaw_enc= 0.0; //datos del encoder
    
    res_in->success = true;
    static char success_msg1[] = "Estado activado exitosamente";
    res_in->message.data = success_msg1;
    res_in->message.size = strlen(res_in->message.data);
    
  } else {
    // Acción cuando es false
    res_in->success = true;
    static char success_msg2[] = "Estado desactivado exitosamente";
    res_in->message.data = success_msg2;
    res_in->message.size = strlen(res_in->message.data);
  }
}
//_________________________________________________

//_________________________________________________________
void watchdog(){
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
