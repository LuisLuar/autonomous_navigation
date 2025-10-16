//microros_configuration.ino
#include <WiFi.h>  // ← Nueva librería

//____________INTERNET_________________
const char* ssid = "OMEN";
const char* password = "12345678";
const char* agent_ip = "10.42.0.1";  // ← IP de tu PC (donde corre el agent)

const uint32_t agent_port = 8888;
//___________________________


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_subscription_t subscriber;
rcl_subscription_t reset_subscriber;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t range1_publisher;
rcl_publisher_t range2_publisher;
rcl_publisher_t range3_publisher;
rcl_publisher_t reset_publisher;

geometry_msgs__msg__Twist msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;

sensor_msgs__msg__Range range1_msg;
sensor_msgs__msg__Range range2_msg;
sensor_msgs__msg__Range range3_msg;

std_msgs__msg__Bool sub_msg;
std_msgs__msg__Bool pub_msg;

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
  RCCHECK(rclc_node_init_default(&node, "Robot_Control","", &support));

  // 4. Crear suscriptor para el servicio
  RCCHECK(rclc_subscription_init_default(&reset_subscriber, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),       "reset/request_srv"));
  RCCHECK(rclc_subscription_init_default(&subscriber,       &node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  
  RCCHECK(rclc_publisher_init_default(&reset_publisher, &node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),     "reset/response_srv"));
  RCCHECK(rclc_publisher_init_default(&odom_publisher,  &node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered"));
  RCCHECK(rclc_publisher_init_default(&imu_publisher,   &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),   "imu/unfiltered"));

  //create a range1 publisher
  RCCHECK(rclc_publisher_init_default(&range1_publisher,  &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),"range1/filtered"));
  RCCHECK(rclc_publisher_init_default(&range2_publisher,  &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),"range2/filtered"));
  RCCHECK(rclc_publisher_init_default(&range3_publisher,  &node,ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),"range3/filtered"));
         
  RCCHECK(rclc_timer_init_default(&odom_timer,  &support,RCL_MS_TO_NS(50),publish_odom));
  RCCHECK(rclc_timer_init_default(&range_timer, &support,RCL_MS_TO_NS(60),publish_ranges_all));
  RCCHECK(rclc_timer_init_default(&sync_timer,  &support,RCL_MS_TO_NS(120000),sync_timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &reset_subscriber, &sub_msg, &reset_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &range_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));

  delay(1000);
  syncTime();

  //________mensaje fijo______________
  // Mensaje fijo range 1
  range1_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range1_msg.field_of_view = 0.52;
  range1_msg.min_range = 0.04;
  range1_msg.max_range = 0.5;
  micro_ros_string_utilities_set(range1_msg.header.frame_id, "range1_link");

   // Mensaje fijo range 1
  range2_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range2_msg.field_of_view = 0.52;
  range2_msg.min_range = 0.04;
  range2_msg.max_range = 0.5;
  micro_ros_string_utilities_set(range2_msg.header.frame_id, "range2_link");

   // Mensaje fijo range 1
  range3_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range3_msg.field_of_view = 0.52;
  range3_msg.min_range = 0.04;
  range3_msg.max_range = 0.5;
  micro_ros_string_utilities_set(range3_msg.header.frame_id, "range3_link");
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
  
  odometry.update(v,w,x_pos,y_pos,yaw_enc);
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
 
  // Range1
  struct timespec time_stamp = getTime();
  range1_msg.header.stamp.sec = time_stamp.tv_sec;
  range1_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  range1_msg.range = sensor1.distAnalog() / 100.0;
  RCSOFTCHECK(rcl_publish(&range1_publisher, &range1_msg, NULL));
  
  // Range2
  time_stamp = getTime();
  range2_msg.header.stamp.sec = time_stamp.tv_sec;
  range2_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  range2_msg.range = sensor2.distAnalog() / 100.0;
  RCSOFTCHECK(rcl_publish(&range2_publisher, &range2_msg, NULL));

  // Range3
  time_stamp = getTime();
  range3_msg.header.stamp.sec = time_stamp.tv_sec;
  range3_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  range3_msg.range = sensor3.distAnalog() / 100.0;
  RCSOFTCHECK(rcl_publish(&range3_publisher, &range3_msg, NULL));
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
void reset_callback(const void *msg_in)
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
}

//_________________________________________________
