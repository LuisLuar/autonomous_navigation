# ObjectInfoArray.msg
std_msgs/Header header
ObjectInfo[] objects
float32 processing_time_ms
uint32 num_static_objects
uint32 num_moving_objects
float32 robot_speed       # velocidad actual del robot (m/s)