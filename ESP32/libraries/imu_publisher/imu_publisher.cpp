#include "imu_publisher.h"

ImuPublisher::ImuPublisher()
{
    imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");

    // Default: zero orientation and angular/linear acceleration
    //imu_msg_.orientation_covariance[0] = -1.0; // if orientation is not used
    
    // Definir matriz de covarianza para la orientación en cuaternios(X, Y, Z)
    double orientation_covariance[9] = {
        0.01, 0.0,  0.0,  
        0.0,  0.01, 0.0,  
        0.0,  0.0,  0.01
    };
    memcpy(imu_msg_.orientation_covariance, orientation_covariance, sizeof(orientation_covariance));    // Copiar usando memcpy
    
    
    // Definir matriz de covarianza para la velocidad angular en (X, Y, Z)
    double angular_velocity_covariance[9] = {
        0.02, 0.0,  0.0,  
        0.0,  0.02, 0.0,  
        0.0,  0.0,  0.02
    };
    memcpy(imu_msg_.angular_velocity_covariance, angular_velocity_covariance, sizeof(angular_velocity_covariance));

    
    // Definir matriz de covarianza para la aceleracion lineal en (X, Y, Z)
    double linear_acceleration_covariance[9] = {
        0.03, 0.0,  0.0,  
        0.0,  0.03, 0.0,  
        0.0,  0.0,  0.03
    };
    memcpy(imu_msg_.linear_acceleration_covariance, linear_acceleration_covariance, sizeof(linear_acceleration_covariance));
    
}

void ImuPublisher::update(
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float roll, float pitch, float yaw)
{
    float q[4];
    euler_to_quat(roll, pitch, yaw, q);

    imu_msg_.orientation.x = q[1];
    imu_msg_.orientation.y = q[2];
    imu_msg_.orientation.z = q[3];
    imu_msg_.orientation.w = q[0];

    imu_msg_.angular_velocity.x = gx;
    imu_msg_.angular_velocity.y = gy;
    imu_msg_.angular_velocity.z = gz;

    imu_msg_.linear_acceleration.x = ax;
    imu_msg_.linear_acceleration.y = ay;
    imu_msg_.linear_acceleration.z = az;
}

sensor_msgs__msg__Imu ImuPublisher::getData()
{
    return imu_msg_;
}

void ImuPublisher::euler_to_quat(float roll, float pitch, float yaw, float* q)
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;

    norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (int i = 0; i < 4; ++i) {
      q[i] /= norm;
    }
}