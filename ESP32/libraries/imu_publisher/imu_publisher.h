#ifndef IMU_PUBLISHER_H
#define IMU_PUBLISHER_H

#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/imu.h>
#include <micro_ros_utilities/string_utilities.h>
#include <math.h>


class ImuPublisher
{
    private:
        float norm; 
        sensor_msgs__msg__Imu imu_msg_;

        void euler_to_quat(float roll, float pitch, float yaw, float* q);

    public:
        ImuPublisher();

        void update(
            float ax, float ay, float az,
            float gx, float gy, float gz,
            float roll = 0.0f, float pitch = 0.0f, float yaw = 0.0f
        );

        sensor_msgs__msg__Imu getData();
};

#endif
