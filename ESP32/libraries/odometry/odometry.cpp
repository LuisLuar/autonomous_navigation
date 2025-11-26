// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "odometry.h"

Odometry::Odometry():
    x_pos_(0.0),
    y_pos_(0.0),
    heading_(0.0)
{
    odom_msg_.header.frame_id = micro_ros_string_utilities_set(odom_msg_.header.frame_id, "odom");
    odom_msg_.child_frame_id = micro_ros_string_utilities_set(odom_msg_.child_frame_id, "base_footprint");
    
    // Definir matriz de covarianza para la posicion (X, Y, Z) y orientación en cuaternios(X, Y, Z)
    double pose_covariance[36] = {
        0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.01
    };
    
    // Copiar usando memcpy
    memcpy(odom_msg_.pose.covariance, pose_covariance, sizeof(pose_covariance));
    
        // Definir matriz de covarianza para la velocidad lineal (X, Y, Z) y la velocidad angular(X, Y, Z)
    double twist_covariance[36] = {
        0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.01
    };
    
    // Copiar usando memcpy
    memcpy(odom_msg_.twist.covariance, twist_covariance, sizeof(twist_covariance));
}

void Odometry::update(float linear_vel_x, float angular_vel_z, float x_pos_, float y_pos_, float heading_ )
{
    
    //calcular la dirección del robot en ángulo de cuaternión
    float q[4];
    euler_to_quat(0, 0, heading_, q);

    //Posición del robot en X, Y, Z
    odom_msg_.pose.pose.position.x = x_pos_;
    odom_msg_.pose.pose.position.y = y_pos_;
    odom_msg_.pose.pose.position.z = 0.0;

    //Dirección del robot en cuaternión
    odom_msg_.pose.pose.orientation.x = (double) q[1];
    odom_msg_.pose.pose.orientation.y = (double) q[2];
    odom_msg_.pose.pose.orientation.z = (double) q[3];
    odom_msg_.pose.pose.orientation.w = (double) q[0];
    
    //velocidad lineal a partir de los encoders
    odom_msg_.twist.twist.linear.x = linear_vel_x;
    odom_msg_.twist.twist.linear.y = 0.0;
    odom_msg_.twist.twist.linear.z = 0.0;

    //velocidad angular a partir de los encoders
    odom_msg_.twist.twist.angular.x = 0.0;
    odom_msg_.twist.twist.angular.y = 0.0;
    odom_msg_.twist.twist.angular.z = angular_vel_z;
}

nav_msgs__msg__Odometry Odometry::getData()
{
    return odom_msg_;
} 

const void Odometry::euler_to_quat(float roll, float pitch, float yaw, float* q) 
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