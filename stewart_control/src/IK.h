#include "IK.h"
#include <cmath>
#include <Eigen/Core>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

IK::IK()
{
    height = 2.0;
    // Define the base and platform positions for the Stewart platform
    b << -0.101,  0.8, 0.25, 1,
          0.101,  0.8, 0.25, 1,
          0.743, -0.313, 0.25, 1,
          0.642, -0.487, 0.25, 1,
         -0.643, -0.486, 0.25, 1,
         -0.744, -0.311, 0.25, 1;

    p << -0.642,  0.487, -0.05, 1,
          0.642,  0.487, -0.05, 1,
          0.743,  0.313, -0.05, 1,
          0.101, -0.8, -0.05, 1,
         -0.101, -0.8, -0.05, 1,
         -0.743,  0.313, -0.05, 1;

    // Initialize Float32MultiArray message with zeros
    for (int i = 0; i < 6; i++)
    {
        f32ma_msg.data.push_back(0);
    }
}

void IK::calculateIK(const std::string& type, double angle)
{
    // Implement your inverse kinematics calculations here
    // This is a placeholder function based on provided data
    // You need to implement the actual IK logic based on your Stewart platform requirements

    if (type == "roll")
    {
        // Implement roll IK calculation
        // Example implementation based on the provided data
        // Adjust according to your actual IK logic
        for (size_t i = 0; i < 6; i++)
        {
            // Placeholder calculation based on the angle
            f32ma_msg.data[i] = angle;
        }
    }
    else if (type == "pitch")
    {
        // Implement pitch IK calculation
        // Example implementation based on the provided data
        // Adjust according to your actual IK logic
        for (size_t i = 0; i < 6; i++)
        {
            // Placeholder calculation based on the angle
            f32ma_msg.data[i] = angle;
        }
    }
    else if (type == "yaw")
    {
        // Implement yaw IK calculation
        // Example implementation based on the provided data
        // Adjust according to your actual IK logic
        for (size_t i = 0; i < 6; i++)
        {
            // Placeholder calculation based on the angle
            f32ma_msg.data[i] = angle;
        }
    }
    else if (type == "home")
    {
        // Implement home position IK calculation
        // Example implementation based on the provided data
        // Adjust according to your actual IK logic
        for (size_t i = 0; i < 6; i++)
        {
            // Placeholder calculation for home position (0 angle)
            f32ma_msg.data[i] = 0.0;
        }
    }
    else
    {
        ROS_WARN("Unknown IK type received: %s", type.c_str());
    }

    // Publish the Float32MultiArray message
    pub.publish(f32ma_msg);
}

Eigen::Matrix4f IK::transformationMatrix(float x, float y, float z, float roll, float pitch, float yaw)
{
    // Implement your transformation matrix calculation here
    Eigen::Matrix4f T;
    T << cos(yaw)*cos(pitch), -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll), sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll), x,
         sin(yaw)*cos(pitch), cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll), -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll), y,
         -sin(pitch),          cos(pitch)*sin(roll),                                cos(pitch)*cos(yaw),                                 z,
         0,                    0,                                                   0,                                                      1;
    
    return T;
}

