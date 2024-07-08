#ifndef STEWART_CONTROL_NODE_H
#define STEWART_CONTROL_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ApplyBodyWrench.h> // Include the necessary header

class StewartControlNode
{
public:
    StewartControlNode();

    void commandCallback(const geometry_msgs::Vector3::ConstPtr& msg);

    void publishControlCommands();

    void generateRandomCommand(); // Declare generateRandomCommand function

private:
    ros::NodeHandle nh_;
    ros::Subscriber command_sub_;
    ros::Publisher control_pub_;
    ros::ServiceClient apply_wrench_client_; // Declare apply_wrench_client_
    geometry_msgs::Vector3 current_command_;
};

#endif // STEWART_CONTROL_NODE_H

