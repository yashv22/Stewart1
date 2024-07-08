#include "stewart_control/stewart_control_node.h"
#include <std_msgs/String.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <cstdlib>
#include <ctime>

StewartControlNode::StewartControlNode()
{
    // Initialize the random seed
    std::srand(std::time(0));

    control_pub_ = nh_.advertise<std_msgs::String>("stewart/control_commands", 10);

    // Service client to apply body wrench
    apply_wrench_client_ = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
}

void StewartControlNode::generateRandomCommand()
{
    current_command_.x = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 2.0f - 1.0f; // Random roll between -1 and 1
    current_command_.y = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 2.0f - 1.0f; // Random pitch between -1 and 1
    current_command_.z = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * 2.0f - 1.0f; // Random heave between -1 and 1
}

void StewartControlNode::publishControlCommands()
{
    generateRandomCommand();
    
    gazebo_msgs::ApplyBodyWrench apply_wrench;
    apply_wrench.request.body_name = "stewart::base_link"; // Adjust the body name to match your model
    apply_wrench.request.reference_frame = "world";
    apply_wrench.request.wrench.force.z = current_command_.z; // Heave
    apply_wrench.request.start_time = ros::Time(0);
    apply_wrench.request.duration = ros::Duration(0.1);

    if (apply_wrench_client_.call(apply_wrench))
    {
        ROS_INFO("Applied heave command: %f", current_command_.z);
    }
    else
    {
        ROS_ERROR("Failed to apply wrench to Stewart platform");
    }

    // Optionally, handle roll and pitch using torque or another method
    // Here we just log the roll and pitch commands
    ROS_INFO("Generated random command: roll=%f, pitch=%f, heave=%f", current_command_.x, current_command_.y, current_command_.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stewart_control_node");
    StewartControlNode stewart_control_node;

    ros::Rate loop_rate(1);  // 1Hz (Change this value to adjust the frequency of the random command generation)
    while (ros::ok())
    {
        stewart_control_node.publishControlCommands();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

