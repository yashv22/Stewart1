#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64

# Define the names of the prismatic joints in the SDF file
JOINT_NAMES = [
    "piston1_prismatic_joint",
    "piston2_prismatic_joint",
    "piston3_prismatic_joint",
    "piston4_prismatic_joint",
    "piston5_prismatic_joint",
    "piston6_prismatic_joint"
]

# Publisher dictionary to control each prismatic joint
joint_publishers = {}

def command_callback(data):
    rospy.loginfo("Received command: Roll={}, Pitch={}, Yaw={}".format(data.orientation.x, data.orientation.y, data.orientation.z))
    
    # Ensure the received command is within valid range for pistons
    command_values = [
        data.orientation.x,
        data.orientation.y,
        data.orientation.z
    ]
    
    for i in range(min(len(command_values), len(JOINT_NAMES))):
        joint_name = JOINT_NAMES[i]
        command_value = command_values[i]
        
        if joint_name in joint_publishers:
            joint_publishers[joint_name].publish(command_value)
            rospy.loginfo("Published command to {}: {}".format(joint_name, command_value))
        else:
            rospy.logwarn("Joint {} not found in joint_publishers".format(joint_name))

if __name__ == '__main__':
    rospy.init_node('stewart_control_node', anonymous=True)
    
    rospy.Subscriber('/stewart/command_pose', Pose, command_callback)
    
    # Initialize publishers for each prismatic joint
    for joint_name in JOINT_NAMES:
        topic_name = '/stewart/{}_position_controller/command'.format(joint_name)
        joint_publishers[joint_name] = rospy.Publisher(topic_name, Float64, queue_size=10)
        rospy.loginfo("Initialized publisher for {}: {}".format(joint_name, topic_name))
    
    rospy.spin()

