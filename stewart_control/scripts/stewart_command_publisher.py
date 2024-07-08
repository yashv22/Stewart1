#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import random

def stewart_command_publisher():
    rospy.init_node('stewart_command_publisher', anonymous=True)
    command_pub = rospy.Publisher('/stewart/command_pose', Pose, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        command = Pose()
        command.orientation.x = random.uniform(-1.0, 1.0)  # Roll
        command.orientation.y = random.uniform(-1.0, 1.0)  # Pitch
        command.orientation.z = random.uniform(-1.0, 1.0)  # Yaw

        command_pub.publish(command)
        rospy.loginfo("Published command: Roll={}, Pitch={}, Yaw={}".format(command.orientation.x, command.orientation.y, command.orientation.z))

        rate.sleep()

if __name__ == '__main__':
    try:
        stewart_command_publisher()
    except rospy.ROSInterruptException:
        pass

