#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def publish_command(topic_name, command_value):
    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    rospy.init_node('stewart_command_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        pub.publish(command_value)
        rospy.loginfo("Publishing command: %s = %f", topic_name, command_value)
        rate.sleep()

if __name__ == '__main__':
    try:
        topic = rospy.get_param('~topic', '/stewart/roll_cmd')  # Default topic
        value = rospy.get_param('~value', 0.0)  # Default value

        rospy.loginfo("Publishing to topic: %s, value: %f", topic, value)
        publish_command(topic, value)
    except rospy.ROSInterruptException:
        pass


