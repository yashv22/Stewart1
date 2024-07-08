#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Float64

class StewartPlatformController:
    def __init__(self):
        rospy.init_node('shaft_control', anonymous=True)
        
        # Publishers for piston shaft positions
        self.publishers = []
        for i in range(1, 7):
            topic_name = f'/piston{i}_shaft_link/command'
            pub = rospy.Publisher(topic_name, Float64, queue_size=10)
            self.publishers.append(pub)
        
        # Control parameters
        self.rate = rospy.Rate(10)  # Control loop rate in Hz

    def run(self):
        rospy.loginfo("Controlling pistons with random small changes.")
        
        while not rospy.is_shutdown():
            for i in range(6):
                # Generate random small increments for piston length
                length_change = random.uniform(-0.05, 0.05)  # Small random change

                # Publish updated length to the corresponding piston
                self.publishers[i].publish(length_change)
                rospy.loginfo(f"Set piston{i+1}_shaft_link to {length_change}")

            self.rate.sleep()  # Control rate

if __name__ == '__main__':
    try:
        controller = StewartPlatformController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

