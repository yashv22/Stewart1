#!/usr/bin/env python

import rospy
import pygame
from pygame.locals import *
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState
import math

# Parameters for controlling the movement
MAX_ROTATION_RATE = 0.1  # Maximum radians per second for rotation (adjust as needed)

class PlatformController:
    def __init__(self):
        rospy.init_node('platform_link_controller', anonymous=True)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_name = 'stewart'  # Adjusted model name
        self.current_orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Initial orientation (identity)

        pygame.init()
        pygame.display.set_mode((300, 300))
        pygame.display.set_caption("Control Platform Link (Press Arrow Keys)")
        self.clock = pygame.time.Clock()

    def run(self):
        running = True
        while running and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                elif event.type == KEYDOWN:
                    self.handle_key_event(event.key)

            # Update orientation smoothly
            self.publish_orientation()
            pygame.display.flip()
            self.clock.tick(30)  # Cap the frame rate to 30 FPS

        pygame.quit()

    def handle_key_event(self, key):
        # Adjust current orientation based on keyboard input
        if key == pygame.K_LEFT:
            self.current_orientation = self.update_orientation(0.0, 0.0, MAX_ROTATION_RATE)
        elif key == pygame.K_RIGHT:
            self.current_orientation = self.update_orientation(0.0, 0.0, -MAX_ROTATION_RATE)
        elif key == pygame.K_UP:
            self.current_orientation = self.update_orientation(0.0, MAX_ROTATION_RATE, 0.0)
        elif key == pygame.K_DOWN:
            self.current_orientation = self.update_orientation(0.0, -MAX_ROTATION_RATE, 0.0)

    def update_orientation(self, roll_rate, pitch_rate, yaw_rate):
        # Calculate new orientation based on rates and current orientation
        roll = self.current_orientation.x + roll_rate
        pitch = self.current_orientation.y + pitch_rate
        yaw = self.current_orientation.z + yaw_rate

        # Normalize yaw to be within [-pi, pi]
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        # Convert roll, pitch, yaw to quaternion
        quaternion = self.quaternion_from_euler(roll, pitch, yaw)
        return quaternion

    def publish_orientation(self):
        # Create a ModelState message
        state_msg = ModelState()
        state_msg.model_name = self.model_name
        state_msg.pose.position = Point(0.0, 0.0, 1.0)  # Set desired position (x, y, z)
        state_msg.pose.orientation = self.current_orientation

        # Call the set_model_state service
        try:
            resp = self.set_state(state_msg)
            rospy.loginfo("Set model state success")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def quaternion_from_euler(self, roll, pitch, yaw):
        # Convert roll, pitch, yaw to quaternion using ROS tf transformations
        import tf.transformations as tf
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

if __name__ == '__main__':
    try:
        controller = PlatformController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

