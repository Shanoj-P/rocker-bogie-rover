#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class VelocityController:
    def __init__(self):
        rospy.init_node('velocity_controller')

        # Robot parameters
        self.wheel_separation = 0.343  # Distance between left and right wheels
        self.wheel_radius = 0.065      # Radius of each wheel

        # Publishers to control the left and right wheels
        self.left_front_pub = rospy.Publisher('/left_front_wheel_velocity_controller/command', Float64, queue_size=10)
        self.left_middle_pub = rospy.Publisher('/left_middle_wheel_velocity_controller/command', Float64, queue_size=10)
        self.left_rear_pub = rospy.Publisher('/left_rear_wheel_velocity_controller/command', Float64, queue_size=10)

        self.right_front_pub = rospy.Publisher('/right_front_wheel_velocity_controller/command', Float64, queue_size=10)
        self.right_middle_pub = rospy.Publisher('/right_middle_wheel_velocity_controller/command', Float64, queue_size=10)
        self.right_rear_pub = rospy.Publisher('/right_rear_wheel_velocity_controller/command', Float64, queue_size=10)

        # Subscriber to the /cmd_vel topic
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        v_x = msg.linear.x  # Linear velocity (m/s)
        omega_z = msg.angular.z  # Angular velocity (rad/s)

        # Calculate wheel velocities based on kinematic equations
        v_left = v_x - (omega_z * self.wheel_separation / 2)
        v_right = v_x + (omega_z * self.wheel_separation / 2)

        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius
        rospy.loginfo('left speed: %2f, right speed: %2f'%(omega_left, omega_right))
        # Publish angular velocities to left and right wheels
        self.left_front_pub.publish(omega_left)
        self.left_middle_pub.publish(omega_left)
        self.left_rear_pub.publish(omega_left)

        self.right_front_pub.publish(omega_right)
        self.right_middle_pub.publish(omega_right)
        self.right_rear_pub.publish(omega_right)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    controller = VelocityController()
    controller.spin()
