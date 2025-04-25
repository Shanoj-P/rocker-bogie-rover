#!/usr/bin/env python
# import rospy
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# def send_velocity_command():
#     rospy.init_node('send_velocity_command')
#     pub = rospy.Publisher('/mobile_robot_controller/command', JointTrajectory, queue_size=10)
#     rospy.sleep(1)  # Wait for publisher to initialize

#     # Create a JointTrajectory message
#     trajectory = JointTrajectory()
#     trajectory.joint_names = ["wheel_1_joint", "wheel_2_joint", "wheel_3_joint", "wheel_4_joint", "wheel_5_joint", "wheel_6_joint"]  # Replace with actual joint names

#     # Define a single point with desired velocities for each joint
#     point = JointTrajectoryPoint()
    
#     point.velocities = [-10, -10, -10, -10, -10, -10]  # Desired velocities (in rad/s) for each joint
#     point.positions = point.velocities
#     # point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
#     point.time_from_start = rospy.Duration(1.0)  # Time to reach these velocities

#     trajectory.points.append(point)

#     # Publish the command
#     rospy.loginfo("Publishing velocity command")
#     pub.publish(trajectory)

#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         send_velocity_command()
#     except rospy.ROSInterruptException:
#         pass






import rospy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64

class VelocityController:
    def __init__(self):
        rospy.init_node('send_velocity_command')

        # Robot parameters
        self.wheel_separation = 0.343  # Distance between left and right wheels
        self.wheel_radius = 0.065      # Radius of each wheel

        # Publishers to control the left and right wheels
        self.pub = rospy.Publisher('/mobile_robot_controller/command', JointTrajectory, queue_size=10)

        # Subscriber to the /cmd_vel topic
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = ["wheel_1_joint", "wheel_2_joint", "wheel_3_joint", "wheel_4_joint", "wheel_5_joint", "wheel_6_joint"]  # Replace with actual joint names

        # Define a single point with desired velocities for each joint
        

    def cmd_vel_callback(self, msg):
        v_x = msg.linear.x  # Linear velocity (m/s)
        omega_z = msg.angular.z  # Angular velocity (rad/s)

        # Calculate wheel velocities based on kinematic equations
        v_left = v_x - (omega_z * self.wheel_separation / 2)
        v_right = v_x + (omega_z * self.wheel_separation / 2)

        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        self.point = JointTrajectoryPoint()
        # Publish angular velocities to left and right wheels
        self.point.velocities = [omega_left, omega_left, omega_left, omega_right, omega_right, omega_right]  # Desired velocities (in rad/s) for each joint
        self.point.positions = [0]*6
        self.point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
        self.point.time_from_start = rospy.Duration(1.0)  # Time to reach these velocities
        self.trajectory.points.clear() 
        self.trajectory.points.append(self.point)
        rospy.loginfo(self.point.velocities)
        self.pub.publish(self.trajectory)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    controller = VelocityController()
    controller.spin()
