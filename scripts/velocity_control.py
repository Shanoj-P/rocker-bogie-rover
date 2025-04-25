#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def perform_trajectory():
    rospy.init_node('velocity_controller',anonymous=False)
    controller_name = "/mobile_robot_controller/command"
    pub = rospy.Publisher(controller_name, JointTrajectory, queue_size=10)
    argv = sys.argv[1:]
    robot_joints = ['Revolute_25', 'Revolute_36']
    goal_pos = [float(argv[0]), float(argv[1])]

    rospy.loginfo('Goal position is set!')
    rospy.sleep(1)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = robot_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_pos
    trajectory_msg.points[0].velocities = [0.0 for i in robot_joints]
    trajectory_msg.points[0].accelerations = [0.5 for i in robot_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)
    pub.publish(trajectory_msg)

if __name__ == '__main__':
    perform_trajectory()