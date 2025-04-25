#!/usr/bin/env python

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

# Path to the bag file
bag_file = "rover_data.bag"

# Initialize storage lists
time_stamps = []
joint_velocities = []
imu_time_stamps = []
imu_linear_acceleration = []
imu_angular_velocity = []
imu_orientation = []
cmd_vel_time_stamps = []
cmd_velocities = []
rover_positions = []

# Read data from rosbag
with rosbag.Bag(bag_file, "r") as bag:
    topics_available = set()
    for topic, msg, t in bag.read_messages():
        topics_available.add(topic)

        # Joint Velocities
        if topic == "/joint_states":
            time_stamps.append(t.to_sec())
            joint_velocities.append(msg.velocity)  # Collect all joint velocities

        # IMU Data
        elif topic == "/imu_data":
            imu_time_stamps.append(t.to_sec())
            imu_linear_acceleration.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            imu_angular_velocity.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            imu_orientation.append([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        # Commanded Velocity
        elif topic == "/cmd_vel":
            cmd_vel_time_stamps.append(t.to_sec())
            cmd_velocities.append([msg.linear.x, msg.linear.y, msg.angular.z])

        # Rover Position
        elif topic == "/gazebo/model_states":
            if "rover2" in msg.name:
                index = msg.name.index("rover2")
                position = msg.pose[index].position
                rover_positions.append([position.x, position.y])

print("Topics in bag file:", topics_available)

# Convert lists to NumPy arrays with proper checks
joint_velocities = np.array([np.array(v) for v in joint_velocities], dtype=object) if joint_velocities else np.array([])
imu_linear_acceleration = np.array(imu_linear_acceleration) if imu_linear_acceleration else np.array([])
imu_angular_velocity = np.array(imu_angular_velocity) if imu_angular_velocity else np.array([])
imu_orientation = np.array(imu_orientation) if imu_orientation else np.array([])
cmd_velocities = np.array(cmd_velocities) if cmd_velocities else np.array([])
rover_positions = np.array(rover_positions) if rover_positions else np.array([])

# Handle empty joint velocities correctly
if len(joint_velocities) > 0:
    max_len = max(len(v) for v in joint_velocities)
    joint_velocities = np.array([np.pad(v, (0, max_len - len(v)), 'constant', constant_values=np.nan) for v in joint_velocities])

# Plot Joint Velocities
if joint_velocities.size > 0:
    plt.figure(figsize=(10, 5))
    for i in range(joint_velocities.shape[1]):
        plt.plot(time_stamps, joint_velocities[:, i], label=f"Wheel {i+1}")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (rad/s)")
    plt.title("Joint Velocities Over Time")
    plt.legend()
    plt.grid()

# Plot IMU Linear Acceleration
if imu_linear_acceleration.size > 0:
    plt.figure(figsize=(10, 5))
    plt.plot(imu_time_stamps, imu_linear_acceleration[:, 0], label="Acc X")
    plt.plot(imu_time_stamps, imu_linear_acceleration[:, 1], label="Acc Y")
    plt.plot(imu_time_stamps, imu_linear_acceleration[:, 2], label="Acc Z")
    plt.xlabel("Time (s)")
    plt.ylabel("Linear Acceleration (m/s²)")
    plt.title("IMU Linear Acceleration")
    plt.legend()
    plt.grid()

# Plot IMU Angular Velocity
if imu_angular_velocity.size > 0:
    plt.figure(figsize=(10, 5))
    plt.plot(imu_time_stamps, imu_angular_velocity[:, 0], label="Angular Vel X")
    plt.plot(imu_time_stamps, imu_angular_velocity[:, 1], label="Angular Vel Y")
    plt.plot(imu_time_stamps, imu_angular_velocity[:, 2], label="Angular Vel Z")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.title("IMU Angular Velocity")
    plt.legend()
    plt.grid()

# Plot IMU Orientation (Quaternion)
if imu_orientation.size > 0:
    plt.figure(figsize=(10, 5))
    plt.plot(imu_time_stamps, imu_orientation[:, 0], label="Quat X")
    plt.plot(imu_time_stamps, imu_orientation[:, 1], label="Quat Y")
    plt.plot(imu_time_stamps, imu_orientation[:, 2], label="Quat Z")
    plt.plot(imu_time_stamps, imu_orientation[:, 3], label="Quat W")
    plt.xlabel("Time (s)")
    plt.ylabel("Quaternion Value")
    plt.title("IMU Orientation (Quaternion)")
    plt.legend()
    plt.grid()

# Plot Rover Path
if rover_positions.size > 0:
    plt.figure(figsize=(8, 8))
    plt.plot(rover_positions[:, 0], rover_positions[:, 1], marker="o", linestyle="-", label="Rover Path")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Rover Trajectory")
    plt.legend()
    plt.grid()

# Show all plots
plt.show()
