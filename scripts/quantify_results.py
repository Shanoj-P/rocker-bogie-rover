#!/usr/bin/env python3

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from scipy.spatial.distance import euclidean
import tf.transformations as tf_trans  # For quaternion to Euler conversion

# Path to the bag file
bag_file = "rover_data_rocky_terrain.bag"

# Initialize storage lists
time_stamps, joint_velocities = [], []
imu_time_stamps, imu_linear_acceleration = [], []
imu_angular_velocity, imu_orientation = [], []
cmd_vel_time_stamps, cmd_velocities = [], []
rover_positions = []

# Read the bag file
with rosbag.Bag(bag_file, "r") as bag:
    for topic, msg, t in bag.read_messages():
        if topic == "/joint_states":
            time_stamps.append(t.to_sec())
            joint_velocities.append(msg.velocity)

        elif topic == "/imu_data":
            imu_time_stamps.append(t.to_sec())
            imu_linear_acceleration.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            imu_angular_velocity.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            imu_orientation.append([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        elif topic == "/cmd_vel":
            cmd_vel_time_stamps.append(t.to_sec())
            cmd_velocities.append([msg.linear.x, msg.linear.y, msg.angular.z])

        elif topic == "/gazebo/model_states":
            if "rover2" in msg.name:
                index = msg.name.index("rover2")
                position = msg.pose[index].position
                rover_positions.append([position.x, position.y])

# Convert lists to NumPy arrays
joint_velocities = np.array([np.array(v) for v in joint_velocities], dtype=object)
imu_linear_acceleration = np.array(imu_linear_acceleration)
imu_angular_velocity = np.array(imu_angular_velocity)
imu_orientation = np.array(imu_orientation)
cmd_velocities = np.array(cmd_velocities)
rover_positions = np.array(rover_positions)

# Handle empty lists
if len(joint_velocities) > 0:
    max_len = max(len(v) for v in joint_velocities)
    joint_velocities = np.array([np.pad(v, (0, max_len - len(v)), 'constant', constant_values=np.nan) for v in joint_velocities])

# 🎯 Convert Quaternion to Euler Angles (Roll, Pitch, Yaw)
if len(imu_orientation) > 0:
    euler_angles = np.array([tf_trans.euler_from_quaternion(q) for q in imu_orientation])

# 🎯 Compute Mean, Max, and Min Values for Quantification
def compute_stats(data, name):
    mean_value = np.mean(data, axis=0)
    max_value = np.max(data, axis=0)
    min_value = np.min(data, axis=0)

    print(f"\n🔹 {name} Analysis:")
    print(f"  Mean: {mean_value}")
    print(f"  Max: {max_value}")
    print(f"  Min: {min_value}")

    return mean_value, max_value, min_value

# Compute Stats for Different Data
if len(joint_velocities) > 0:
    mean_joint_vel, max_joint_vel, min_joint_vel = compute_stats(joint_velocities, "Joint Velocity")

if len(imu_linear_acceleration) > 0:
    mean_accel, max_accel, min_accel = compute_stats(imu_linear_acceleration, "IMU Linear Acceleration")

if len(imu_angular_velocity) > 0:
    mean_ang_vel, max_ang_vel, min_ang_vel = compute_stats(imu_angular_velocity, "IMU Angular Velocity")

if len(euler_angles) > 0:
    mean_euler, max_euler, min_euler = compute_stats(euler_angles, "IMU Orientation (Euler Angles)")

if len(cmd_velocities) > 0:
    mean_cmd_vel, max_cmd_vel, min_cmd_vel = compute_stats(cmd_velocities, "Commanded Velocity")

# 📊 **Plot Joint Velocities with Annotations**
plt.figure(figsize=(10, 5))
for i in range(joint_velocities.shape[1]):
    plt.plot(time_stamps, joint_velocities[:, i], label=f"Wheel {i+1}")
    # plt.scatter(time_stamps[np.argmax(joint_velocities[:, i])], max_joint_vel[i], color='red', marker='o', label=f"Max (Wheel {i+1})")
    # plt.scatter(time_stamps[np.argmin(joint_velocities[:, i])], min_joint_vel[i], color='blue', marker='o', label=f"Min (Wheel {i+1})")

plt.xlabel("Time (s)")
plt.ylabel("Velocity (rad/s)")
plt.title("wheel Velocities Over Time")
plt.legend()
plt.grid()

# 📊 **Plot IMU Angular Velocity**
plt.figure(figsize=(10, 5))
plt.plot(imu_time_stamps, imu_angular_velocity[:, 0], label="Angular Velocity X")
plt.plot(imu_time_stamps, imu_angular_velocity[:, 1], label="Angular Velocity Y")
plt.plot(imu_time_stamps, imu_angular_velocity[:, 2], label="Angular Velocity Z")
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("IMU Angular Velocity Over Time")
plt.legend()
plt.grid()

# 📊 **Plot IMU Linear Acceleration**
plt.figure(figsize=(10, 5))
plt.plot(imu_time_stamps, imu_linear_acceleration[:, 0], label="Acc X")
plt.plot(imu_time_stamps, imu_linear_acceleration[:, 1], label="Acc Y")
plt.plot(imu_time_stamps, imu_linear_acceleration[:, 2], label="Acc Z")
plt.xlabel("Time (s)")
plt.ylabel("Linear Acceleration (m/s²)")
plt.title("IMU Linear Acceleration Over Time")
plt.legend()
plt.grid()

# 📊 **Plot IMU Orientation (Euler Angles)**
plt.figure(figsize=(10, 5))
plt.plot(imu_time_stamps, euler_angles[:, 0], label="Roll")
plt.plot(imu_time_stamps, euler_angles[:, 1], label="Pitch")
plt.plot(imu_time_stamps, euler_angles[:, 2], label="Yaw")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.title("IMU Orientation (Euler Angles) Over Time")
plt.legend()
plt.grid()

# 📊 **Plot Commanded Velocity**
plt.figure(figsize=(10, 5))
plt.plot(cmd_vel_time_stamps, cmd_velocities[:, 0], label="Linear Velocity")
# plt.plot(cmd_vel_time_stamps, cmd_velocities[:, 1], label="Linear Velocity Y")
plt.plot(cmd_vel_time_stamps, cmd_velocities[:, 2], label="Angular Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity")
plt.title("Commanded Velocities Over Time")
plt.legend()
plt.grid()

# 📊 **Plot Rover Path with Distance & Average Speed**
# 📊 **Plot Rover Path with Distance & Average Speed**
if len(rover_positions) > 1:
    total_distance = sum(euclidean(rover_positions[i], rover_positions[i+1]) for i in range(len(rover_positions)-1))
    time_duration = 30  # Fixed time duration
    avg_speed = total_distance / time_duration  # Compute average speed

    print(f"\n🔹 Rover Path Analysis: Total Distance Traveled = {total_distance:.3f} meters")
    print(f"🔹 Average Speed (Fixed Time = 30s) = {avg_speed:.3f} m/s")

    plt.figure(figsize=(8, 8))
    plt.plot(rover_positions[:, 0], rover_positions[:, 1], marker="o", linestyle="-", label="Rover Path")
    plt.scatter(rover_positions[0, 0], rover_positions[0, 1], color='green', marker='o', s=100, label="Start")
    plt.scatter(rover_positions[-1, 0], rover_positions[-1, 1], color='red', marker='o', s=100, label="End")

    # Add annotation for average speed
    plt.text(rover_positions[len(rover_positions) // 2, 0], 
             rover_positions[len(rover_positions) // 2, 1], 
             f"Avg Speed: {avg_speed:.3f} m/s", 
             fontsize=12, color="blue", bbox=dict(facecolor='white', alpha=0.6))

    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title(f"Rover Trajectory (Distance: {total_distance:.3f} m, Avg Speed: {avg_speed:.3f} m/s)")
    plt.legend()
    plt.grid()

# 📊 **Plot Speed vs Time with Average Speed**
# Ensure there are at least two positions to compute speeds
# Ensure there are at least two timestamps and positions to compute speeds
# if len(rover_positions) > 1 and len(time_stamps) > 1:
#     speeds = []
#     speed_time_stamps = []
    
#     # Find the shorter length to avoid index mismatch
#     min_length = min(len(rover_positions), len(time_stamps))

#     for i in range(min_length - 1):  # Ensure valid index range
#         dt = time_stamps[i+1] - time_stamps[i]  # Time difference

#         if dt > 0:  # Avoid division by zero
#             speed = euclidean(rover_positions[i], rover_positions[i+1]) / dt
#             speeds.append(speed)
#             speed_time_stamps.append(time_stamps[i])  # Align timestamps

#     # Compute average speed
#     avg_speed = total_distance / 30  # Fixed time duration

#     # 📊 **Plot Speed vs Time**
#     plt.figure(figsize=(10, 5))
#     plt.plot(speed_time_stamps, speeds, label="Instantaneous Speed", color="blue")
#     plt.axhline(y=avg_speed, color="red", linestyle="--", label=f"Avg Speed: {avg_speed:.3f} m/s")

#     plt.xlabel("Time (s)")
#     plt.ylabel("Speed (m/s)")
#     plt.title("Rover Speed vs Time")
#     plt.legend()
#     plt.grid()

# else:
#     print("❌ Not enough rover position or timestamp data to compute speed.")

# Show all plots
plt.show()

