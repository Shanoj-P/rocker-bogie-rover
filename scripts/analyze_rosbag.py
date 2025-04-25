#!/usr/bin/env python

import rosbag
import pandas as pd
import matplotlib.pyplot as plt

# Set your rosbag file name
bag_file = "manipulator_test.bag"

# Initialize lists to store data
time_stamps = []
joint_names = []
joint_positions = []
joint_velocities = []
joint_efforts = []

# Open the rosbag file
print("Opening rosbag file...")
with rosbag.Bag(bag_file, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=["/joint_states"]):
        time_stamps.append(t.to_sec())
        joint_positions.append(msg.position)
        joint_velocities.append(msg.velocity)
        joint_efforts.append(msg.effort)
        if not joint_names:
            joint_names = msg.name  # Store joint names once

# Convert to DataFrame
df = pd.DataFrame(joint_positions, columns=joint_names)
df["time"] = time_stamps
df.set_index("time", inplace=True)

# Plot joint positions over time
plt.figure(figsize=(10, 6))
for joint in joint_names:
    plt.plot(df.index, df[joint], label=joint)
plt.xlabel("Time (s)")
plt.ylabel("Joint Position (rad)")
plt.title("Joint Positions Over Time")
plt.legend()
plt.grid()
plt.show()

# Plot joint velocities over time
plt.figure(figsize=(10, 6))
for i, joint in enumerate(joint_names):
    plt.plot(df.index, [vel[i] for vel in joint_velocities], label=joint)
plt.xlabel("Time (s)")
plt.ylabel("Joint Velocity (rad/s)")
plt.title("Joint Velocities Over Time")
plt.legend()
plt.grid()
plt.show()

# Plot joint efforts over time
plt.figure(figsize=(10, 6))
for i, joint in enumerate(joint_names):
    plt.plot(df.index, [eff[i] for eff in joint_efforts], label=joint)
plt.xlabel("Time (s)")
plt.ylabel("Joint Effort (Nm)")
plt.title("Joint Efforts Over Time")
plt.legend()
plt.grid()
plt.show()
