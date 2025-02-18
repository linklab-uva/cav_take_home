import rclpy, rclpy.time, rclpy.duration, deepracing_ros
import rpyutils

with rpyutils.add_dll_directories_from_env("PATH"):
    import rosbag2_py
import argparse
import typing
from rclpy.time import Time
from typing import List
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import deepracing_ros.utils.rosbag_utils as rosbag_utils

from geometry_msgs.msg import PointStamped, Point, Vector3Stamped, Vector3
from vrxperience_msgs.msg import (
    VehicleOutput,
    CabToModelCorrective,
    CabToSteeringCorrective,
)
import torch, torchvision
import numpy as np

import scipy
import matplotlib.pyplot as plt
import os

import time
import cv2
import yaml
from tqdm import tqdm as tqdm

parser = argparse.ArgumentParser(
    description="Introspection of data from a run on the ANSYS simulator"
)
parser.add_argument("bag_dir", type=str, help="Bag to load")
# parser.add_argument("--trackfiledir", help="Path to the directory containing the raceline json files. Default is environment variable F1_TRACK_DIR",  type=str, default=os.environ.get("F1_TRACK_DIR",default=None))

args = parser.parse_args()
argdict = dict(vars(args))

bag_dir = argdict["bag_dir"]
if bag_dir[-2:] in {"\\\\", "//"}:
    bag_dir = bag_dir[0:-2]
elif bag_dir[-1] in {"\\", "/"}:
    bag_dir = bag_dir[0:-1]
topic_types, type_map, reader = rosbag_utils.open_bagfile(bag_dir)
with open(os.path.join(bag_dir, "metadata.yaml"), "r") as f:
    metadata_dict = yaml.load(f, Loader=yaml.SafeLoader)["rosbag2_bagfile_information"]
topic_count_dict = {
    entry["topic_metadata"]["name"]: entry["message_count"]
    for entry in metadata_dict["topics_with_message_count"]
}
topic_counts = np.array(list(topic_count_dict.values()))
total_msgs = np.sum(topic_counts)
msg_dict = {key: [] for key in topic_count_dict.keys()}
print("Loading data from bag")
for idx in tqdm(iterable=range(total_msgs)):
    if reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = type_map[topic]
        msg_type_full = get_message(msg_type)
        msg = deserialize_message(data, msg_type_full)
        msg_dict[topic].append(msg)
vehicle_output_msgs: List[VehicleOutput] = msg_dict["/vehicle_output"]
cab_to_steering_corrective_msgs: List[CabToSteeringCorrective] = msg_dict[
    "/cab_to_steering_corrective"
]
cab_to_model_corrective_msgs: List[CabToModelCorrective] = msg_dict[
    "/cab_to_model_corrective"
]

vehicle_output_times = np.asarray(
    [
        float(Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        for msg in vehicle_output_msgs
    ]
)
vehicle_output_grips = np.asarray([msg.grip for msg in vehicle_output_msgs])
numslips = np.sum(vehicle_output_grips != 1.0)
print("Grip was not 1 in %d places" % (numslips,))
vehicle_output_steering_angles = np.asarray(
    [msg.steering_wheel_angle for msg in vehicle_output_msgs]
)
vehicle_output_ang_vels = np.asarray(
    [
        [msg.cdg_angular_speed.x, msg.cdg_angular_speed.y, msg.cdg_angular_speed.z]
        for msg in vehicle_output_msgs
    ]
)
vehicle_output_ang_accels = np.asarray(
    [
        [msg.cdg_angular_accel.x, msg.cdg_angular_accel.y, msg.cdg_angular_accel.z]
        for msg in vehicle_output_msgs
    ]
)

corrective_steering_angle_times = np.asarray(
    [
        float(Time.from_msg(msg.header.stamp).nanoseconds) / 1e9
        for msg in cab_to_steering_corrective_msgs
    ]
)
corrective_steering_angles = np.asarray(
    [msg.additive_steering_wheel_angle for msg in cab_to_steering_corrective_msgs]
)

fig1 = plt.figure()
plt.scatter(
    corrective_steering_angle_times, corrective_steering_angles, label="corrective"
)
plt.scatter(
    vehicle_output_times, vehicle_output_steering_angles, label="vehicle_output"
)
plt.legend()
fig2 = plt.figure()
# plt.scatter(vehicle_output_steering_angles, vehicle_output_ang_vels[:,2], label="Angular velocity versus steering angle")
# plt.xlabel("Steering Angle")
plt.plot(
    vehicle_output_times,
    vehicle_output_ang_vels[:, 2],
    label="Angular velocity versus time",
)
plt.plot(
    vehicle_output_times,
    vehicle_output_ang_accels[:, 2],
    label="Angular acceleration versus time",
)
plt.xlabel("Time")

# steering_range = np.linspace(np.min(vehicle_output_steering_angles), np.max(vehicle_output_steering_angles), num=vehicle_output_steering_angles.shape[0])
# plt.plot(steering_range, steering_range, label="Diagonal Line")


plt.ylabel("Rotation Speed")
plt.legend()
plt.show()
