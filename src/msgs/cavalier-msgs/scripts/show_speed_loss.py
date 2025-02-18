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
import deepracing_ros.convert as C

from geometry_msgs.msg import PointStamped, Point, Vector3Stamped, Vector3
from vrxperience_msgs.msg import (
    VehicleOutput,
    CabToModelCorrective,
    CabToSteeringCorrective,
)
import torch, torchvision
import numpy as np

import scipy
from scipy.spatial.transform import Rotation as Rot, RotationSpline as RotSpline
import matplotlib.pyplot as plt
import os


import time
import cv2
import yaml
from tqdm import tqdm as tqdm
from typing import List


def vehicleOutputKey(vo: VehicleOutput):
    return Time.from_msg(vo.header.stamp)


parser = argparse.ArgumentParser(
    description="Introspection of data from a run on the ANSYS simulator"
)
parser.add_argument("bag_dirs", type=str, nargs="+", help="Bags to load")
parser.add_argument(
    "--inner_boundary", type=str, required=False, help="Path to the inner boundary"
)

# parser.add_argument("--trackfiledir", help="Path to the directory containing the raceline json files. Default is environment variable F1_TRACK_DIR",  type=str, default=os.environ.get("F1_TRACK_DIR",default=None))

args = parser.parse_args()
argdict = dict(vars(args))
bag_dirs = argdict["bag_dirs"]
inner_boundary_file = argdict.get("inner_boundary", None)
track_centroid: torch.Tensor = torch.as_tensor(
    [57.5, -1637.75, 0.73], dtype=torch.float64
)
figspeeddistance, axspeeddistnace = plt.subplots()
axspeeddistnace.set_title("Speed Versus Distance")
figspeed, axspeed = plt.subplots()
axspeed.set_title("Speed Versus Time")
figpositions, axpositions = plt.subplots()
axpositions.set_title("Positions")
if inner_boundary_file is not None:
    inner_boundary = np.loadtxt(inner_boundary_file, skiprows=1, delimiter=",")
    axpositions.plot(
        inner_boundary[:, 0],
        inner_boundary[:, 1],
        label="Inner Boundary",
        color="black",
    )

for bag_dir in bag_dirs:
    if bag_dir[-2:] in {"\\\\", "//"}:
        bag_dir = bag_dir[0:-2]
    elif bag_dir[-1] in {"\\", "/"}:
        bag_dir = bag_dir[0:-1]
    topic_types, type_map, reader = rosbag_utils.open_bagfile(bag_dir)
    with open(os.path.join(bag_dir, "metadata.yaml"), "r") as f:
        metadata_dict = yaml.load(f, Loader=yaml.SafeLoader)[
            "rosbag2_bagfile_information"
        ]
    topic_count_dict = {
        entry["topic_metadata"]["name"]: entry["message_count"]
        for entry in metadata_dict["topics_with_message_count"]
    }
    topic_counts = np.array(list(topic_count_dict.values()))
    total_msgs = np.sum(topic_counts)
    vehicle_outputs: List[VehicleOutput] = []
    print("Loading data from bag: %s" % (bag_dir,))
    for idx in tqdm(iterable=range(total_msgs)):
        if reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == "/vehicle_output":
                msg_type = type_map[topic]
                msg_type_full = get_message(msg_type)
                msg = deserialize_message(data, msg_type_full)
                vehicle_outputs.append(msg)
    vehicle_outputs = sorted(vehicle_outputs, key=vehicleOutputKey)
    L = 3.050
    idx = 0
    for i in range(len(vehicle_outputs)):
        if vehicle_outputs[i].cdg_position.y < -1847.0:
            idx = i
            break
    vehicle_outputs = vehicle_outputs[idx:]
    for i in range(len(vehicle_outputs) - 1, -1, -1):
        if vehicle_outputs[i].cdg_position.y > -1847.0:
            idxend = i
            break
    vehicle_outputs = vehicle_outputs[:idxend]

    velocity_vectors: torch.Tensor = torch.stack(
        [
            C.vectorMsgToTorch(
                vehicle_outputs[i].cdg_speed,
                dtype=track_centroid.dtype,
                device=track_centroid.device,
            )
            for i in range(len(vehicle_outputs))
        ],
        dim=0,
    )
    angvel_vectors: torch.Tensor = torch.stack(
        [
            C.vectorMsgToTorch(
                vehicle_outputs[i].cdg_angular_speed,
                dtype=velocity_vectors.dtype,
                device=velocity_vectors.device,
            )
            for i in range(len(vehicle_outputs))
        ],
        dim=0,
    )
    position_vectors: torch.Tensor = torch.stack(
        [
            C.vectorMsgToTorch(
                vehicle_outputs[i].cdg_position,
                dtype=velocity_vectors.dtype,
                device=velocity_vectors.device,
            )
            for i in range(len(vehicle_outputs))
        ],
        dim=0,
    )
    steering_wheel_angles: torch.Tensor = torch.as_tensor(
        [vehicle_outputs[i].steering_wheel_angle for i in range(len(vehicle_outputs))],
        dtype=velocity_vectors.dtype,
        device=velocity_vectors.device,
    )
    rpy_vectors = torch.stack(
        [
            C.vectorMsgToTorch(
                vehicle_outputs[i].cdg_orientation,
                dtype=velocity_vectors.dtype,
                device=velocity_vectors.device,
            )
            for i in range(len(vehicle_outputs))
        ],
        dim=0,
    )
    rotations = Rot.from_euler("xyz", rpy_vectors.cpu().numpy())
    rot_mats = torch.as_tensor(
        rotations.as_matrix(),
        dtype=velocity_vectors.dtype,
        device=velocity_vectors.device,
    )
    poses = torch.stack(
        [
            torch.eye(4, dtype=velocity_vectors.dtype, device=velocity_vectors.device)
            for i in range(rot_mats.shape[0])
        ],
        dim=0,
    )

    times: torch.Tensor = torch.as_tensor(
        [
            float(Time.from_msg(vehicle_outputs[i].header.stamp).nanoseconds) / 1e9
            for i in range(len(vehicle_outputs))
        ],
        dtype=velocity_vectors.dtype,
        device=velocity_vectors.device,
    )
    times = times - times[0]

    poses[:, 0:3, 0:3] = rot_mats
    poses[:, 0:3, 3] = position_vectors
    cgToBl: torch.Tensor = torch.eye(
        4, dtype=velocity_vectors.dtype, device=velocity_vectors.device
    )
    cgToBl[0:3, 3] = torch.as_tensor(
        [-1.321, 0.030, -0.434],
        dtype=velocity_vectors.dtype,
        device=velocity_vectors.device,
    )
    poses = torch.matmul(poses, cgToBl)

    baselink_positions = poses[:, 0:3, 3]
    baselink_positions_centered = baselink_positions - track_centroid
    distances = torch.zeros_like(times)
    distances[1:] = torch.cumsum(
        torch.norm(
            baselink_positions_centered[1:] - baselink_positions_centered[:-1],
            p=2,
            dim=1,
        ),
        0,
    )

    linearspeeds = torch.norm(velocity_vectors, p=2, dim=1)
    axspeeddistnace.plot(
        distances.cpu().numpy(), 3.6 * linearspeeds.cpu().numpy(), label=bag_dir
    )
    axspeed.plot(times.cpu().numpy(), 3.6 * linearspeeds.cpu().numpy(), label=bag_dir)
    axpositions.plot(
        poses[:, 0, 3].cpu().numpy(), poses[:, 1, 3].cpu().numpy(), label=bag_dir
    )

    # track_angles = torch.atan2(baselink_positions_centered[:,1], baselink_positions_centered[:,0])
    # I = ~((track_angles<-145.0*np.pi/180.0)*(linearspeeds>70.0))
    # track_angles = track_angles[I]
    # linearspeeds = linearspeeds[I]
    # pltline = plt.plot(track_angles.cpu().numpy(), 3.6*linearspeeds.cpu().numpy(), label=bag_dir)


axspeeddistnace.set_xlabel("distance (meters)")
axspeeddistnace.set_ylabel("Speed (kph)")
axspeeddistnace.legend()
axspeeddistnace.grid()

axspeed.set_xlabel("Time (seconds)")
axspeed.set_ylabel("Speed (kph)")
axspeed.legend()
axspeed.grid()

axpositions.set_xlabel("X position")
axpositions.set_ylabel("Y position")
axpositions.legend(bbox_to_anchor=(0, 1), loc="right", ncol=1)
axpositions.grid()
axpositions.set_aspect(1.6)


plt.show()


# line = np.polyfit(back_computed_steering.cpu().numpy(), steering_wheel_angles.cpu().numpy(), 1)
# fitvals = np.polyval(line, back_computed_steering.cpu().numpy())
# plt.scatter(back_computed_steering.cpu().numpy(), steering_wheel_angles.cpu().numpy())
# plt.plot(back_computed_steering.cpu().numpy(), fitvals, c="black")
# plt.show()
