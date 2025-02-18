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
import scipy.interpolate
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
parser.add_argument(
    "--inner_boundary", type=str, required=True, help="Path to the inner boundary"
)
parser.add_argument("raceline_file", type=str, help="Raceline to Display")

# parser.add_argument("--trackfiledir", help="Path to the directory containing the raceline json files. Default is environment variable F1_TRACK_DIR",  type=str, default=os.environ.get("F1_TRACK_DIR",default=None))

args = parser.parse_args()
argdict = dict(vars(args))
raceline_file = argdict["raceline_file"]
inner_boundary_file = argdict["inner_boundary"]
print("Hello World!")
track_centroid: torch.Tensor = torch.as_tensor(
    [57.5, -1637.75, 0.73], dtype=torch.float64
)
figpositions, axpositions = plt.subplots()
axpositions.set_title("Positions")
outer_boundary_file = os.path.join(
    os.path.dirname(inner_boundary_file), "outer_boundary.csv"
)
inner_boundary = torch.as_tensor(
    np.loadtxt(inner_boundary_file, skiprows=1, delimiter=","),
    dtype=track_centroid.dtype,
)
outer_boundary = torch.as_tensor(
    np.loadtxt(outer_boundary_file, skiprows=1, delimiter=","),
    dtype=track_centroid.dtype,
)
axpositions.plot(
    inner_boundary[:, 0], inner_boundary[:, 1], label="Inner Boundary", color="black"
)
axpositions.plot(
    outer_boundary[:, 0], outer_boundary[:, 1], label="Outer Boundary", color="black"
)
track_centroid: torch.Tensor = torch.mean(inner_boundary, dim=0)
racelinein = torch.as_tensor(
    np.loadtxt(raceline_file, skiprows=1, delimiter=","), dtype=track_centroid.dtype
)


istart = 1100
iend = racelinein.shape[0] - 500

raceline = racelinein[istart:iend].clone()

axpositions.plot(raceline[:, 0], raceline[:, 1], label=os.path.basename(raceline_file))
axpositions.legend()
# pextra = torch.as_tensor([-303.26]
print("showing raceline")
plt.show()


endsegment = raceline[-4:-1].clone()
endpoly = np.polyfit(endsegment[:, 0], endsegment[:, 1], 1)

extrax = torch.linspace(
    endsegment[-1, 0] + 0.1, -303.0, steps=50, dtype=track_centroid.dtype
)
extray = torch.as_tensor(np.polyval(endpoly, extrax), dtype=track_centroid.dtype)

extrapoints = torch.stack(
    [extrax, extray, track_centroid[2] * torch.ones_like(extray)], dim=1
)

racelinestep1 = torch.cat([raceline, extrapoints], dim=0)


figpositions, axpositions = plt.subplots()
axpositions.set_title("Positions")
axpositions.plot(
    inner_boundary[:, 0], inner_boundary[:, 1], label="Inner Boundary", color="black"
)
axpositions.plot(
    outer_boundary[:, 0], outer_boundary[:, 1], label="Outer Boundary", color="black"
)
axpositions.plot(
    racelinestep1[:, 0], racelinestep1[:, 1], label=os.path.basename(raceline_file)
)

plt.show()


interp_point = torch.as_tensor(
    [-301.69, -2108.25, track_centroid[2].item()], dtype=track_centroid.dtype
)
# interp_point2 = torch.as_tensor([-303.861, -2142.6, track_centroid[2].item()], dtype=track_centroid.dtype)

splpoints = torch.cat(
    [racelinestep1[-5:], interp_point.unsqueeze(0), racelinestep1[:10]], dim=0
)
# splpoints = torch.cat([racelinestep1[-5:], racelinestep1[:10]],dim=0)
# print(splpoints)

distances = torch.zeros_like(splpoints[:, 0])
distances[1:] = torch.cumsum(torch.norm(splpoints[1:] - splpoints[:-1], p=2, dim=1), 0)
# print(distances.shape)
# print(splpoints.shape)

spl: scipy.interpolate.BSpline = scipy.interpolate.make_interp_spline(
    distances.cpu(), splpoints.cpu(), bc_type="natural", k=3
)
dsamp = torch.arange(
    distances[4].item() + 0.5,
    distances[6].item(),
    step=torch.mean(torch.norm(racelinestep1[10:20] - racelinestep1[9:19], p=2, dim=1)),
)
psamp = torch.as_tensor(spl(dsamp), dtype=splpoints.dtype)


figpositions, axpositions = plt.subplots()
axpositions.set_title("Positions")
axpositions.plot(
    inner_boundary[:, 0], inner_boundary[:, 1], label="Inner Boundary", color="black"
)
axpositions.plot(
    outer_boundary[:, 0], outer_boundary[:, 1], label="Outer Boundary", color="black"
)
axpositions.plot(
    racelinestep1[:, 0],
    racelinestep1[:, 1],
    label=os.path.basename(raceline_file),
    color="blue",
)
axpositions.plot(psamp[:, 0], psamp[:, 1], label="Interpolated Points", color="red")
axpositions.legend()
plt.show()


racelinestep2 = torch.cat([racelinestep1, psamp], dim=0)

np.savetxt(
    os.path.join(os.path.dirname(raceline_file), "optimal.csv"),
    racelinestep2.cpu().numpy(),
    header="X,Y,Z",
    delimiter=",",
    fmt="%.6e",
)
