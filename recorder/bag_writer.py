"""
ROS2 bag2 writer for the SLAM data-collection recorder, using the pure-Python
`rosbags` library (no ROS2 install needed on the Pi).

Topic layout matches the RGB-D topics used by the WSL2 RTAB-Map workflow:
  /oak/rgb/image_raw      sensor_msgs/Image      (bgr8)
  /oak/stereo/image_raw   sensor_msgs/Image      (16UC1, millimeters, aligned to RGB)
  /oak/rgb/camera_info    sensor_msgs/CameraInfo
  /odom                   nav_msgs/Odometry
  /tf                     tf2_msgs/TFMessage     (odom -> base_link)

All messages are stamped from the same host (Pi) wall-clock reading taken at
the moment each piece of data was produced/dequeued - see recorder/oak_camera.py
and recorder/base_poller.py. This is the plan's single clock-authority fix for
the TF_OLD_DATA / clock-mismatch issues encountered previously.
"""
import math
import os

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

RGB_TOPIC = "/oak/rgb/image_raw"
DEPTH_TOPIC = "/oak/stereo/image_raw"
CAMERA_INFO_TOPIC = "/oak/rgb/camera_info"
ODOM_TOPIC = "/odom"
TF_TOPIC = "/tf"

RGB_FRAME_ID = "oak_rgb_camera_optical_frame"
ODOM_FRAME_ID = "odom"
BASE_FRAME_ID = "base_link"


def _ros_time(ts_types, host_ts):
    ts_ns = int(host_ts * 1e9)
    sec, nanosec = divmod(ts_ns, 10**9)
    return ts_types["builtin_interfaces/msg/Time"](sec=sec, nanosec=nanosec), ts_ns


class BagWriter:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        self.writer = Writer(bag_path, version=Writer.VERSION_LATEST)
        self._conns = {}

    def open(self):
        self.writer.open()
        ts = self.typestore
        self._conns["rgb"] = self.writer.add_connection(RGB_TOPIC, "sensor_msgs/msg/Image", typestore=ts)
        self._conns["depth"] = self.writer.add_connection(DEPTH_TOPIC, "sensor_msgs/msg/Image", typestore=ts)
        self._conns["camera_info"] = self.writer.add_connection(CAMERA_INFO_TOPIC, "sensor_msgs/msg/CameraInfo", typestore=ts)
        self._conns["odom"] = self.writer.add_connection(ODOM_TOPIC, "nav_msgs/msg/Odometry", typestore=ts)
        self._conns["tf"] = self.writer.add_connection(TF_TOPIC, "tf2_msgs/msg/TFMessage", typestore=ts)

    def _header(self, host_ts, frame_id):
        T = self.typestore.types
        stamp, _ = _ros_time(T, host_ts)
        return T["std_msgs/msg/Header"](stamp=stamp, frame_id=frame_id)

    def write_rgb(self, host_ts, frame_bgr):
        T = self.typestore.types
        height, width = frame_bgr.shape[:2]
        msg = T["sensor_msgs/msg/Image"](
            header=self._header(host_ts, RGB_FRAME_ID),
            height=height,
            width=width,
            encoding="bgr8",
            is_bigendian=0,
            step=width * 3,
            data=np.ascontiguousarray(frame_bgr).reshape(-1),
        )
        ts_ns = int(host_ts * 1e9)
        self.writer.write(self._conns["rgb"], ts_ns, self.typestore.serialize_cdr(msg, msg.__msgtype__))

    def write_depth(self, host_ts, frame_u16):
        T = self.typestore.types
        height, width = frame_u16.shape[:2]
        msg = T["sensor_msgs/msg/Image"](
            header=self._header(host_ts, RGB_FRAME_ID),
            height=height,
            width=width,
            encoding="16UC1",
            is_bigendian=0,
            step=width * 2,
            data=np.ascontiguousarray(frame_u16).view(np.uint8).reshape(-1),
        )
        ts_ns = int(host_ts * 1e9)
        self.writer.write(self._conns["depth"], ts_ns, self.typestore.serialize_cdr(msg, msg.__msgtype__))

    def write_camera_info(self, host_ts, info):
        T = self.typestore.types
        roi = T["sensor_msgs/msg/RegionOfInterest"](x_offset=0, y_offset=0, height=0, width=0, do_rectify=False)
        fx, fy = info["k"][0], info["k"][4]
        cx, cy = info["k"][2], info["k"][5]
        msg = T["sensor_msgs/msg/CameraInfo"](
            header=self._header(host_ts, RGB_FRAME_ID),
            height=info["height"],
            width=info["width"],
            distortion_model=info["distortion_model"],
            d=np.array(info["d"], dtype=np.float64),
            k=np.array(info["k"], dtype=np.float64),
            r=np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64),
            p=np.array([fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float64),
            binning_x=0,
            binning_y=0,
            roi=roi,
        )
        ts_ns = int(host_ts * 1e9)
        self.writer.write(self._conns["camera_info"], ts_ns, self.typestore.serialize_cdr(msg, msg.__msgtype__))

    def write_odometry(self, host_ts, x, y, theta, vx, vth):
        T = self.typestore.types
        qz, qw = math.sin(theta / 2.0), math.cos(theta / 2.0)
        pose = T["geometry_msgs/msg/Pose"](
            position=T["geometry_msgs/msg/Point"](x=x, y=y, z=0.0),
            orientation=T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=qz, w=qw),
        )
        twist = T["geometry_msgs/msg/Twist"](
            linear=T["geometry_msgs/msg/Vector3"](x=vx, y=0.0, z=0.0),
            angular=T["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=vth),
        )
        zero_cov = np.zeros(36, dtype=np.float64)
        msg = T["nav_msgs/msg/Odometry"](
            header=self._header(host_ts, ODOM_FRAME_ID),
            child_frame_id=BASE_FRAME_ID,
            pose=T["geometry_msgs/msg/PoseWithCovariance"](pose=pose, covariance=zero_cov),
            twist=T["geometry_msgs/msg/TwistWithCovariance"](twist=twist, covariance=zero_cov),
        )
        ts_ns = int(host_ts * 1e9)
        self.writer.write(self._conns["odom"], ts_ns, self.typestore.serialize_cdr(msg, msg.__msgtype__))

    def write_tf(self, host_ts, x, y, theta):
        T = self.typestore.types
        qz, qw = math.sin(theta / 2.0), math.cos(theta / 2.0)
        transform = T["geometry_msgs/msg/TransformStamped"](
            header=self._header(host_ts, ODOM_FRAME_ID),
            child_frame_id=BASE_FRAME_ID,
            transform=T["geometry_msgs/msg/Transform"](
                translation=T["geometry_msgs/msg/Vector3"](x=x, y=y, z=0.0),
                rotation=T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=qz, w=qw),
            ),
        )
        msg = T["tf2_msgs/msg/TFMessage"](transforms=[transform])
        ts_ns = int(host_ts * 1e9)
        self.writer.write(self._conns["tf"], ts_ns, self.typestore.serialize_cdr(msg, msg.__msgtype__))

    def close(self):
        self.writer.close()
        self._fix_metadata_for_native_ros2bag()

    def _fix_metadata_for_native_ros2bag(self):
        # rosbags writes offered_qos_profiles as an empty YAML list ([]) and
        # custom_data as null. The native rosbag2 C++ reader (yaml-cpp)
        # expects offered_qos_profiles to be a string and custom_data to be
        # a map, so `ros2 bag play` / `ros2 bag info` fail with "bad
        # conversion" unless we patch the file after rosbags writes it.
        meta_path = os.path.join(self.bag_path, "metadata.yaml")
        try:
            with open(meta_path, "r") as fh:
                text = fh.read()
        except FileNotFoundError:
            return
        text = text.replace("offered_qos_profiles: []", "offered_qos_profiles: ''")
        text = text.replace("custom_data: null", "custom_data: {}")
        with open(meta_path, "w") as fh:
            fh.write(text)
