import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField

from rover_bridge.protocol import depth_png_bytes


def depth_to_points(depth, camera_info, decimation=4, min_depth_m=0.3, max_depth_m=5.0):
    """Project a uint16 millimetre depth image into optical-frame XYZ points."""
    if depth.ndim != 2 or depth.dtype != np.uint16:
        raise ValueError("depth image must be a uint16 single-channel image")
    if len(camera_info.k) != 9:
        raise ValueError("camera info K must contain nine values")
    if decimation < 1:
        raise ValueError("decimation must be positive")
    if not 0.0 < min_depth_m < max_depth_m:
        raise ValueError("depth range must be positive and ordered")

    depth_height, depth_width = depth.shape
    info_width = float(camera_info.width or depth_width)
    info_height = float(camera_info.height or depth_height)
    fx = float(camera_info.k[0]) * depth_width / info_width
    fy = float(camera_info.k[4]) * depth_height / info_height
    cx = float(camera_info.k[2]) * depth_width / info_width
    cy = float(camera_info.k[5]) * depth_height / info_height
    if not all(np.isfinite(value) and value > 0.0 for value in (fx, fy)):
        raise ValueError("camera info has invalid focal lengths")

    rows = np.arange(0, depth_height, decimation, dtype=np.float32)
    columns = np.arange(0, depth_width, decimation, dtype=np.float32)
    column_grid, row_grid = np.meshgrid(columns, rows)
    z = depth[::decimation, ::decimation].astype(np.float32) / 1000.0
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32)

    z = z[valid]
    x = (column_grid[valid] - cx) * z / fx
    y = (row_grid[valid] - cy) * z / fy
    return np.column_stack((x, y, z)).astype(np.float32, copy=False)


class PointCloudPreview(Node):
    def __init__(self):
        super().__init__("gcs_pointcloud_preview")
        self.declare_parameter(
            "depth_topic", "/camera/depth/image_raw/compressedDepth"
        )
        self.declare_parameter("camera_info_topic", "/oak/rgb/camera_info")
        self.declare_parameter("pointcloud_topic", "/gcs/pointcloud")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("decimation", 4)
        self.declare_parameter("min_depth_m", 0.3)
        self.declare_parameter("max_depth_m", 5.0)

        depth_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self._latest_depth = None
        self._latest_camera_info = None
        self._depth_subscription = self.create_subscription(
            CompressedImage, depth_topic, self._depth_callback, depth_qos
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self._publisher = self.create_publisher(
            PointCloud2, pointcloud_topic, depth_qos
        )
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if not np.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_cloud)

    def _depth_callback(self, message):
        self._latest_depth = message

    def _camera_info_callback(self, message):
        self._latest_camera_info = message

    def _publish_cloud(self):
        depth_message = self._latest_depth
        camera_info = self._latest_camera_info
        if depth_message is None or camera_info is None:
            return
        self._latest_depth = None
        try:
            png = depth_png_bytes(bytes(depth_message.data))
            depth = cv2.imdecode(
                np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_UNCHANGED
            )
            points = depth_to_points(
                depth,
                camera_info,
                int(self.get_parameter("decimation").value),
                float(self.get_parameter("min_depth_m").value),
                float(self.get_parameter("max_depth_m").value),
            )
        except (ValueError, TypeError, cv2.error) as exc:
            self.get_logger().warning(f"Skipping point cloud: {exc}")
            return

        cloud = PointCloud2()
        cloud.header = depth_message.header
        cloud.height = 1
        cloud.width = points.shape[0]
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = points.astype("<f4", copy=False).tobytes()
        self._publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPreview()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()