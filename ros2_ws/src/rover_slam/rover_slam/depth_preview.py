import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from rover_bridge.protocol import depth_png_bytes


class DepthPreview(Node):
    def __init__(self):
        super().__init__("gcs_depth_preview")
        self.declare_parameter(
            "depth_topic", "/camera/depth/image_raw/compressedDepth"
        )
        self.declare_parameter("preview_topic", "/gcs/depth_preview/image_raw")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("min_depth_m", 0.3)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("jpeg_quality", 70)

        source_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        preview_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        depth_topic = str(self.get_parameter("depth_topic").value)
        preview_topic = str(self.get_parameter("preview_topic").value)
        self._latest = None
        self._subscription = self.create_subscription(
            CompressedImage, depth_topic, self._depth_callback, source_qos
        )
        self._publisher = self.create_publisher(
            CompressedImage, preview_topic + "/compressed", preview_qos
        )
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_preview)

    def _depth_callback(self, message):
        self._latest = message

    def _publish_preview(self):
        message = self._latest
        if message is None:
            return
        self._latest = None
        try:
            png = depth_png_bytes(bytes(message.data))
            depth = cv2.imdecode(
                np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_UNCHANGED
            )
            if depth is None or depth.dtype != np.uint16:
                raise ValueError("decoded depth image is not uint16")
            min_depth = float(self.get_parameter("min_depth_m").value) * 1000.0
            max_depth = float(self.get_parameter("max_depth_m").value) * 1000.0
            if not 0.0 < min_depth < max_depth:
                raise ValueError("depth range must be positive and ordered")
            preview = cv2.convertScaleAbs(
                depth,
                alpha=255.0 / (max_depth - min_depth),
                beta=-min_depth * 255.0 / (max_depth - min_depth),
            )
            preview[depth == 0] = 0
            preview = cv2.applyColorMap(preview, cv2.COLORMAP_TURBO)
            preview[depth == 0] = 0
            quality = int(self.get_parameter("jpeg_quality").value)
            ok, encoded = cv2.imencode(
                ".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, quality]
            )
            if not ok:
                raise ValueError("could not encode depth preview")
        except (ValueError, cv2.error) as exc:
            self.get_logger().warning(f"Skipping depth preview: {exc}")
            return

        preview_message = CompressedImage()
        preview_message.header = message.header
        preview_message.format = "jpeg"
        preview_message.data = encoded.tobytes()
        self._publisher.publish(preview_message)


def main(args=None):
    rclpy.init(args=args)
    node = DepthPreview()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()