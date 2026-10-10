"""
OAK-D Lite camera wrapper for the SLAM data-collection recorder.

Builds a DepthAI pipeline that produces host-clock-timestamped, synchronized
(RGB, aligned-depth) frame pairs using an on-device dai.node.Sync node.

Design notes:
  - Depth is aligned to the RGB camera on-device (StereoDepth.setDepthAlign)
    so RGB and depth share the same resolution/frame_id/intrinsics - this is
    what RTAB-Map expects (rgb_topic/depth_topic/camera_info_topic, not a
    pre-fused point cloud).
  - stereo.setOutputSize(...) MUST match the RGB output size, otherwise the
    align node crashes the device (RGB full-res width is not a multiple of 16).
  - The first several seconds of depth frames are sparse/noisy (auto-exposure
    and stereo matching haven't converged) - call warmup() before recording.
  - Timestamps: this module ALWAYS stamps with time.time() at host dequeue
    time, per the plan's clock-authority decision. depthai's own internal
    timestamps are read but not used as the authoritative timestamp.
"""
import time
from datetime import timedelta

import depthai as dai


class OakCamera:
    def __init__(self, rgb_size=(640, 480), mono_size=(640, 400), fps=10.0, sync_threshold_ms=50, latest_only=False, profile=False):
        self.rgb_size = rgb_size
        self.mono_size = mono_size
        self.fps = fps
        self.sync_threshold_ms = sync_threshold_ms
        self.latest_only = latest_only
        self.profile = profile
        self.last_metadata = {}

        self.pipeline = None
        self.sync_queue = None
        self._camera_info = None

    def start(self):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.create(dai.node.Camera).build()
        mono_left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        mono_right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(*self.rgb_size)

        mono_left_out = mono_left.requestOutput(self.mono_size, fps=self.fps)
        mono_right_out = mono_right.requestOutput(self.mono_size, fps=self.fps)
        mono_left_out.link(stereo.left)
        mono_right_out.link(stereo.right)

        rgb_out = cam_rgb.requestOutput(self.rgb_size, fps=self.fps)

        sync = pipeline.create(dai.node.Sync)
        if self.profile:
            sync.setSyncThreshold(timedelta(milliseconds=self.sync_threshold_ms))
        rgb_out.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth"])

        self.sync_queue = (
            sync.out.createOutputQueue(maxSize=1, blocking=False)
            if self.latest_only else sync.out.createOutputQueue()
        )

        # Grab calibration before start() (calibration data is available on the pipeline itself).
        calib = pipeline.getCalibrationData()
        width, height = self.rgb_size
        k = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height)
        d = calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        self._camera_info = {
            "width": width,
            "height": height,
            "k": [float(v) for row in k for v in row],
            "d": [float(v) for v in d],
            "distortion_model": "rational_polynomial",
        }

        self.pipeline = pipeline
        pipeline.start()

    def warmup(self, seconds=8.0):
        """Drain frames without returning them so depth density stabilizes before recording starts."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            if self.sync_queue.has():
                self.sync_queue.get()
            else:
                time.sleep(0.005)

    def get_camera_info(self):
        return self._camera_info

    def read(self, timeout=1.0):
        """Return (host_ts, rgb_bgr, depth_u16) for the next synchronized frame pair, or None on timeout."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.sync_queue.has():
                dequeued_at = time.monotonic() if self.profile else None
                group = self.sync_queue.get()
                host_ts = time.time()
                converted_at = time.monotonic() if self.profile else None
                rgb_frame = group["rgb"].getCvFrame()
                rgb_ready_at = time.monotonic() if self.profile else None
                depth_frame = group["depth"].getFrame()
                if self.profile:
                    depth_ready_at = time.monotonic()
                    rgb_device = group["rgb"].getTimestampDevice().total_seconds()
                    depth_device = group["depth"].getTimestampDevice().total_seconds()
                    self.last_metadata = {
                        "rgb_seq": group["rgb"].getSequenceNum(),
                        "depth_seq": group["depth"].getSequenceNum(),
                        "rgb_device_s": rgb_device,
                        "depth_device_s": depth_device,
                        "device_clock": "DepthAI device; not host monotonic",
                        "rgb_depth_skew_ms": (rgb_device - depth_device) * 1000,
                        "dequeued_at": dequeued_at,
                        "dequeue_ms": (converted_at - dequeued_at) * 1000,
                        "rgb_conversion_ms": (rgb_ready_at - converted_at) * 1000,
                        "depth_conversion_ms": (depth_ready_at - rgb_ready_at) * 1000,
                    }
                return host_ts, rgb_frame, depth_frame
            time.sleep(0.005)
        return None

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
