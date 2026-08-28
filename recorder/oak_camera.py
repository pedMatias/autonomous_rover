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

import depthai as dai


class OakCamera:
    def __init__(self, rgb_size=(640, 480), mono_size=(640, 400), fps=10.0, sync_threshold_ms=50):
        self.rgb_size = rgb_size
        self.mono_size = mono_size
        self.fps = fps
        self.sync_threshold_ms = sync_threshold_ms

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
        rgb_out.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth"])

        self.sync_queue = sync.out.createOutputQueue()

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
        t0 = time.time()
        while time.time() - t0 < seconds:
            if self.sync_queue.has():
                self.sync_queue.get()

    def get_camera_info(self):
        return self._camera_info

    def read(self, timeout=1.0):
        """Return (host_ts, rgb_bgr, depth_u16) for the next synchronized frame pair, or None on timeout."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.sync_queue.has():
                group = self.sync_queue.get()
                host_ts = time.time()
                rgb_frame = group["rgb"].getCvFrame()
                depth_frame = group["depth"].getFrame()
                return host_ts, rgb_frame, depth_frame
        return None

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
