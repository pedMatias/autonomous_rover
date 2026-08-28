"""
Background poller for the ESP32 feedback stream (T:1001: odl/odr/IMU/voltage).

Stamps every sample with time.time() at the moment it's dequeued from the
serial reader (the plan's single clock-authority decision - never mix in the
ESP32's own notion of time, since it sends none, and never mix in the OAK-D's
device clock either). Feeds odl/odr into an OdometryIntegrator and reports
(host_ts, raw_feedback_dict, pose) to a callback for every sample that
contains encoder data.
"""
import threading
import time


class BasePoller(threading.Thread):
    def __init__(self, base, odometry, on_sample, poll_interval=0.02):
        super().__init__(daemon=True)
        self.base = base
        self.odometry = odometry
        self.on_sample = on_sample
        self.poll_interval = poll_interval
        self._stop_event = threading.Event()

    def enable_feedback_stream(self):
        # Matches app.py's cmd_on_boot() sequence for the base's periodic feedback.
        self.base.base_json_ctrl({"T": 142, "cmd": 50})
        self.base.base_json_ctrl({"T": 131, "cmd": 1})
        self.base.base_json_ctrl({"T": 143, "cmd": 0})
        time.sleep(0.3)

    def run(self):
        while not self._stop_event.is_set():
            data = self.base.feedback_data()
            host_ts = time.time()
            if data and "odl" in data and "odr" in data:
                pose = self.odometry.update(data["odl"], data["odr"], host_ts)
                self.on_sample(host_ts, data, pose)
            time.sleep(self.poll_interval)

    def stop(self):
        self._stop_event.set()
