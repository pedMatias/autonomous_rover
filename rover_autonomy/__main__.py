import argparse
from dataclasses import asdict
import json
import logging
from pathlib import Path
import signal
import subprocess
import threading

from werkzeug.serving import make_server

from rover_preview.core import ROOT, PreviewEngine
from rover_preview.runtime import demo_source
from .controller import Controller, load_config
from .motor import MockMotor, SerialMotor
from .records import RunLog
from .runtime import ProfiledSource, Runtime
from .server import create_app


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Pi-local autonomy. Default uses the live OAK-D and real motor UART."
    )
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "rover_autonomy.yaml")
    parser.add_argument("--port", type=int, default=5002)
    parser.add_argument("--logs", type=Path, default=ROOT / "recordings" / "autonomy")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--demo", choices=("clear", "wall", "empty"),
                       help="synthetic camera scene; no physical hardware")
    modes.add_argument("--replay-run", type=Path, help="offline run replay; no camera or UART")
    parser.add_argument("--mock-motors", action="store_true",
                        help="use simulated wheel feedback; camera remains live unless --demo is set")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if args.demo:
        args.mock_motors = True
    return args


def main():
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    if args.replay_run:
        from .replay import replay_run
        print(json.dumps(replay_run(args.replay_run), indent=2))
        return
    settings, planner, calibration, agent, limits = load_config(args.config)
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True,
                              text=True, check=False).stdout.strip() or "unknown"
    records = RunLog(args.logs, {"preview": asdict(settings), "planner": planner,
                                "calibration": calibration, "agent": agent, "limits": asdict(limits),
                                "revision": revision, "motion_enabled": not args.mock_motors},
                     limits.log_queue_size, limits.disk_reserve_bytes)
    event = lambda kind, payload: records.submit(kind, payload)
    motor = SerialMotor(agent, event) if not args.mock_motors else MockMotor(
        event, min_speed=limits.min_wheel_speed_mps, max_speed=agent["max_wheel_speed_mps"])
    controller = Controller(PreviewEngine(settings, planner), calibration, motor, records, limits)
    source = demo_source(args.demo) if args.demo else ProfiledSource(settings)
    runtime = Runtime(controller, source, calibration)
    server = None
    stopped = threading.Event()

    def shutdown(signum, frame):
        if not stopped.is_set():
            stopped.set()
            controller.stop("shutdown signal")
            if server is not None:
                threading.Thread(target=server.shutdown, name="autonomy-http-shutdown", daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server = make_server("127.0.0.1", args.port, create_app(runtime), threaded=True)
        runtime.start()
        logging.warning("CAMERA=%s MOTORS=%s; GCS http://127.0.0.1:%s; recording %s",
                        "SYNTHETIC" if args.demo else "LIVE OAK-D",
                        "MOCK" if args.mock_motors else "REAL UART",
                        args.port, records.directory)
        server.serve_forever()
    finally:
        runtime.close()
        if server is not None:
            server.server_close()


if __name__ == "__main__":
    main()
