import argparse
import logging
from pathlib import Path
import signal
import threading

from werkzeug.serving import make_server

from .core import ROOT, PreviewEngine, load_settings
from .runtime import LiveSource, PreviewRuntime, demo_source, replay_source
from .server import create_app


def main() -> None:
    parser = argparse.ArgumentParser(description="Pi-local OAK-D preview dashboard. Never sends motor commands.")
    parser.add_argument("--config", type=Path, help="override preview YAML; replay otherwise restores recorded settings")
    parser.add_argument("--host", default="127.0.0.1", help="use 0.0.0.0 for a trusted LAN")
    parser.add_argument("--port", type=int, default=5001)
    sources = parser.add_mutually_exclusive_group()
    sources.add_argument("--demo", choices=("clear", "wall", "empty"), help="synthetic input, no camera")
    sources.add_argument("--replay", type=Path, help="loop a saved .npz frame, no camera")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    if not 1 <= args.port <= 65535:
        parser.error("port must be in [1, 65535]")
    settings, planner = load_settings(args.config or ROOT / "config" / "rover_preview.yaml")
    goal = None
    if args.replay:
        source, recorded_settings, recorded_planner, goal = replay_source(args.replay)
        if args.config is None and recorded_settings is not None:
            settings, planner = recorded_settings, recorded_planner
    elif args.demo:
        source = demo_source(args.demo)
    else:
        source = LiveSource(settings)
    engine = PreviewEngine(settings, planner)
    if goal is not None:
        engine.set_goal(goal)
    runtime = PreviewRuntime(engine, source)
    server = make_server(args.host, args.port, create_app(runtime), threaded=True)
    stopped = threading.Event()

    def shutdown(signum, frame):
        if not stopped.is_set():
            stopped.set()
            threading.Thread(target=server.shutdown, name="preview-http-shutdown", daemon=True).start()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    logging.info("NO-MOTION preview: %s; dashboard http://%s:%s", source.name, args.host, args.port)
    runtime.start()
    try:
        server.serve_forever()
    finally:
        runtime.stop()
        server.server_close()


if __name__ == "__main__":
    main()
