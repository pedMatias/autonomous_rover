"""Loopback GCS API with only high-level requests."""

import io
from pathlib import Path
import time
from urllib.parse import urlsplit

from flask import Flask, g, jsonify, request, send_file
from werkzeug.exceptions import BadRequest, RequestEntityTooLarge


def create_app(runtime):
    app = Flask(__name__, static_folder=str(Path(__file__).parent / "static"))
    app.config["MAX_CONTENT_LENGTH"] = 2048

    @app.before_request
    def validate_write():
        g.started = time.monotonic()
        if request.content_length is not None and request.content_length > app.config["MAX_CONTENT_LENGTH"]:
            app.logger.warning("Rejected oversized GCS request")
            return jsonify(error="request body must not exceed 2048 bytes"), 413
        if request.method == "POST":
            origin = request.headers.get("Origin")
            if request.headers.get("X-Rover-Autonomy") != "1" or (
                origin is not None and urlsplit(origin).netloc != request.host
            ):
                app.logger.warning("Rejected cross-origin GCS request")
                return jsonify(error="same-origin request requires X-Rover-Autonomy: 1"), 403
        return None

    @app.after_request
    def headers(response):
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; img-src 'self' blob:; style-src 'self'; "
            "script-src 'self'; connect-src 'self'; frame-ancestors 'none'"
        )
        runtime.controller.records.submit("http", {
            "route": request.path, "status": response.status_code,
            "handler_ms": (time.monotonic() - g.started) * 1000,
        })
        return response

    @app.errorhandler(BadRequest)
    @app.errorhandler(RequestEntityTooLarge)
    def invalid_request(error):
        app.logger.warning("Invalid GCS request: %s", error)
        return jsonify(error=error.description), error.code

    @app.get("/")
    def index():
        return app.send_static_file("index.html")

    @app.get("/api/status")
    def status():
        return jsonify(runtime.status())

    @app.get("/api/view")
    def view():
        snapshot = runtime.view()
        if snapshot is None:
            return jsonify(error="waiting for rendered camera input"), 503
        return jsonify({**snapshot.data, "image_available": runtime.jpeg(snapshot.data["seq"]) is not None,
                        "dequeued_at": snapshot.frame.dequeued_at,
                        "view_age_s": time.monotonic() - snapshot.frame.dequeued_at})

    @app.get("/api/frame.jpg")
    def frame():
        try:
            sequence = int(request.args["seq"])
        except (KeyError, ValueError):
            app.logger.warning("Invalid image sequence request")
            return jsonify(error="seq must be an integer"), 400
        jpeg = runtime.jpeg(sequence)
        if jpeg is None:
            return jsonify(error="image expired; request a fresh matching view"), 409
        response = send_file(io.BytesIO(jpeg), mimetype="image/jpeg")
        response.headers["X-Rover-Sequence"] = str(sequence)
        return response

    @app.post("/api/heartbeat")
    def heartbeat():
        runtime.controller.heartbeat()
        return jsonify(ok=True)

    @app.post("/api/heartbeat/configure")
    def configure_heartbeat():
        if not request.is_json:
            return jsonify(error="heartbeat configuration requires application/json"), 415
        values = request.get_json()
        try:
            if not isinstance(values, dict) or set(values) != {"enabled", "timeout_s"}:
                raise ValueError("configuration requires exactly enabled and timeout_s")
            configured = runtime.controller.configure_heartbeat(**values)
        except (ValueError, TypeError) as exc:
            app.logger.warning("Rejected heartbeat configuration: %s", exc)
            return jsonify(error=str(exc)), 400
        return jsonify(configured), 200

    @app.post("/api/goal")
    def goal():
        if not request.is_json:
            app.logger.warning("Rejected non-JSON goal")
            return jsonify(error="goal requires application/json"), 415
        values = request.get_json()
        try:
            if not isinstance(values, dict) or set(values) != {"x", "y", "request_id", "epoch"}:
                raise ValueError("goal requires exactly x, y, request_id and epoch")
            accepted = runtime.controller.accept_goal(**values)
        except (ValueError, TypeError) as exc:
            app.logger.warning("Rejected goal: %s", exc)
            runtime.controller.records.submit("goal_rejected", {"input": repr(values), "reason": str(exc)})
            return jsonify(error=str(exc)), 400
        return jsonify(accepted), 202

    @app.post("/api/stop")
    @app.post("/api/cancel")
    def stop():
        reason = "operator cancel" if request.path.endswith("cancel") else "operator stop"
        runtime.controller.records.submit("stop_requested", {"reason": reason})
        runtime.controller.stop(reason)
        return jsonify(state="STOPPED"), 202

    return app
