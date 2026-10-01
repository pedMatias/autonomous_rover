"""LAN dashboard API: hypothetical goals only, no motor-command endpoint."""

import io
from pathlib import Path
from urllib.parse import urlsplit

from flask import Flask, jsonify, request, send_file
from werkzeug.exceptions import BadRequest, RequestEntityTooLarge

from .core import validate_goal
from .runtime import PreviewRuntime, recording_bytes


def create_app(runtime: PreviewRuntime) -> Flask:
    app = Flask(__name__, static_folder=str(Path(__file__).parent / "static"), static_url_path="/static")
    app.config["MAX_CONTENT_LENGTH"] = 1024

    @app.after_request
    def response_headers(response):
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; img-src 'self' blob:; style-src 'self'; "
            "script-src 'self'; connect-src 'self'; frame-ancestors 'none'"
        )
        return response

    @app.before_request
    def validate_write():
        if request.method == "POST":
            origin = request.headers.get("Origin")
            if request.headers.get("X-Rover-Preview") != "1" or (
                origin is not None and urlsplit(origin).netloc != request.host
            ):
                app.logger.warning("Rejected cross-origin or non-preview goal request")
                return jsonify(error="same-origin preview requests require X-Rover-Preview: 1"), 403
        return None

    @app.errorhandler(BadRequest)
    def bad_request(error):
        app.logger.warning("Invalid preview request: %s", error)
        return jsonify(error="invalid JSON request"), 400

    @app.errorhandler(RequestEntityTooLarge)
    def oversized_request(error):
        app.logger.warning("Rejected oversized preview request")
        return jsonify(error="request body must not exceed 1024 bytes"), 413

    @app.route("/")
    def index():
        return app.send_static_file("index.html")

    @app.route("/api/config")
    def config():
        return jsonify({
            **runtime.engine.config(),
            "recorded_decision": runtime.source.recorded_decision,
        })

    @app.route("/api/health")
    def health():
        return jsonify(runtime.health())

    @app.route("/api/snapshot")
    def latest():
        return jsonify(runtime.engine.latest().data)

    @app.route("/api/view")
    @app.route("/api/frozen")
    def view():
        frozen = request.path == "/api/frozen"
        snapshot = runtime.view(frozen)
        if snapshot is None:
            return jsonify(error="no frozen failure available"), 404
        return jsonify({
            **snapshot.data,
            "image_available": runtime.jpeg(snapshot.data["seq"], frozen) is not None,
            "has_camera_frame": snapshot.frame is not None,
        })

    @app.route("/api/frame.jpg")
    @app.route("/api/frozen/frame.jpg")
    def frame():
        try:
            sequence = int(request.args["seq"])
        except (KeyError, ValueError):
            app.logger.warning("Invalid frame sequence request")
            return jsonify(error="seq must be an integer"), 400
        jpeg = runtime.jpeg(sequence, frozen="/frozen/" in request.path)
        if jpeg is None:
            return jsonify(error="frame not available; fetch a fresh view"), 409
        response = send_file(io.BytesIO(jpeg), mimetype="image/jpeg")
        response.headers["X-Preview-Sequence"] = str(sequence)
        return response

    @app.route("/api/goal", methods=["POST"])
    def goal():
        if not request.is_json:
            app.logger.warning("Rejected non-JSON preview goal")
            return jsonify(error="goal requires application/json"), 415
        values = request.get_json()
        try:
            if not isinstance(values, dict) or set(values) != {"x", "y"}:
                raise ValueError("goal must contain exactly x and y")
            target = validate_goal(values["x"], values["y"])
            revision = runtime.engine.set_goal(target)
        except ValueError as exc:
            app.logger.warning("Rejected preview goal: %s", exc)
            return jsonify(error=str(exc)), 400
        app.logger.info("Hypothetical goal %s accepted, revision %s", target, revision)
        return jsonify(goal=target, goal_revision=revision, preview_only=True), 202

    @app.route("/api/goal/cancel", methods=["POST"])
    def cancel():
        revision = runtime.engine.set_goal(None)
        app.logger.info("Hypothetical goal cleared, revision %s", revision)
        return jsonify(goal=None, goal_revision=revision, preview_only=True), 202

    @app.route("/api/frozen/download")
    def download():
        snapshot = runtime.engine.frozen()
        if snapshot is None or snapshot.frame is None:
            return jsonify(error="no frozen camera frame available"), 404
        if request.args.get("seq") != str(snapshot.data["seq"]):
            return jsonify(error="frozen failure changed; fetch it again before downloading"), 409
        data = recording_bytes(snapshot, runtime.engine)
        return send_file(io.BytesIO(data), mimetype="application/octet-stream",
                         as_attachment=True, download_name=f"preview-failure-{snapshot.data['seq']}.npz")

    return app
