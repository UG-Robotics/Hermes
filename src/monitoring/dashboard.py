"""
Monitoring dashboard — one web page to watch and drive the whole robot.

It runs a simulated (or real) Runtime and exposes:

    /                 the dashboard UI
    /stream           Server-Sent Events: live logs, telemetry, state, comms
    /camera.mjpg      the live camera feed (synthetic or real) as MJPEG
    /api/snapshot     the current value of every channel (for first paint)
    /api/event        POST inject a state-machine event
    /api/manual       POST toggle manual mode / send a manual drive target
    /api/scenarios    GET list scenarios / POST run or cancel one

Built on Flask + SSE + MJPEG so it needs no build step and runs comfortably on
a Raspberry Pi 4. Because everything already flows through the TelemetryHub, the
dashboard is just a window onto that hub — it shows the same stream whether the
hardware is real or simulated.
"""

from __future__ import annotations

import json
import queue
import time

from flask import Flask, Response, request, jsonify, render_template

from utils.logger import get_logger
from utils.telemetry_hub import get_hub

logger = get_logger("Dashboard")


def create_app(runtime):
    app = Flask(__name__)
    hub = get_hub()
    app.config["runtime"] = runtime
    app.config["scenario_player"] = None

    # ---------------------------------------------------------------- pages
    @app.route("/")
    def index():
        return render_template("index.html")

    # ---------------------------------------------------------- live stream
    @app.route("/stream")
    def stream():
        def gen():
            q = hub.subscribe()
            # Prime the client with the current snapshot so it paints instantly.
            yield _sse({"channel": "snapshot", "data": hub.snapshot()})
            try:
                while True:
                    try:
                        record = q.get(timeout=15)
                        yield _sse(record)
                    except queue.Empty:
                        # Heartbeat comment keeps the connection from timing out.
                        yield ": keepalive\n\n"
            except GeneratorExit:
                pass
            finally:
                hub.unsubscribe(q)
        return Response(gen(), mimetype="text/event-stream",
                        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    # ------------------------------------------------------------- camera
    @app.route("/camera.mjpg")
    def camera_stream():
        cam = runtime.camera

        def gen():
            boundary = b"--frame"
            while True:
                if cam is None:
                    time.sleep(0.5)
                    continue
                try:
                    jpeg = cam.get_jpeg()
                except Exception as e:
                    logger.error(f"Camera frame error: {e}")
                    time.sleep(0.2)
                    continue
                yield (boundary + b"\r\nContent-Type: image/jpeg\r\n"
                       + f"Content-Length: {len(jpeg)}\r\n\r\n".encode() + jpeg + b"\r\n")
                time.sleep(1.0 / max(1, cam.fps))

        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    # --------------------------------------------------------------- APIs
    @app.route("/api/snapshot")
    def snapshot():
        return jsonify(hub.snapshot())

    @app.route("/api/event", methods=["POST"])
    def api_event():
        body = request.get_json(force=True, silent=True) or {}
        name = body.get("event")
        if not name:
            return jsonify({"ok": False, "error": "missing 'event'"}), 400
        ok = runtime.inject_event(name, source="dashboard")
        return jsonify({"ok": ok})

    @app.route("/api/manual", methods=["POST"])
    def api_manual():
        body = request.get_json(force=True, silent=True) or {}
        listener = runtime.listener
        if listener is None:
            return jsonify({"ok": False, "error": "no manual listener"}), 400
        if "active" in body:
            listener.set_remote_manual(bool(body["active"]))
        if "target" in body and body["target"] is not None:
            t = body["target"]
            if t == "clear":
                listener.clear_remote_target()  # release back to keyboard
            else:
                listener.set_remote_target(t.get("speed", 0), t.get("steer", 0),
                                           t.get("action", "STOP"))
        return jsonify({"ok": True, "manual": listener.is_manual_mode_active()})

    @app.route("/api/scenarios", methods=["GET", "POST"])
    def api_scenarios():
        from monitoring.scenario import list_scenarios, load_scenario, ScenarioPlayer

        if request.method == "GET":
            return jsonify({"scenarios": list_scenarios()})

        body = request.get_json(force=True, silent=True) or {}
        action = body.get("action", "run")

        player = app.config.get("scenario_player")
        if action == "cancel":
            if player:
                player.cancel()
            return jsonify({"ok": True})

        name = body.get("scenario")
        if not name:
            return jsonify({"ok": False, "error": "missing 'scenario'"}), 400
        try:
            scenario = load_scenario(name)
        except FileNotFoundError as e:
            return jsonify({"ok": False, "error": str(e)}), 404

        player = ScenarioPlayer(runtime, scenario)
        app.config["scenario_player"] = player
        player.start()
        return jsonify({"ok": True, "name": scenario.get("name")})

    @app.route("/api/logs")
    def api_logs():
        limit = request.args.get("limit", default=200, type=int)
        return jsonify({"logs": hub.recent_logs(limit)})

    return app


def _sse(record) -> str:
    return f"data: {json.dumps(record)}\n\n"


def run_dashboard(runtime, host: str = "0.0.0.0", port: int = 5000) -> None:
    """Start the runtime loop and serve the dashboard (blocking)."""
    runtime.start_background()
    app = create_app(runtime)
    logger.info(f"Dashboard live at http://{host}:{port}  (Ctrl-C to stop)")
    # threaded=True so SSE + MJPEG + API calls are served concurrently.
    app.run(host=host, port=port, threaded=True, debug=False, use_reloader=False)
