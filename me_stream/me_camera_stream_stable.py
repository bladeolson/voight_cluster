#!/usr/bin/env python3
from flask import Flask, Response, jsonify, request
from flask_cors import CORS
import cv2
import time
import base64
import requests
import threading

app = Flask(__name__)
CORS(app)

camera_lock = threading.Lock()
latest_frame = None
cap = None
fail_count = 0

# Prefer opening by device path (more reliable than numeric index on this camera)
CAMERA_DEVICES = [
    "/dev/video1",
    "/dev/video2",
    "/dev/video0",
]

FRAME_WIDTH = 2560
FRAME_HEIGHT = 960


def init_camera() -> bool:
    global cap, fail_count

    try:
        if cap is not None:
            cap.release()
    except Exception:
        pass

    cap = None
    fail_count = 0

    for dev in CAMERA_DEVICES:
        c = cv2.VideoCapture(dev)
        if not c.isOpened():
            c.release()
            continue

        c.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

        ok = False
        for _ in range(12):
            ret, frame = c.read()
            if ret and frame is not None and frame.size > 0:
                ok = True
                break
            time.sleep(0.05)

        if ok:
            cap = c
            print(f"[ME] Camera opened: {dev}")
            return True

        c.release()

    print("[ME] No camera device opened.")
    return False


def get_frame():
    global latest_frame, cap, fail_count

    with camera_lock:
        if cap is None or not cap.isOpened():
            init_camera()

        if cap is None or not cap.isOpened():
            return latest_frame

        ret, frame = cap.read()
        if ret and frame is not None and frame.size > 0:
            latest_frame = frame
            fail_count = 0
            return frame

        fail_count += 1
        if fail_count >= 25:
            print("[ME] Camera read failing; reinitializing...")
            init_camera()
            fail_count = 0

    return latest_frame


def generate_frames():
    while True:
        frame = get_frame()
        if frame is None:
            time.sleep(0.1)
            continue

        ok, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            time.sleep(0.05)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
        )


@app.route("/")
def index():
    return (
        '<html><body style="background:#111;text-align:center;padding:20px">'
        '<h1 style="color:#e63946">目 ME Node - 3D Vision</h1>'
        '<img src="/stream" style="max-width:100%;border-radius:8px">'
        '<br><br><a href="/snapshot" style="color:#e63946">Get Snapshot</a>'
        ' | <a href="/analyze" style="color:#e63946">Analyze Scene</a>'
        "</body></html>"
    )


@app.route("/stream")
def stream():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/snapshot")
def snapshot():
    frame = get_frame()
    if frame is None:
        return jsonify({"error": "No frame available"}), 500
    ok, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        return jsonify({"error": "Encode failed"}), 500
    return Response(buffer.tobytes(), mimetype="image/jpeg")


@app.route("/snapshot_base64")
def snapshot_base64():
    frame = get_frame()
    if frame is None:
        return jsonify({"error": "No frame available"}), 500
    small = cv2.resize(frame, (640, 240))
    ok, buffer = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return jsonify({"error": "Encode failed"}), 500
    b64 = base64.b64encode(buffer).decode("utf-8")
    return jsonify({"image": b64})


@app.route("/analyze", methods=["GET", "POST"])
def analyze():
    prompt = request.args.get("prompt", "Describe what you see in this image concisely.")
    frame = get_frame()
    if frame is None:
        return jsonify({"error": "No frame available"}), 500

    small = cv2.resize(frame, (640, 240))
    ok, buffer = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return jsonify({"error": "Encode failed"}), 500
    b64 = base64.b64encode(buffer).decode("utf-8")

    try:
        response = requests.post(
            "http://localhost:11434/api/generate",
            json={
                "model": "llava:7b",
                "prompt": prompt,
                "images": [b64],
                "stream": False,
            },
            timeout=120,
        )
        result = response.json()
        return jsonify(
            {
                "description": result.get("response", "No response"),
                "prompt": prompt,
            }
        )
    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == "__main__":
    print("ME Node Vision Server at http://0.0.0.0:8028")
    print("Endpoints: /stream, /snapshot, /snapshot_base64, /analyze")
    init_camera()
    app.run(host="0.0.0.0", port=8028, threaded=True)


