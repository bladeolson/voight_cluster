#!/usr/bin/env python3
"""
Simple MJPEG Camera Stream Server
==================================
Streams camera feed as MJPEG for viewing in browser.
Run on me.local to view the robot arm.
"""

from flask import Flask, Response
import cv2
import time

app = Flask(__name__)

# Camera settings
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
WARMUP_FRAMES = 10


def generate_frames():
    """Generate MJPEG frames from camera."""
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    # Warm up camera
    for _ in range(WARMUP_FRAMES):
        cap.read()
        time.sleep(0.05)
    
    print(f"Camera opened: {cap.isOpened()}")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame capture failed, retrying...")
            time.sleep(0.1)
            continue
        
        # Encode as JPEG
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        frame_bytes = buffer.tobytes()
        
        # Yield as MJPEG
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route('/')
def index():
    """Simple HTML page with video."""
    return '''
<!DOCTYPE html>
<html>
<head>
    <title>ME Camera Stream - 目</title>
    <style>
        body {
            background: #0a0a0f;
            color: white;
            font-family: monospace;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 20px;
        }
        h1 { color: #e63946; }
        img {
            max-width: 100%;
            border: 2px solid #333;
            border-radius: 8px;
        }
    </style>
</head>
<body>
    <h1>目 ME Camera Stream</h1>
    <img src="/stream" alt="Camera Stream">
</body>
</html>
'''


@app.route('/stream')
def stream():
    """MJPEG stream endpoint."""
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/snapshot')
def snapshot():
    """Single frame snapshot."""
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    # Warm up
    for _ in range(5):
        cap.read()
    
    ret, frame = cap.read()
    cap.release()
    
    if ret:
        _, buffer = cv2.imencode('.jpg', frame)
        return Response(buffer.tobytes(), mimetype='image/jpeg')
    else:
        return "Failed to capture", 500


if __name__ == '__main__':
    print("=" * 50)
    print("  ME Camera Stream Server - 目")
    print("  Stream: http://0.0.0.0:8028/stream")
    print("  Snapshot: http://0.0.0.0:8028/snapshot")
    print("=" * 50)
    app.run(host='0.0.0.0', port=8028, threaded=True)

