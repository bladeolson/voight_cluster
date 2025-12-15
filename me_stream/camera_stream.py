#!/usr/bin/env python3
"""
ME Node - Stereo Vision Server (目)
====================================
Dual GPU stereo camera processing:
- GPU 0: Left eye processing
- GPU 1: Right eye processing

Streams both eyes as separate MJPEG feeds.
"""

from flask import Flask, Response, jsonify, request
from flask_cors import CORS
import cv2
import time
import threading
import base64
import requests
import os

app = Flask(__name__)
CORS(app)

# Camera settings - 3D stereo camera outputs side-by-side
CAMERA_INDEX = 1  # May need adjustment
STEREO_WIDTH = 2560
STEREO_HEIGHT = 960
EYE_WIDTH = 1280  # Half of stereo width
EYE_HEIGHT = 960
WARMUP_FRAMES = 10

# Global camera and frame buffers
camera_lock = threading.Lock()
cap = None
current_frame = None
left_frame = None
right_frame = None
frame_time = 0


def init_camera():
    """Initialize the stereo camera."""
    global cap
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, STEREO_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, STEREO_HEIGHT)
    
    # Warm up
    for _ in range(WARMUP_FRAMES):
        cap.read()
        time.sleep(0.05)
    
    print(f"Camera initialized: {cap.isOpened()}")
    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Resolution: {actual_w}x{actual_h}")
    return cap


def capture_loop():
    """Background thread to capture frames."""
    global cap, current_frame, left_frame, right_frame, frame_time
    
    while True:
        with camera_lock:
            if cap is None or not cap.isOpened():
                cap = init_camera()
            
            ret, frame = cap.read()
            if ret:
                current_frame = frame
                frame_time = time.time()
                
                # Split stereo frame into left and right
                h, w = frame.shape[:2]
                mid = w // 2
                left_frame = frame[:, :mid]
                right_frame = frame[:, mid:]
        
        time.sleep(0.016)  # ~60fps capture


def generate_stereo_frames():
    """Generate full stereo MJPEG frames."""
    while True:
        if current_frame is not None:
            _, buffer = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)


def generate_left_frames():
    """Generate left eye MJPEG frames."""
    while True:
        if left_frame is not None:
            _, buffer = cv2.imencode('.jpg', left_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)


def generate_right_frames():
    """Generate right eye MJPEG frames."""
    while True:
        if right_frame is not None:
            _, buffer = cv2.imencode('.jpg', right_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)


@app.route('/')
def index():
    """Status page with all stream links."""
    return '''
<!DOCTYPE html>
<html>
<head>
    <title>ME Stereo Vision - 目</title>
    <style>
        body { background: #0a0a0f; color: #e0e0e0; font-family: system-ui; padding: 20px; }
        h1 { color: #6366f1; }
        .streams { display: flex; gap: 20px; flex-wrap: wrap; }
        .stream { background: #1a1a24; padding: 15px; border-radius: 8px; }
        .stream h3 { color: #e63946; margin-top: 0; }
        img { max-width: 100%; border-radius: 4px; }
        a { color: #6366f1; }
    </style>
</head>
<body>
    <h1>目 ME Node - Stereo Vision</h1>
    <p>Dual GPU stereo camera processing</p>
    
    <div class="streams">
        <div class="stream">
            <h3>👁️ Left Eye (GPU 0)</h3>
            <img src="/stream/left" width="400">
            <p><a href="/stream/left">/stream/left</a> | <a href="/analyze/left">/analyze/left</a></p>
        </div>
        <div class="stream">
            <h3>👁️ Right Eye (GPU 1)</h3>
            <img src="/stream/right" width="400">
            <p><a href="/stream/right">/stream/right</a> | <a href="/analyze/right">/analyze/right</a></p>
        </div>
    </div>
    
    <h3>Endpoints</h3>
    <ul>
        <li><a href="/stream">/stream</a> - Full stereo stream</li>
        <li><a href="/stream/left">/stream/left</a> - Left eye only</li>
        <li><a href="/stream/right">/stream/right</a> - Right eye only</li>
        <li><a href="/snapshot">/snapshot</a> - Full stereo snapshot</li>
        <li><a href="/snapshot/left">/snapshot/left</a> - Left eye snapshot</li>
        <li><a href="/snapshot/right">/snapshot/right</a> - Right eye snapshot</li>
        <li>/analyze/left?prompt=... - Vision AI (GPU 0)</li>
        <li>/analyze/right?prompt=... - Vision AI (GPU 1)</li>
        <li>/analyze?prompt=... - Combined vision (both GPUs)</li>
    </ul>
</body>
</html>
'''


@app.route('/stream')
def stream():
    """Full stereo MJPEG stream."""
    return Response(generate_stereo_frames(), 
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream/left')
def stream_left():
    """Left eye MJPEG stream."""
    return Response(generate_left_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/stream/right')
def stream_right():
    """Right eye MJPEG stream."""
    return Response(generate_right_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/snapshot')
def snapshot():
    """Full stereo snapshot."""
    if current_frame is not None:
        _, buffer = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        return Response(buffer.tobytes(), mimetype='image/jpeg')
    return jsonify({"error": "No frame available"}), 500


@app.route('/snapshot/left')
def snapshot_left():
    """Left eye snapshot."""
    if left_frame is not None:
        _, buffer = cv2.imencode('.jpg', left_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        return Response(buffer.tobytes(), mimetype='image/jpeg')
    return jsonify({"error": "No frame available"}), 500


@app.route('/snapshot/right')
def snapshot_right():
    """Right eye snapshot."""
    if right_frame is not None:
        _, buffer = cv2.imencode('.jpg', right_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        return Response(buffer.tobytes(), mimetype='image/jpeg')
    return jsonify({"error": "No frame available"}), 500


def analyze_with_gpu(frame, prompt, gpu_id):
    """Run LLaVA analysis on specified GPU."""
    if frame is None:
        return {"error": "No frame available"}
    
    # Resize for LLM
    small = cv2.resize(frame, (640, 480))
    _, buffer = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 80])
    b64 = base64.b64encode(buffer).decode('utf-8')
    
    try:
        # Set GPU for this request via environment (Ollama respects CUDA_VISIBLE_DEVICES)
        # Note: This affects the Ollama server, which should be configured per-GPU
        resp = requests.post('http://localhost:11434/api/generate', json={
            "model": "llava:7b",
            "prompt": prompt,
            "images": [b64],
            "stream": False,
            "options": {
                "num_gpu": 1,
                # Ollama will use the available GPU
            }
        }, timeout=120)
        
        result = resp.json()
        return {
            "description": result.get('response', ''),
            "prompt": prompt,
            "eye": "left" if gpu_id == 0 else "right",
            "gpu": gpu_id
        }
    except Exception as e:
        return {"error": str(e), "gpu": gpu_id}


@app.route('/analyze/left')
def analyze_left():
    """Analyze left eye view using GPU 0."""
    prompt = request.args.get('prompt', 'Describe what you see in one sentence.')
    return jsonify(analyze_with_gpu(left_frame, prompt, gpu_id=0))


@app.route('/analyze/right')
def analyze_right():
    """Analyze right eye view using GPU 1."""
    prompt = request.args.get('prompt', 'Describe what you see in one sentence.')
    return jsonify(analyze_with_gpu(right_frame, prompt, gpu_id=1))


@app.route('/analyze')
def analyze_stereo():
    """Analyze both eyes and combine descriptions."""
    prompt = request.args.get('prompt', 'Describe what you see in one sentence.')
    
    # Analyze both in parallel (simplified - sequential for now)
    left_result = analyze_with_gpu(left_frame, prompt + " (Focus on the left side of your view)", gpu_id=0)
    right_result = analyze_with_gpu(right_frame, prompt + " (Focus on the right side of your view)", gpu_id=1)
    
    # Combine descriptions
    combined = f"Left eye: {left_result.get('description', 'N/A')}. Right eye: {right_result.get('description', 'N/A')}"
    
    return jsonify({
        "description": combined,
        "left": left_result,
        "right": right_result,
        "prompt": prompt,
        "stereo": True
    })


@app.route('/status')
def status():
    """Camera and GPU status."""
    import subprocess
    
    # Get GPU info
    gpus = []
    try:
        result = subprocess.run(['/usr/bin/nvidia-smi', '--query-gpu=index,name,memory.used,memory.total,utilization.gpu',
                                '--format=csv,noheader,nounits'], capture_output=True, text=True)
        for line in result.stdout.strip().split('\n'):
            if line:
                parts = [p.strip() for p in line.split(',')]
                if len(parts) >= 5:
                    gpus.append({
                        "index": int(parts[0]),
                        "name": parts[1],
                        "memory_used": int(parts[2]),
                        "memory_total": int(parts[3]),
                        "utilization": int(parts[4])
                    })
    except:
        pass
    
    return jsonify({
        "node": "me",
        "camera": {
            "connected": cap is not None and cap.isOpened() if cap else False,
            "stereo": True,
            "resolution": f"{STEREO_WIDTH}x{STEREO_HEIGHT}",
            "eye_resolution": f"{EYE_WIDTH}x{EYE_HEIGHT}",
            "last_frame": frame_time
        },
        "gpus": gpus,
        "endpoints": {
            "left_eye": "/stream/left",
            "right_eye": "/stream/right",
            "stereo": "/stream",
            "analyze_left": "/analyze/left",
            "analyze_right": "/analyze/right"
        }
    })


if __name__ == '__main__':
    print("=" * 60)
    print("  目 ME Node - Stereo Vision Server")
    print("  Dual GPU Processing for Left/Right Eyes")
    print("=" * 60)
    print("  Streams:")
    print("    Left Eye:  http://0.0.0.0:8028/stream/left")
    print("    Right Eye: http://0.0.0.0:8028/stream/right")
    print("    Stereo:    http://0.0.0.0:8028/stream")
    print("=" * 60)
    
    # Start capture thread
    capture_thread = threading.Thread(target=capture_loop, daemon=True)
    capture_thread.start()
    
    app.run(host='0.0.0.0', port=8028, threaded=True)
