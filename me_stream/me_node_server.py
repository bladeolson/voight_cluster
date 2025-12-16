#!/usr/bin/env python3
"""
ME Node - Vision & LiDAR Server (目)
=====================================
Stereo camera + 2D LiDAR integration for VOIGHT CLUSTER.

Features:
- Stereo camera streaming (left/right eyes)
- 2D LiDAR scanning with real-time visualization
- Scene analysis via LLaVA
"""

from flask import Flask, Response, jsonify, request
from flask_cors import CORS
import cv2
import time
import threading
import base64
import requests
import json
import math
import glob
import os

app = Flask(__name__)
CORS(app)

# =============================================================================
# Configuration
# =============================================================================

# 3D Stereo Camera settings (for robot EYES)
# 3D USB Camera is at /dev/video0 and /dev/video1
STEREO_CAMERA_DEVICES = ["/dev/video0", "/dev/video1"]
STEREO_WIDTH = 2560
STEREO_HEIGHT = 960
EYE_WIDTH = 1280
EYE_HEIGHT = 960

# Arm POV Camera settings (Razer Kiyo on robot arm)
# Razer Kiyo is at /dev/video2 and /dev/video3
ARM_CAMERA_DEVICE = "/dev/video2"
ARM_CAMERA_WIDTH = 1280
ARM_CAMERA_HEIGHT = 720

# LiDAR settings
LIDAR_BAUDRATE = 115200
LIDAR_TIMEOUT = 3

# =============================================================================
# Global State
# =============================================================================

# Stereo Camera state (EYES)
camera_lock = threading.Lock()
cap = None
current_frame = None
left_frame = None
right_frame = None
frame_time = 0
camera_fail_count = 0

# Arm Camera state (POV)
arm_camera_lock = threading.Lock()
arm_cap = None
arm_frame = None
arm_frame_time = 0
arm_camera_connected = False

# LiDAR state
lidar_lock = threading.Lock()
lidar_device = None
lidar_scan = []  # List of (angle, distance) tuples
lidar_scan_time = 0
lidar_connected = False
lidar_port = None
lidar_error = None

# =============================================================================
# Camera Functions
# =============================================================================

def find_stereo_camera() -> bool:
    """Find and initialize the 3D stereo camera for EYES."""
    global cap, camera_fail_count

    try:
        if cap is not None:
            cap.release()
    except Exception:
        pass

    cap = None
    camera_fail_count = 0

    for dev in STEREO_CAMERA_DEVICES:
        c = cv2.VideoCapture(dev)
        if not c.isOpened():
            c.release()
            continue

        c.set(cv2.CAP_PROP_FRAME_WIDTH, STEREO_WIDTH)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, STEREO_HEIGHT)

        # Warm up and verify
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

    print("[ME] No camera device found.")
    return False


def camera_capture_loop():
    """Background thread for camera capture."""
    global cap, current_frame, left_frame, right_frame, frame_time, camera_fail_count

    while True:
        with camera_lock:
            if cap is None or not cap.isOpened():
                find_stereo_camera()

            if cap is None or not cap.isOpened():
                time.sleep(0.5)
                continue

            ret, frame = cap.read()
            if ret and frame is not None and frame.size > 0:
                current_frame = frame
                frame_time = time.time()
                camera_fail_count = 0

                # Split stereo frame
                h, w = frame.shape[:2]
                mid = w // 2
                left_frame = frame[:, :mid]
                right_frame = frame[:, mid:]
            else:
                camera_fail_count += 1
                if camera_fail_count >= 25:
                    print("[ME] Camera failing, reinitializing...")
                    find_stereo_camera()
                    camera_fail_count = 0

        time.sleep(0.016)  # ~60fps


# =============================================================================
# Arm Camera Functions (Razer POV)
# =============================================================================

def find_arm_camera() -> bool:
    """Find and initialize the arm POV camera (Razer)."""
    global arm_cap, arm_camera_connected
    
    with arm_camera_lock:
        if arm_cap is not None:
            arm_cap.release()
        
        arm_cap = cv2.VideoCapture(ARM_CAMERA_DEVICE)
        if arm_cap.isOpened():
            arm_cap.set(cv2.CAP_PROP_FRAME_WIDTH, ARM_CAMERA_WIDTH)
            arm_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ARM_CAMERA_HEIGHT)
            arm_cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            arm_camera_connected = True
            print(f"[ME] Arm camera opened: {ARM_CAMERA_DEVICE}")
            return True
        
        arm_camera_connected = False
        print(f"[ME] Arm camera not found at {ARM_CAMERA_DEVICE}")
        return False


def arm_camera_capture_loop():
    """Background thread for arm camera capture."""
    global arm_cap, arm_frame, arm_frame_time, arm_camera_connected
    
    while True:
        with arm_camera_lock:
            if arm_cap is None or not arm_cap.isOpened():
                find_arm_camera()
                time.sleep(1)
                continue
            
            ret, frame = arm_cap.read()
            if ret:
                arm_frame = frame.copy()
                arm_frame_time = time.time()
                arm_camera_connected = True
            else:
                arm_camera_connected = False
        
        time.sleep(0.033)  # ~30fps


# =============================================================================
# LiDAR Functions
# =============================================================================

def find_lidar_port():
    """Auto-detect LiDAR serial port."""
    lidar_keywords = [
        "lidar", "rplidar", "ydlidar", "slamtec", "cp210", "silabs", "pl2303", "ch340"
    ]
    
    # Check by-id symlinks first (more stable)
    by_id_paths = sorted(glob.glob("/dev/serial/by-id/*"))
    for p in by_id_paths:
        name = os.path.basename(p).lower()
        if any(k in name for k in lidar_keywords):
            return os.path.realpath(p)
    
    # Fallback to ttyUSB devices
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if candidates:
        return candidates[0]
    
    return None


def init_lidar():
    """Initialize LiDAR connection."""
    global lidar_device, lidar_connected, lidar_port, lidar_error
    
    try:
        from rplidar import RPLidar
    except ImportError:
        lidar_error = "rplidar library not installed (pip install rplidar-roboticia)"
        print(f"[ME] {lidar_error}")
        return False
    
    port = find_lidar_port()
    if not port:
        lidar_error = "No LiDAR port found"
        print(f"[ME] {lidar_error}")
        return False
    
    try:
        lidar_device = RPLidar(port, baudrate=LIDAR_BAUDRATE, timeout=LIDAR_TIMEOUT)
        info = lidar_device.get_info()
        health = lidar_device.get_health()
        
        lidar_port = port
        lidar_connected = True
        lidar_error = None
        
        print(f"[ME] LiDAR connected: {port}")
        print(f"[ME] LiDAR info: {info}")
        print(f"[ME] LiDAR health: {health}")
        return True
        
    except Exception as e:
        lidar_error = str(e)
        lidar_connected = False
        print(f"[ME] LiDAR init error: {e}")
        return False


def lidar_scan_loop():
    """Background thread for LiDAR scanning."""
    global lidar_device, lidar_scan, lidar_scan_time, lidar_connected, lidar_error
    
    while True:
        if not lidar_connected:
            if not init_lidar():
                time.sleep(5)
                continue
        
        try:
            # Use iter_scans for continuous scanning
            for scan in lidar_device.iter_scans():
                with lidar_lock:
                    # scan is list of (quality, angle, distance) tuples
                    # Convert to (angle_deg, distance_mm)
                    lidar_scan = [(point[1], point[2]) for point in scan if point[2] > 0]
                    lidar_scan_time = time.time()
                    
        except Exception as e:
            print(f"[ME] LiDAR scan error: {e}")
            lidar_error = str(e)
            lidar_connected = False
            
            # Try to clean up
            try:
                if lidar_device:
                    lidar_device.stop()
                    lidar_device.disconnect()
            except:
                pass
            lidar_device = None
            
            time.sleep(2)


def stop_lidar():
    """Stop LiDAR scanning."""
    global lidar_device, lidar_connected
    
    if lidar_device:
        try:
            lidar_device.stop()
            lidar_device.stop_motor()
            lidar_device.disconnect()
        except:
            pass
    lidar_device = None
    lidar_connected = False


# =============================================================================
# Stream Generators
# =============================================================================

def generate_stereo_frames():
    """Generate full stereo MJPEG frames."""
    while True:
        if current_frame is not None:
            ok, buffer = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)


def generate_left_frames():
    """Generate left eye MJPEG frames."""
    while True:
        if left_frame is not None:
            ok, buffer = cv2.imencode('.jpg', left_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)


def generate_right_frames():
    """Generate right eye MJPEG frames."""
    while True:
        if right_frame is not None:
            ok, buffer = cv2.imencode('.jpg', right_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)


# =============================================================================
# API Routes - Camera
# =============================================================================

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
        ok, buffer = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            return Response(buffer.tobytes(), mimetype='image/jpeg')
    return jsonify({"error": "No frame available"}), 500


@app.route('/snapshot/left')
def snapshot_left():
    """Left eye snapshot."""
    if left_frame is not None:
        ok, buffer = cv2.imencode('.jpg', left_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            return Response(buffer.tobytes(), mimetype='image/jpeg')
    return jsonify({"error": "No frame available"}), 500


@app.route('/snapshot/right')
def snapshot_right():
    """Right eye snapshot."""
    if right_frame is not None:
        ok, buffer = cv2.imencode('.jpg', right_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            return Response(buffer.tobytes(), mimetype='image/jpeg')
    return jsonify({"error": "No frame available"}), 500


# =============================================================================
# API Routes - Arm Camera (Razer POV)
# =============================================================================

def generate_arm_frames():
    """Generate arm camera frames for MJPEG stream."""
    while True:
        if arm_frame is not None:
            ok, buffer = cv2.imencode('.jpg', arm_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)


@app.route('/stream/arm')
def stream_arm():
    """Arm POV camera MJPEG stream (Razer)."""
    return Response(generate_arm_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/snapshot/arm')
def snapshot_arm():
    """Arm POV camera snapshot."""
    if arm_frame is not None:
        ok, buffer = cv2.imencode('.jpg', arm_frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if ok:
            return Response(buffer.tobytes(), mimetype='image/jpeg')
    return jsonify({"error": "Arm camera not available"}), 500


@app.route('/arm/status')
def arm_camera_status():
    """Get arm camera status."""
    return jsonify({
        "connected": arm_camera_connected,
        "device": ARM_CAMERA_DEVICE,
        "resolution": f"{ARM_CAMERA_WIDTH}x{ARM_CAMERA_HEIGHT}" if arm_camera_connected else None
    })


@app.route('/lidar/embed')
def lidar_embed():
    """Embeddable LiDAR visualization (for KOKORO dashboard iframe)."""
    return '''<!DOCTYPE html>
<html>
<head>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        html, body { width: 100%; height: 100%; background: #0a0a0f; overflow: hidden; }
        canvas { display: block; margin: auto; }
    </style>
</head>
<body>
    <canvas id="lidar"></canvas>
    <script>
        const canvas = document.getElementById('lidar');
        const ctx = canvas.getContext('2d');
        
        function resize() {
            const size = Math.min(window.innerWidth, window.innerHeight);
            canvas.width = size;
            canvas.height = size;
        }
        resize();
        window.addEventListener('resize', resize);
        
        const MAX_MM = 5000;
        
        function draw(scan) {
            const W = canvas.width, H = canvas.height;
            const cx = W/2, cy = H/2;
            const maxR = Math.min(cx, cy) - 10;
            const scale = maxR / MAX_MM;
            
            ctx.fillStyle = '#0a0a0f';
            ctx.fillRect(0, 0, W, H);
            
            ctx.strokeStyle = '#1a1a24';
            ctx.lineWidth = 1;
            for (let r = 1000; r <= MAX_MM; r += 1000) {
                ctx.beginPath();
                ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);
                ctx.stroke();
            }
            
            ctx.beginPath();
            ctx.moveTo(cx, 0); ctx.lineTo(cx, H);
            ctx.moveTo(0, cy); ctx.lineTo(W, cy);
            ctx.stroke();
            
            ctx.fillStyle = '#ff2d55';
            ctx.beginPath();
            ctx.arc(cx, cy, 4, 0, Math.PI * 2);
            ctx.fill();
            
            if (scan && scan.length > 0) {
                for (let i = 0; i < scan.length; i++) {
                    const a = scan[i][0], d = scan[i][1];
                    const rad = (a - 90) * Math.PI / 180;
                    const x = cx + Math.cos(rad) * d * scale;
                    const y = cy + Math.sin(rad) * d * scale;
                    const t = Math.min(d / MAX_MM, 1);
                    ctx.fillStyle = 'rgb(' + 
                        Math.floor(255*(1-t)) + ',' + 
                        Math.floor(45*(1-t) + 212*t) + ',' + 
                        Math.floor(85*(1-t) + 255*t) + ')';
                    ctx.beginPath();
                    ctx.arc(x, y, 2.5, 0, Math.PI * 2);
                    ctx.fill();
                }
            }
        }
        
        async function update() {
            try {
                const r = await fetch('/lidar/data');
                const d = await r.json();
                draw(d.scan);
            } catch(e) { draw([]); }
        }
        
        draw([]);
        update();
        setInterval(update, 200);
    </script>
</body>
</html>'''


# =============================================================================
# API Routes - LiDAR
# =============================================================================

@app.route('/lidar/data')
def lidar_data():
    """Get current LiDAR scan data."""
    with lidar_lock:
        scan_copy = list(lidar_scan)
        scan_time_copy = lidar_scan_time
    
    return jsonify({
        "connected": lidar_connected,
        "port": lidar_port,
        "error": lidar_error,
        "timestamp": scan_time_copy,
        "point_count": len(scan_copy),
        "scan": scan_copy  # List of [angle_deg, distance_mm]
    })


@app.route('/lidar/status')
def lidar_status():
    """Get LiDAR status info."""
    return jsonify({
        "connected": lidar_connected,
        "port": lidar_port,
        "error": lidar_error,
        "last_scan_time": lidar_scan_time,
        "point_count": len(lidar_scan)
    })


# =============================================================================
# API Routes - Analysis
# =============================================================================

def analyze_with_llava(frame, prompt):
    """Run LLaVA analysis on a frame."""
    if frame is None:
        return {"error": "No frame available"}
    
    small = cv2.resize(frame, (640, 480))
    ok, buffer = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return {"error": "Encode failed"}
    
    b64 = base64.b64encode(buffer).decode('utf-8')
    
    try:
        resp = requests.post('http://localhost:11434/api/generate', json={
            "model": "llava:7b",
            "prompt": prompt,
            "images": [b64],
            "stream": False,
        }, timeout=120)
        
        result = resp.json()
        return {
            "description": result.get('response', ''),
            "prompt": prompt,
        }
    except Exception as e:
        return {"error": str(e)}


@app.route('/analyze')
def analyze():
    """Analyze current camera view."""
    prompt = request.args.get('prompt', 'Describe what you see concisely.')
    return jsonify(analyze_with_llava(current_frame, prompt))


@app.route('/analyze/left')
def analyze_left():
    """Analyze left eye view."""
    prompt = request.args.get('prompt', 'Describe what you see concisely.')
    return jsonify(analyze_with_llava(left_frame, prompt))


@app.route('/analyze/right')
def analyze_right():
    """Analyze right eye view."""
    prompt = request.args.get('prompt', 'Describe what you see concisely.')
    return jsonify(analyze_with_llava(right_frame, prompt))


# =============================================================================
# API Routes - Status
# =============================================================================

@app.route('/status')
def status():
    """Full node status."""
    import subprocess
    import psutil
    
    # GPU info
    gpus = []
    try:
        result = subprocess.run(
            ['nvidia-smi', '--query-gpu=index,name,memory.used,memory.total,utilization.gpu',
             '--format=csv,noheader,nounits'],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.strip().split('\n'):
            if line:
                parts = [p.strip() for p in line.split(',')]
                if len(parts) >= 5:
                    gpus.append({
                        "index": int(parts[0]),
                        "name": parts[1],
                        "memory_used_mb": int(parts[2]),
                        "memory_total_mb": int(parts[3]),
                        "utilization": int(parts[4])
                    })
    except:
        pass
    
    return jsonify({
        "node": "me",
        "kanji": "目",
        "role": "vision",
        "camera": {
            "connected": cap is not None and cap.isOpened() if cap else False,
            "stereo": True,
            "resolution": f"{STEREO_WIDTH}x{STEREO_HEIGHT}",
            "eye_resolution": f"{EYE_WIDTH}x{EYE_HEIGHT}",
            "last_frame": frame_time
        },
        "arm_camera": {
            "connected": arm_camera_connected,
            "device": ARM_CAMERA_DEVICE,
            "resolution": f"{ARM_CAMERA_WIDTH}x{ARM_CAMERA_HEIGHT}" if arm_camera_connected else None
        },
        "lidar": {
            "connected": lidar_connected,
            "port": lidar_port,
            "error": lidar_error,
            "point_count": len(lidar_scan),
            "last_scan": lidar_scan_time
        },
        "hardware": {
            "cpu_percent": psutil.cpu_percent(),
            "ram_used_gb": round(psutil.virtual_memory().used / (1024**3), 1),
            "ram_total_gb": round(psutil.virtual_memory().total / (1024**3), 1),
            "gpus": gpus
        },
        "version": "0.3.0"
    })


# =============================================================================
# Dashboard UI
# =============================================================================

@app.route('/')
def index():
    """Main dashboard with camera feeds and LiDAR visualization."""
    return '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>目 ME Node - Vision & LiDAR</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500;600&family=Zen+Kaku+Gothic+New:wght@300;500&display=swap');
        
        :root {
            --bg-deep: #050508;
            --bg-panel: #0c0c12;
            --bg-card: #14141c;
            --accent: #ff2d55;
            --accent-dim: #ff2d5540;
            --cyan: #00d4ff;
            --cyan-dim: #00d4ff30;
            --green: #32d74b;
            --text: #e5e5e7;
            --text-dim: #6e6e73;
            --border: #1c1c24;
        }
        
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'IBM Plex Mono', monospace;
            background: var(--bg-deep);
            color: var(--text);
            min-height: 100vh;
            overflow-x: hidden;
        }
        
        /* Animated background */
        body::before {
            content: '';
            position: fixed;
            top: 0; left: 0; right: 0; bottom: 0;
            background: 
                radial-gradient(ellipse at 20% 20%, var(--accent-dim) 0%, transparent 50%),
                radial-gradient(ellipse at 80% 80%, var(--cyan-dim) 0%, transparent 50%);
            pointer-events: none;
            z-index: -1;
        }
        
        header {
            padding: 1.5rem 2rem;
            border-bottom: 1px solid var(--border);
            display: flex;
            align-items: center;
            justify-content: space-between;
            background: var(--bg-panel);
        }
        
        .logo {
            display: flex;
            align-items: center;
            gap: 1rem;
        }
        
        .kanji {
            font-family: 'Zen Kaku Gothic New', sans-serif;
            font-size: 3rem;
            font-weight: 300;
            color: var(--accent);
            text-shadow: 0 0 30px var(--accent-dim);
        }
        
        .title {
            font-size: 0.875rem;
            font-weight: 500;
            letter-spacing: 0.15em;
            text-transform: uppercase;
            color: var(--text-dim);
        }
        
        .status-bar {
            display: flex;
            gap: 1.5rem;
            font-size: 0.75rem;
        }
        
        .status-item {
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }
        
        .dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: var(--text-dim);
        }
        
        .dot.online {
            background: var(--green);
            box-shadow: 0 0 10px var(--green);
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        main {
            display: grid;
            grid-template-columns: 1fr 1fr;
            grid-template-rows: auto auto;
            gap: 1rem;
            padding: 1rem;
            max-width: 1800px;
            margin: 0 auto;
        }
        
        .panel {
            background: var(--bg-card);
            border: 1px solid var(--border);
            border-radius: 12px;
            overflow: hidden;
        }
        
        .panel-header {
            padding: 0.75rem 1rem;
            border-bottom: 1px solid var(--border);
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: var(--bg-panel);
        }
        
        .panel-title {
            font-size: 0.75rem;
            font-weight: 600;
            letter-spacing: 0.1em;
            text-transform: uppercase;
            color: var(--cyan);
        }
        
        .panel-badge {
            font-size: 0.625rem;
            padding: 0.25rem 0.5rem;
            background: var(--cyan-dim);
            color: var(--cyan);
            border-radius: 4px;
        }
        
        .panel-content {
            padding: 1rem;
        }
        
        /* Camera panels */
        .camera-panel img {
            width: 100%;
            height: auto;
            display: block;
            border-radius: 8px;
            background: #000;
        }
        
        /* LiDAR panel spans full width */
        .lidar-panel {
            grid-column: 1 / -1;
        }
        
        .lidar-container {
            display: flex;
            gap: 1rem;
            padding: 1rem;
        }
        
        .lidar-viz {
            flex: 1;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        
        #lidarCanvas {
            background: var(--bg-deep);
            border-radius: 50%;
            box-shadow: 
                0 0 60px var(--cyan-dim),
                inset 0 0 60px rgba(0, 212, 255, 0.05);
        }
        
        .lidar-stats {
            width: 220px;
            display: flex;
            flex-direction: column;
            gap: 0.75rem;
        }
        
        .stat-card {
            background: var(--bg-panel);
            border: 1px solid var(--border);
            border-radius: 8px;
            padding: 0.75rem;
        }
        
        .stat-label {
            font-size: 0.625rem;
            color: var(--text-dim);
            text-transform: uppercase;
            letter-spacing: 0.1em;
            margin-bottom: 0.25rem;
        }
        
        .stat-value {
            font-size: 1.25rem;
            font-weight: 600;
            color: var(--cyan);
        }
        
        .stat-value.accent {
            color: var(--accent);
        }
        
        .stat-unit {
            font-size: 0.75rem;
            color: var(--text-dim);
            margin-left: 0.25rem;
        }
        
        /* Responsive */
        @media (max-width: 1200px) {
            main {
                grid-template-columns: 1fr;
            }
            .lidar-container {
                flex-direction: column;
                align-items: center;
            }
            .lidar-stats {
                width: 100%;
                flex-direction: row;
                flex-wrap: wrap;
            }
            .stat-card {
                flex: 1;
                min-width: 100px;
            }
        }
    </style>
</head>
<body>
    <header>
        <div class="logo">
            <span class="kanji">目</span>
            <div>
                <div class="title">ME Node</div>
                <div style="font-size: 0.7rem; color: var(--text-dim);">Vision & LiDAR</div>
            </div>
        </div>
        <div class="status-bar">
            <div class="status-item">
                <div class="dot" id="cameraStatus"></div>
                <span>Camera</span>
            </div>
            <div class="status-item">
                <div class="dot" id="lidarStatus"></div>
                <span>LiDAR</span>
            </div>
        </div>
    </header>
    
    <main>
        <div class="panel camera-panel">
            <div class="panel-header">
                <span class="panel-title">👁️ Left Eye</span>
                <span class="panel-badge">3D Cam</span>
            </div>
            <div class="panel-content">
                <img src="/stream/left" alt="Left Eye" onerror="this.style.opacity=0.3">
            </div>
        </div>
        
        <div class="panel camera-panel arm-camera">
            <div class="panel-header">
                <span class="panel-title">🦾 Arm POV</span>
                <span class="panel-badge" id="armCamBadge">Razer</span>
            </div>
            <div class="panel-content">
                <img src="/stream/arm" alt="Arm Camera" onerror="this.style.opacity=0.3; this.parentElement.innerHTML='<div style=\\'color:#666;padding:40px;text-align:center\\'>Arm Camera Offline</div>'">
            </div>
        </div>
        
        <div class="panel camera-panel">
            <div class="panel-header">
                <span class="panel-title">👁️ Right Eye</span>
                <span class="panel-badge">3D Cam</span>
            </div>
            <div class="panel-content">
                <img src="/stream/right" alt="Right Eye" onerror="this.style.opacity=0.3">
            </div>
        </div>
        
        <div class="panel lidar-panel">
            <div class="panel-header">
                <span class="panel-title">🔴 LiDAR Scan</span>
                <span class="panel-badge" id="lidarBadge">Connecting...</span>
            </div>
            <div class="lidar-container">
                <div class="lidar-viz">
                    <canvas id="lidarCanvas" width="500" height="500"></canvas>
                </div>
                <div class="lidar-stats">
                    <div class="stat-card">
                        <div class="stat-label">Points</div>
                        <div class="stat-value" id="pointCount">0</div>
                    </div>
                    <div class="stat-card">
                        <div class="stat-label">Min Distance</div>
                        <div class="stat-value accent" id="minDist">--<span class="stat-unit">mm</span></div>
                    </div>
                    <div class="stat-card">
                        <div class="stat-label">Max Distance</div>
                        <div class="stat-value" id="maxDist">--<span class="stat-unit">mm</span></div>
                    </div>
                    <div class="stat-card">
                        <div class="stat-label">Scan Rate</div>
                        <div class="stat-value" id="scanRate">--<span class="stat-unit">Hz</span></div>
                    </div>
                    <div class="stat-card">
                        <div class="stat-label">Port</div>
                        <div class="stat-value" id="lidarPort" style="font-size: 0.75rem;">--</div>
                    </div>
                </div>
            </div>
        </div>
    </main>
    
    <script>
        const canvas = document.getElementById('lidarCanvas');
        const ctx = canvas.getContext('2d');
        const centerX = canvas.width / 2;
        const centerY = canvas.height / 2;
        const maxRadius = Math.min(centerX, centerY) - 20;
        
        // Scale: pixels per mm (adjust based on expected max range)
        const MAX_RANGE_MM = 5000;  // 5 meters - adjust for your environment
        const scale = maxRadius / MAX_RANGE_MM;
        
        let lastScanTime = 0;
        let scanTimes = [];
        
        function drawLidarBackground() {
            ctx.fillStyle = '#050508';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            
            // Draw range rings
            ctx.strokeStyle = '#1c1c24';
            ctx.lineWidth = 1;
            
            for (let r = 1000; r <= MAX_RANGE_MM; r += 1000) {
                ctx.beginPath();
                ctx.arc(centerX, centerY, r * scale, 0, Math.PI * 2);
                ctx.stroke();
                
                // Range label
                ctx.fillStyle = '#3a3a44';
                ctx.font = '10px IBM Plex Mono';
                ctx.fillText(`${r/1000}m`, centerX + 5, centerY - r * scale + 12);
            }
            
            // Draw crosshairs
            ctx.strokeStyle = '#1c1c24';
            ctx.beginPath();
            ctx.moveTo(centerX, 0);
            ctx.lineTo(centerX, canvas.height);
            ctx.moveTo(0, centerY);
            ctx.lineTo(canvas.width, centerY);
            ctx.stroke();
            
            // Draw cardinal directions
            ctx.fillStyle = '#6e6e73';
            ctx.font = '12px IBM Plex Mono';
            ctx.textAlign = 'center';
            ctx.fillText('0°', centerX, 15);
            ctx.fillText('180°', centerX, canvas.height - 5);
            ctx.fillText('90°', canvas.width - 15, centerY + 4);
            ctx.fillText('270°', 20, centerY + 4);
            
            // Center dot (robot position)
            ctx.fillStyle = '#ff2d55';
            ctx.beginPath();
            ctx.arc(centerX, centerY, 4, 0, Math.PI * 2);
            ctx.fill();
        }
        
        function drawLidarPoints(scan) {
            // Draw points with gradient based on distance
            scan.forEach(point => {
                const angle = point[0];
                const dist = point[1];
                
                // Convert polar to cartesian (angle 0 = forward = up)
                const angleRad = (angle - 90) * Math.PI / 180;
                const x = centerX + Math.cos(angleRad) * dist * scale;
                const y = centerY + Math.sin(angleRad) * dist * scale;
                
                // Color based on distance (close = red, far = cyan)
                const t = Math.min(dist / MAX_RANGE_MM, 1);
                const r = Math.floor(255 * (1 - t) + 0 * t);
                const g = Math.floor(45 * (1 - t) + 212 * t);
                const b = Math.floor(85 * (1 - t) + 255 * t);
                
                ctx.fillStyle = `rgb(${r}, ${g}, ${b})`;
                ctx.beginPath();
                ctx.arc(x, y, 2, 0, Math.PI * 2);
                ctx.fill();
            });
        }
        
        function updateLidar() {
            fetch('/lidar/data')
                .then(res => res.json())
                .then(data => {
                    // Update status indicators
                    const lidarStatusDot = document.getElementById('lidarStatus');
                    const lidarBadge = document.getElementById('lidarBadge');
                    
                    if (data.connected) {
                        lidarStatusDot.classList.add('online');
                        lidarBadge.textContent = `${data.point_count} pts`;
                    } else {
                        lidarStatusDot.classList.remove('online');
                        lidarBadge.textContent = data.error || 'Disconnected';
                    }
                    
                    // Update stats
                    document.getElementById('pointCount').textContent = data.point_count;
                    document.getElementById('lidarPort').textContent = data.port || '--';
                    
                    if (data.scan && data.scan.length > 0) {
                        const distances = data.scan.map(p => p[1]);
                        const minD = Math.min(...distances);
                        const maxD = Math.max(...distances);
                        document.getElementById('minDist').innerHTML = `${Math.round(minD)}<span class="stat-unit">mm</span>`;
                        document.getElementById('maxDist').innerHTML = `${Math.round(maxD)}<span class="stat-unit">mm</span>`;
                        
                        // Calculate scan rate
                        const now = Date.now();
                        if (lastScanTime > 0) {
                            scanTimes.push(now - lastScanTime);
                            if (scanTimes.length > 10) scanTimes.shift();
                            const avgMs = scanTimes.reduce((a, b) => a + b, 0) / scanTimes.length;
                            const hz = Math.round(1000 / avgMs * 10) / 10;
                            document.getElementById('scanRate').innerHTML = `${hz}<span class="stat-unit">Hz</span>`;
                        }
                        lastScanTime = now;
                        
                        // Draw
                        drawLidarBackground();
                        drawLidarPoints(data.scan);
                    } else {
                        drawLidarBackground();
                    }
                })
                .catch(err => {
                    console.error('LiDAR fetch error:', err);
                    drawLidarBackground();
                });
        }
        
        function updateCameraStatus() {
            fetch('/status')
                .then(res => res.json())
                .then(data => {
                    const camDot = document.getElementById('cameraStatus');
                    if (data.camera && data.camera.connected) {
                        camDot.classList.add('online');
                    } else {
                        camDot.classList.remove('online');
                    }
                })
                .catch(() => {});
        }
        
        // Initial draw
        drawLidarBackground();
        
        // Update loops
        setInterval(updateLidar, 100);  // 10Hz LiDAR update
        setInterval(updateCameraStatus, 5000);  // Camera status every 5s
        updateCameraStatus();
    </script>
</body>
</html>
'''


# =============================================================================
# Main Entry Point
# =============================================================================

if __name__ == '__main__':
    import atexit
    
    print("=" * 60)
    print("  目 ME Node - Vision & LiDAR Server")
    print("  VOIGHT CLUSTER")
    print("=" * 60)
    print("  EYES (3D Stereo Camera):")
    print("    Stereo:     http://0.0.0.0:8028/stream")
    print("    Left Eye:   http://0.0.0.0:8028/stream/left")
    print("    Right Eye:  http://0.0.0.0:8028/stream/right")
    print("  ARM POV (Razer Camera):")
    print("    Stream:     http://0.0.0.0:8028/stream/arm")
    print("    Snapshot:   http://0.0.0.0:8028/snapshot/arm")
    print("  LiDAR:")
    print("    Data:       http://0.0.0.0:8028/lidar/data")
    print("    Status:     http://0.0.0.0:8028/lidar/status")
    print("  Dashboard:    http://0.0.0.0:8028/")
    print("=" * 60)
    
    # Start background threads
    camera_thread = threading.Thread(target=camera_capture_loop, daemon=True)
    camera_thread.start()
    
    arm_thread = threading.Thread(target=arm_camera_capture_loop, daemon=True)
    arm_thread.start()
    
    lidar_thread = threading.Thread(target=lidar_scan_loop, daemon=True)
    lidar_thread.start()
    
    # Cleanup on exit
    atexit.register(stop_lidar)
    
    # Run server
    app.run(host='0.0.0.0', port=8028, threaded=True)

