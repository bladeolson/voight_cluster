#!/usr/bin/env python3
"""
Zen TE Node Server - 6 Servo Arm Control (手)
==============================================
Controls 6 servos on A0-A5 + DC motor on M3

Hardware:
- A0: Base       A1: Shoulder    A2: Elbow
- A3: Wrist Bend A4: Wrist Swivel A5: Pincher
- M3: DC Motor (D6/D4/D9)

Port: 8027
"""

import glob
import json
import os
import re
import time
from typing import Optional

import psutil
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False

app = FastAPI(title="Zen TE Arm", description="6-Servo Arm Control (手)", version="0.4.0")

SERIAL_BAUD = 115200
STEP_DEG = 10
M3_PWM = 120
M3_MS = 150

# Connection state
arduino_connection = None
connected_port: Optional[str] = None
last_response: Optional[dict] = None
last_error: Optional[str] = None

# Servo state (6 arm + 1 head)
SERVO_NAMES = ["base", "shoulder", "elbow", "wrist_bend", "wrist_swivel", "pincher", "head_pan"]
servo_angles = {name: 90 for name in SERVO_NAMES}
m3_last = {"dir": 0, "pwm": 0, "ms": 0}


# =============================================================================
# Serial Communication
# =============================================================================

def find_ports():
    ports = []
    for pat in ["/dev/ttyUSB*", "/dev/ttyACM*"]:
        ports.extend(glob.glob(pat))
    return sorted(set(ports))


def is_connected():
    try:
        if connected_port and not os.path.exists(connected_port):
            return False
        return arduino_connection is not None and arduino_connection.is_open
    except:
        return False


def send_command(cmd: dict) -> Optional[dict]:
    global last_response, last_error
    if not is_connected():
        last_error = "not connected"
        return None
    try:
        line = json.dumps(cmd, separators=(",", ":")) + "\n"
        arduino_connection.write(line.encode())
        arduino_connection.flush()
        time.sleep(0.05)
        raw = arduino_connection.readline().decode(errors="ignore").strip()
        if raw.startswith("{"):
            resp = json.loads(raw)
            last_response = resp
            last_error = None
            return resp
        last_error = f"no json: {raw[:80]}" if raw else "no response"
    except Exception as e:
        last_error = str(e)
    return None


# =============================================================================
# API Endpoints
# =============================================================================

@app.get("/")
def root():
    return {"node": "te", "kanji": "手", "servos": SERVO_NAMES, "version": "0.4.0"}


@app.get("/status")
def status():
    return {
        "node": "te",
        "connected": is_connected(),
        "port": connected_port,
        "servos": servo_angles,
        "m3_last": m3_last,
        "last_response": last_response,
        "last_error": last_error,
        "hardware": {
            "cpu_percent": psutil.cpu_percent(interval=None),
            "ram_gb": round(psutil.virtual_memory().used / (1024**3), 1),
            "serial_ports": find_ports(),
        },
    }


@app.post("/connect")
def connect(port: str = None):
    global arduino_connection, connected_port
    if not SERIAL_AVAILABLE:
        raise HTTPException(500, "pyserial not installed")
    
    ports = find_ports()
    if port is None:
        if not ports:
            raise HTTPException(404, "No Arduino found")
        port = sorted(ports, key=lambda p: int(re.search(r"(\d+)$", p).group(1)) if re.search(r"(\d+)$", p) else 0, reverse=True)[0]
    
    try:
        if arduino_connection:
            arduino_connection.close()
        arduino_connection = serial.Serial(port, SERIAL_BAUD, timeout=1)
        connected_port = port
        time.sleep(2.0)
        arduino_connection.reset_input_buffer()
        send_command({"cmd": "status"})
        return {"success": True, "port": port}
    except Exception as e:
        raise HTTPException(500, str(e))


@app.post("/disconnect")
def disconnect():
    global arduino_connection, connected_port
    if arduino_connection:
        arduino_connection.close()
        arduino_connection = None
        connected_port = None
    return {"success": True}


# =============================================================================
# Servo Control - Generic endpoints for all 6 servos
# =============================================================================

def _move_servo(name: str, angle: int):
    if name not in SERVO_NAMES:
        raise HTTPException(400, f"Unknown servo: {name}")
    angle = max(0, min(180, angle))
    servo_angles[name] = angle
    if is_connected():
        send_command({"cmd": name, "angle": angle})
    return {"success": True, "servo": name, "angle": angle}


@app.post("/servo/{name}/set")
def servo_set(name: str, angle: int):
    return _move_servo(name, angle)


@app.post("/servo/{name}/left")
def servo_left(name: str, steps: int = 1):
    if name not in SERVO_NAMES:
        raise HTTPException(400, f"Unknown servo: {name}")
    return _move_servo(name, servo_angles[name] - (STEP_DEG * steps))


@app.post("/servo/{name}/right")
def servo_right(name: str, steps: int = 1):
    if name not in SERVO_NAMES:
        raise HTTPException(400, f"Unknown servo: {name}")
    return _move_servo(name, servo_angles[name] + (STEP_DEG * steps))


@app.post("/servo/{name}/center")
def servo_center(name: str):
    return _move_servo(name, 90)


@app.post("/home")
def home_all():
    if is_connected():
        send_command({"cmd": "home"})
    for name in SERVO_NAMES:
        servo_angles[name] = 90
    return {"success": True, "servos": servo_angles}


# =============================================================================
# Motor Control
# =============================================================================

@app.post("/motor/left")
def motor_left():
    global m3_last
    if is_connected():
        send_command({"cmd": "m2", "dir": -1, "pwm": M3_PWM, "ms": M3_MS})
    m3_last = {"dir": -1, "pwm": M3_PWM, "ms": M3_MS}
    return {"success": True, "direction": "left"}


@app.post("/motor/right")
def motor_right():
    global m3_last
    if is_connected():
        send_command({"cmd": "m2", "dir": 1, "pwm": M3_PWM, "ms": M3_MS})
    m3_last = {"dir": 1, "pwm": M3_PWM, "ms": M3_MS}
    return {"success": True, "direction": "right"}


@app.post("/motor/stop")
def motor_stop():
    if is_connected():
        send_command({"cmd": "m2_stop"})
    return {"success": True}


# =============================================================================
# Dashboard
# =============================================================================

@app.get("/dashboard", response_class=HTMLResponse)
def dashboard():
    return HTMLResponse(content="""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>TE Arm - 手</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;700&display=swap');
        :root { --bg:#0a0a0f; --card:#12121a; --accent:#ff2d55; --accent2:#5ac8fa; --go:#30d158; --text:#f5f5f7; --dim:#48484a; --border:#1c1c1e; }
        * { margin:0; padding:0; box-sizing:border-box; }
        body { font-family:'JetBrains Mono',monospace; background:linear-gradient(135deg,var(--bg),#0f0f18); color:var(--text); min-height:100vh; padding:1rem; }
        header { text-align:center; margin-bottom:1rem; }
        .kanji { font-size:3rem; color:var(--accent); text-shadow:0 0 40px var(--accent); }
        h1 { font-size:0.8rem; color:var(--dim); letter-spacing:0.3em; margin-top:0.3rem; }
        .status { display:flex; justify-content:center; gap:0.5rem; margin-bottom:1rem; font-size:0.7rem; }
        .status-item { display:flex; align-items:center; gap:0.3rem; padding:0.3rem 0.6rem; background:var(--card); border-radius:12px; border:1px solid var(--border); }
        .dot { width:8px; height:8px; border-radius:50%; background:var(--dim); }
        .dot.online { background:var(--go); box-shadow:0 0 10px var(--go); }
        .grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(150px,1fr)); gap:0.7rem; max-width:700px; margin:0 auto; }
        .card { background:var(--card); border:1px solid var(--border); border-radius:12px; padding:0.7rem; }
        .card-name { font-size:0.65rem; color:var(--accent2); letter-spacing:0.1em; margin-bottom:0.2rem; text-transform:uppercase; }
        .card-angle { font-size:1.4rem; font-weight:700; margin-bottom:0.4rem; }
        .card-queue { font-size:0.75rem; color:var(--dim); margin-bottom:0.4rem; }
        .card-queue.active { color:var(--go); font-weight:700; }
        .btns { display:flex; gap:0.3rem; }
        .btn { flex:1; padding:0.5rem; border:1px solid var(--border); border-radius:8px; background:transparent; color:var(--text); font-family:inherit; font-size:0.85rem; cursor:pointer; transition:all 0.1s; }
        .btn:hover { background:rgba(255,45,85,0.2); border-color:var(--accent); }
        .btn:active { transform:scale(0.95); }
        .btn.go { background:var(--go); border-color:var(--go); color:#000; font-weight:700; }
        .btn.go:disabled { background:var(--dim); border-color:var(--dim); }
        .conn-bar { display:flex; gap:0.5rem; max-width:700px; margin:1rem auto 0; }
        .conn-bar .btn { padding:0.5rem; font-size:0.7rem; }
        .btn.connect { background:var(--go); border-color:var(--go); color:#000; }
        #log { max-width:700px; margin:0.7rem auto; padding:0.5rem; background:var(--card); border-radius:8px; font-size:0.6rem; color:var(--dim); }
    </style>
</head>
<body>
    <header><div class="kanji">手</div><h1>TE ARM CONTROL</h1></header>
    <div class="status">
        <div class="status-item"><div class="dot" id="dot"></div><span id="connStatus">...</span></div>
        <div class="status-item"><span id="portStatus">-</span></div>
    </div>
    <div class="grid" id="grid"></div>
    <div class="conn-bar">
        <button class="btn connect" onclick="connect()">CONNECT</button>
        <button class="btn" onclick="disconnect()">DISCONNECT</button>
        <button class="btn" onclick="homeAll()">HOME ALL</button>
    </div>
    <div id="log">Ready</div>
<script>
const SERVOS = ['base','shoulder','elbow','wrist_bend','wrist_swivel','pincher','head_pan'];
const LABELS = ['BASE (A0)','SHOULDER (A1)','ELBOW (A2)','WRIST BEND (A3)','WRIST SWIVEL (A4)','PINCHER (A5)','HEAD PAN (D2)'];
const STEP = 10;
let angles = {}; SERVOS.forEach(s => angles[s] = 90);
let queues = {}; SERVOS.forEach(s => queues[s] = 0);

const log = m => document.getElementById('log').textContent = `[${new Date().toLocaleTimeString()}] ${m}`;

function render() {
    document.getElementById('grid').innerHTML = SERVOS.map((s,i) => `
        <div class="card">
            <div class="card-name">${LABELS[i]}</div>
            <div class="card-angle" id="a_${s}">${angles[s]}°</div>
            <div class="card-queue${queues[s]?' active':''}" id="q_${s}">${queues[s]>=0?'+':''}${queues[s]}°</div>
            <div class="btns">
                <button class="btn" onclick="queue('${s}',-1)">◀</button>
                <button class="btn go" id="g_${s}" onclick="go('${s}')"${queues[s]?'':' disabled'}>GO</button>
                <button class="btn" onclick="queue('${s}',1)">▶</button>
            </div>
        </div>`).join('');
}

function queue(s, d) {
    queues[s] += d * STEP;
    const t = angles[s] + queues[s];
    if (t < 0) queues[s] = -angles[s];
    if (t > 180) queues[s] = 180 - angles[s];
    render();
    log(`Queued ${s}: ${queues[s]>=0?'+':''}${queues[s]}°`);
}

async function go(s) {
    if (!queues[s]) return;
    const t = Math.max(0, Math.min(180, angles[s] + queues[s]));
    log(`Moving ${s} to ${t}°...`);
    const r = await fetch(`/servo/${s}/set?angle=${t}`, {method:'POST'}).then(r=>r.json()).catch(()=>null);
    if (r?.angle !== undefined) angles[s] = r.angle;
    queues[s] = 0;
    render();
    log(`${s} → ${angles[s]}°`);
}

async function connect() {
    log('Connecting...');
    const r = await fetch('/connect', {method:'POST'}).then(r=>r.json()).catch(()=>null);
    r?.success ? (log('Connected: '+r.port), updateStatus()) : log('Failed');
}

async function disconnect() {
    await fetch('/disconnect', {method:'POST'});
    log('Disconnected');
    updateStatus();
}

async function homeAll() {
    log('Homing...');
    await fetch('/home', {method:'POST'});
    SERVOS.forEach(s => { angles[s]=90; queues[s]=0; });
    render();
    log('All servos homed');
}

async function updateStatus() {
    const r = await fetch('/status').then(r=>r.json()).catch(()=>null);
    if (r) {
        if (r.servos) SERVOS.forEach(s => { if(r.servos[s]!==undefined) angles[s]=r.servos[s]; });
        SERVOS.forEach(s => { const e=document.getElementById('a_'+s); if(e) e.textContent=angles[s]+'°'; });
        const dot = document.getElementById('dot');
        document.getElementById('connStatus').textContent = r.connected ? 'Connected' : 'Disconnected';
        document.getElementById('portStatus').textContent = r.port || '-';
        r.connected ? dot.classList.add('online') : dot.classList.remove('online');
    }
}

render();
updateStatus();
setInterval(updateStatus, 5000);
</script>
</body>
</html>
""")


if __name__ == "__main__":
    import uvicorn
    print("=" * 50)
    print("  TE Arm - 6 Servo Control (手)")
    print("  http://0.0.0.0:8027/dashboard")
    print("=" * 50)
    uvicorn.run(app, host="0.0.0.0", port=8027)
