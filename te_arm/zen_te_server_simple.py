#!/usr/bin/env python3
"""
Zen TE Node Server - Simplified Control (手)
=============================================
Simple control for BASE servo and DC motor on MOTOR2 (M2).

Hardware:
- BASE servo: PWM servo on Arduino (channel 0)
- M2 DC motor: DC motor on the OSEPP TB6612 shield MOTOR2 terminals

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
from pydantic import BaseModel

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False

app = FastAPI(
    title="Zen TE Node - Simple",
    description="BASE servo + Stepper M2 control (手)",
    version="0.3.0",
)

# Serial config
SERIAL_BAUD = 115200

# Current state
arduino_connection = None
connected_port: Optional[str] = None

# BASE servo state (0-180 degrees)
base_angle = 90

# DC Motor M2 state (last pulse)
m2_last: dict = {"dir": 0, "pwm": 0, "ms": 0}

# Debug / diagnostics
last_arduino_response: Optional[dict] = None
last_arduino_error: Optional[str] = None
last_arduino_ts: Optional[float] = None

# Movement settings
BASE_STEP = 5        # degrees per click
# Motor pulse settings (safe, short)
M2_PWM = 120
M2_MS = 120


# =============================================================================
# Serial Communication
# =============================================================================

def find_arduino_ports() -> list[str]:
    """Find available serial ports."""
    patterns = ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/tty.usbserial*", "/dev/tty.usbmodem*"]
    ports = []
    for pat in patterns:
        ports.extend(glob.glob(pat))
    return sorted(set(ports))


def _port_sort_key(p: str) -> tuple[int, str]:
    """
    Prefer the newest ttyUSB/ttyACM number (Linux tends to bump numbers on replug),
    otherwise fall back to lexicographic.
    """
    m = re.search(r"(ttyUSB|ttyACM)(\d+)$", p)
    if not m:
        return (-1, p)
    return (int(m.group(2)), p)


def choose_default_port(ports: list[str]) -> Optional[str]:
    """Choose best default port from discovered ports."""
    if not ports:
        return None
    # Prefer highest index for ttyUSB/ttyACM to handle replug (ttyUSB1 -> ttyUSB2)
    return sorted(ports, key=_port_sort_key, reverse=True)[0]


def is_connected() -> bool:
    """Check if Arduino is connected."""
    try:
        # If the port disappeared (replug), consider disconnected.
        if connected_port and not os.path.exists(connected_port):
            return False
        return arduino_connection is not None and arduino_connection.is_open
    except Exception:
        return False


def ensure_connection_ok() -> None:
    """If we have a stale connection (port gone), tear it down so /connect can recover."""
    global arduino_connection, connected_port
    if connected_port and not os.path.exists(connected_port):
        try:
            if arduino_connection:
                arduino_connection.close()
        except Exception:
            pass
        arduino_connection = None
        connected_port = None


def send_command(cmd: dict) -> Optional[dict]:
    """Send JSON command to Arduino and read response."""
    global arduino_connection, last_arduino_response, last_arduino_error, last_arduino_ts
    if not is_connected():
        last_arduino_error = "not connected"
        last_arduino_ts = time.time()
        return None
    
    try:
        line = json.dumps(cmd, separators=(",", ":")) + "\n"
        arduino_connection.write(line.encode("utf-8"))
        arduino_connection.flush()
        
        # Read response
        time.sleep(0.05)
        raw = arduino_connection.readline().decode("utf-8", errors="ignore").strip()
        if raw.startswith("{"):
            resp = json.loads(raw)
            last_arduino_response = resp
            last_arduino_error = None
            last_arduino_ts = time.time()
            return resp
        # No JSON response is still useful to record
        last_arduino_response = None
        last_arduino_error = f"no json response (raw={raw[:120]!r})" if raw else "no response"
        last_arduino_ts = time.time()
    except Exception as e:
        last_arduino_response = None
        last_arduino_error = f"serial error: {e}"
        last_arduino_ts = time.time()
        print(f"Serial error: {e}")
    
    return None


# =============================================================================
# Pydantic Models
# =============================================================================

class MoveCommand(BaseModel):
    direction: str  # "left" or "right"
    steps: int = 1  # number of step increments


class StepperCommand(BaseModel):
    direction: str  # "left" or "right" (or "cw" / "ccw")
    steps: int = 10  # number of motor steps


# =============================================================================
# API Endpoints
# =============================================================================

@app.get("/")
def root():
    """Root endpoint."""
    return {
        "node": "te",
        "kanji": "手",
        "mode": "simple",
        "endpoints": ["/status", "/dashboard", "/base/left", "/base/right", "/stepper/left", "/stepper/right"],
    }


@app.get("/status")
def status():
    """Node status."""
    ensure_connection_ok()
    ports = find_arduino_ports()
    mem = psutil.virtual_memory()
    
    return {
        "node": "te",
        "kanji": "手",
        "role": "limbs",
        "mode": "simple",
        "connected": is_connected(),
        "port": connected_port,
        "base_angle": base_angle,
        "m2_last": m2_last,
        "arduino": {
            "last_response": last_arduino_response,
            "last_error": last_arduino_error,
            "last_ts": last_arduino_ts,
        },
        "hardware": {
            "cpu_percent": psutil.cpu_percent(interval=None),
            "ram_used_gb": round(mem.used / (1024**3), 1),
            "ram_total_gb": round(mem.total / (1024**3), 1),
            "serial_ports": ports,
        },
        "version": "0.3.0",
    }


@app.post("/connect")
def connect(port: str = None):
    """Connect to Arduino."""
    global arduino_connection, connected_port
    
    if not SERIAL_AVAILABLE:
        raise HTTPException(status_code=500, detail="pyserial not installed")
    
    ports = find_arduino_ports()
    if port is None:
        if not ports:
            raise HTTPException(status_code=404, detail="No Arduino found")
        port = choose_default_port(ports)
        if port is None:
            raise HTTPException(status_code=404, detail="No Arduino found")
    
    try:
        if arduino_connection:
            arduino_connection.close()
        
        arduino_connection = serial.Serial(port, SERIAL_BAUD, timeout=1)
        connected_port = port
        time.sleep(2.0)  # Wait for Arduino reset
        arduino_connection.reset_input_buffer()
        # Prime a status request so we know comms are alive
        send_command({"cmd": "status"})
        
        return {"success": True, "port": port}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/disconnect")
def disconnect():
    """Disconnect from Arduino."""
    global arduino_connection, connected_port
    
    if arduino_connection:
        arduino_connection.close()
        arduino_connection = None
        connected_port = None
    
    return {"success": True}


# -----------------------------------------------------------------------------
# BASE Servo Control
# -----------------------------------------------------------------------------

@app.post("/base/left")
def base_left(steps: int = 1):
    """Move BASE servo left (decrease angle)."""
    global base_angle
    
    new_angle = max(0, base_angle - (BASE_STEP * steps))
    base_angle = new_angle
    
    if is_connected():
        # New firmware command
        send_command({"cmd": "base", "angle": base_angle})
    
    return {"success": True, "base_angle": base_angle, "direction": "left"}


@app.post("/base/right")
def base_right(steps: int = 1):
    """Move BASE servo right (increase angle)."""
    global base_angle
    
    new_angle = min(180, base_angle + (BASE_STEP * steps))
    base_angle = new_angle
    
    if is_connected():
        send_command({"cmd": "base", "angle": base_angle})
    
    return {"success": True, "base_angle": base_angle, "direction": "right"}


@app.post("/base/set")
def base_set(angle: int):
    """Set BASE servo to specific angle."""
    global base_angle
    
    base_angle = max(0, min(180, angle))
    
    if is_connected():
        send_command({"cmd": "base", "angle": base_angle})
    
    return {"success": True, "base_angle": base_angle}


@app.post("/base/center")
def base_center():
    """Center BASE servo (90 degrees)."""
    global base_angle
    base_angle = 90
    
    if is_connected():
        send_command({"cmd": "base", "angle": 90})
    
    return {"success": True, "base_angle": base_angle}


@app.post("/base/sweep")
def base_sweep():
    """
    Debug: do an obvious sweep so you can visually confirm motion.
    0 -> 180 -> 90
    """
    global base_angle
    if is_connected():
        send_command({"cmd": "base", "angle": 0})
        time.sleep(1.2)
        send_command({"cmd": "base", "angle": 180})
        time.sleep(1.2)
        send_command({"cmd": "base", "angle": 90})
        base_angle = 90
        return {"success": True, "base_angle": base_angle, "note": "sweep complete"}
    return {"success": False, "base_angle": base_angle, "note": "not connected"}


# -----------------------------------------------------------------------------
# Stepper Motor M2 Control
# -----------------------------------------------------------------------------

@app.post("/stepper/left")
def stepper_left(steps: int = None):
    """Run DC motor on M2 left (dir=-1) for a short pulse."""
    global m2_last
    if is_connected():
        send_command({"cmd": "m2", "dir": -1, "pwm": M2_PWM, "ms": M2_MS})
    m2_last = {"dir": -1, "pwm": M2_PWM, "ms": M2_MS}
    return {"success": True, "direction": "left", "pwm": M2_PWM, "ms": M2_MS}


@app.post("/stepper/right")
def stepper_right(steps: int = None):
    """Run DC motor on M2 right (dir=+1) for a short pulse."""
    global m2_last
    if is_connected():
        send_command({"cmd": "m2", "dir": 1, "pwm": M2_PWM, "ms": M2_MS})
    m2_last = {"dir": 1, "pwm": M2_PWM, "ms": M2_MS}
    return {"success": True, "direction": "right", "pwm": M2_PWM, "ms": M2_MS}


@app.post("/stepper/stop")
def stepper_stop():
    """Stop DC motor M2."""
    if is_connected():
        send_command({"cmd": "m2_stop"})
    return {"success": True, "message": "M2 stopped"}


# -------------------------------------------------------------------------
# Diagnostic: MOTOR1 (in case your DC motor is wired to MOTOR1 terminals)
# -------------------------------------------------------------------------

@app.post("/m1/left")
def m1_left():
    """Run DC motor on MOTOR1 left (dir=-1) for a short pulse."""
    if is_connected():
        send_command({"cmd": "m1", "dir": -1, "pwm": M2_PWM, "ms": M2_MS})
    return {"success": True, "direction": "left", "pwm": M2_PWM, "ms": M2_MS}


@app.post("/m1/right")
def m1_right():
    """Run DC motor on MOTOR1 right (dir=+1) for a short pulse."""
    if is_connected():
        send_command({"cmd": "m1", "dir": 1, "pwm": M2_PWM, "ms": M2_MS})
    return {"success": True, "direction": "right", "pwm": M2_PWM, "ms": M2_MS}


@app.post("/m1/stop")
def m1_stop():
    """Stop DC motor on MOTOR1."""
    if is_connected():
        send_command({"cmd": "m1_stop"})
    return {"success": True, "message": "M1 stopped"}


# =============================================================================
# Dashboard
# =============================================================================

@app.get("/dashboard", response_class=HTMLResponse)
def dashboard():
    """Simple control dashboard."""
    
    html = f"""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>TE Control - 手</title>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;600;700&display=swap');
        
        :root {{
            --bg: #0d0d12;
            --card: #16161d;
            --accent: #ff3366;
            --accent2: #00ccff;
            --text: #f0f0f5;
            --dim: #5a5a6e;
            --success: #00dd77;
            --border: #2a2a35;
        }}
        
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        
        body {{
            font-family: 'Space Grotesk', sans-serif;
            background: var(--bg);
            color: var(--text);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 2rem;
        }}
        
        header {{
            text-align: center;
            margin-bottom: 2rem;
        }}
        
        .kanji {{
            font-size: 5rem;
            color: var(--accent);
            text-shadow: 0 0 40px var(--accent);
            line-height: 1;
        }}
        
        h1 {{
            font-size: 1.2rem;
            font-weight: 400;
            color: var(--dim);
            letter-spacing: 0.3em;
            margin-top: 0.5rem;
        }}
        
        .status {{
            display: flex;
            gap: 1rem;
            margin-bottom: 2rem;
            font-size: 0.85rem;
        }}
        
        .status-item {{
            display: flex;
            align-items: center;
            gap: 0.5rem;
            padding: 0.5rem 1rem;
            background: var(--card);
            border-radius: 20px;
            border: 1px solid var(--border);
        }}
        
        .dot {{
            width: 10px;
            height: 10px;
            border-radius: 50%;
            background: var(--dim);
        }}
        
        .dot.online {{
            background: var(--success);
            box-shadow: 0 0 10px var(--success);
        }}
        
        .controls {{
            display: flex;
            flex-direction: column;
            gap: 2rem;
            width: 100%;
            max-width: 500px;
        }}
        
        .control-group {{
            background: var(--card);
            border: 1px solid var(--border);
            border-radius: 16px;
            padding: 1.5rem;
        }}
        
        .control-header {{
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 1.5rem;
        }}
        
        .control-title {{
            font-size: 1rem;
            font-weight: 600;
            color: var(--accent2);
            letter-spacing: 0.1em;
        }}
        
        .control-value {{
            font-size: 1.5rem;
            font-weight: 700;
            color: var(--text);
        }}
        
        .button-row {{
            display: flex;
            gap: 1rem;
        }}
        
        .btn {{
            flex: 1;
            padding: 1.2rem 1rem;
            border: 2px solid var(--border);
            border-radius: 12px;
            background: transparent;
            color: var(--text);
            font-family: inherit;
            font-size: 1.5rem;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.15s ease;
            user-select: none;
        }}
        
        .btn:hover {{
            background: var(--accent);
            border-color: var(--accent);
            transform: scale(1.02);
        }}
        
        .btn:active {{
            transform: scale(0.98);
        }}
        
        .btn.center {{
            flex: 0.5;
            font-size: 0.9rem;
            padding: 1rem;
        }}
        
        .btn.center:hover {{
            background: var(--accent2);
            border-color: var(--accent2);
        }}
        
        .btn.connect {{
            background: var(--success);
            border-color: var(--success);
            color: #000;
        }}
        
        .connection-bar {{
            display: flex;
            gap: 0.5rem;
            margin-top: 2rem;
        }}
        
        .connection-bar .btn {{
            font-size: 0.9rem;
            padding: 0.8rem;
        }}
        
        #log {{
            margin-top: 2rem;
            padding: 1rem;
            background: var(--card);
            border: 1px solid var(--border);
            border-radius: 8px;
            font-family: monospace;
            font-size: 0.8rem;
            color: var(--dim);
            max-height: 100px;
            overflow-y: auto;
            width: 100%;
            max-width: 500px;
        }}
    </style>
</head>
<body>
    <header>
        <div class="kanji">手</div>
        <h1>TE CONTROL</h1>
    </header>
    
    <div class="status">
        <div class="status-item">
            <div class="dot" id="connDot"></div>
            <span id="connStatus">Checking...</span>
        </div>
        <div class="status-item">
            <span id="portStatus">-</span>
        </div>
    </div>
    
    <div class="controls">
        <!-- BASE Servo -->
        <div class="control-group">
            <div class="control-header">
                <span class="control-title">BASE SERVO</span>
                <span class="control-value" id="baseValue">{base_angle}°</span>
            </div>
            <div class="button-row">
                <button class="btn" onclick="baseLeft()">◀ LEFT</button>
                <button class="btn center" onclick="baseCenter()">⟲</button>
                <button class="btn" onclick="baseRight()">RIGHT ▶</button>
            </div>
            <div class="connection-bar" style="margin-top: 1rem;">
                <button class="btn" onclick="baseSweep()">TEST SWEEP</button>
            </div>
        </div>
        
        <!-- DC Motor on M2 -->
        <div class="control-group">
            <div class="control-header">
                <span class="control-title">M2 DC MOTOR</span>
                <span class="control-value" id="stepperValue">PULSE</span>
            </div>
            <div class="button-row">
                <button class="btn" onclick="stepperLeft()">◀ LEFT</button>
                <button class="btn center" onclick="stepperStop()">⏹</button>
                <button class="btn" onclick="stepperRight()">RIGHT ▶</button>
            </div>
        </div>
        
        <!-- Connection -->
        <div class="connection-bar">
            <button class="btn connect" onclick="connect()">CONNECT</button>
            <button class="btn" onclick="disconnect()">DISCONNECT</button>
        </div>
    </div>
    
    <div id="log">Ready</div>
    
    <script>
        const baseValue = document.getElementById('baseValue');
        const stepperValue = document.getElementById('stepperValue');
        const connDot = document.getElementById('connDot');
        const connStatus = document.getElementById('connStatus');
        const portStatus = document.getElementById('portStatus');
        const log = document.getElementById('log');
        
        function logMsg(msg) {{
            const time = new Date().toLocaleTimeString();
            log.textContent = `[${{time}}] ${{msg}}`;
        }}
        
        async function api(endpoint, method = 'POST') {{
            try {{
                const res = await fetch(endpoint, {{ method }});
                const data = await res.json();
                return data;
            }} catch (e) {{
                logMsg('Error: ' + e.message);
                return null;
            }}
        }}
        
        async function updateStatus() {{
            const data = await api('/status', 'GET');
            if (data) {{
                baseValue.textContent = data.base_angle + '°';
                // For DC motor we just show last pulse direction
                const m2 = data.m2_last || {{}};
                if (m2.dir === 1) stepperValue.textContent = 'RIGHT';
                else if (m2.dir === -1) stepperValue.textContent = 'LEFT';
                else stepperValue.textContent = 'PULSE';
                
                if (data.connected) {{
                    connDot.classList.add('online');
                    connStatus.textContent = 'Connected';
                    portStatus.textContent = data.port || '-';
                }} else {{
                    connDot.classList.remove('online');
                    connStatus.textContent = 'Disconnected';
                    portStatus.textContent = '-';
                }}
            }}
        }}
        
        async function baseLeft() {{
            const data = await api('/base/left');
            if (data) {{
                baseValue.textContent = data.base_angle + '°';
                logMsg('Base → ' + data.base_angle + '°');
            }}
        }}
        
        async function baseRight() {{
            const data = await api('/base/right');
            if (data) {{
                baseValue.textContent = data.base_angle + '°';
                logMsg('Base → ' + data.base_angle + '°');
            }}
        }}
        
        async function baseCenter() {{
            const data = await api('/base/center');
            if (data) {{
                baseValue.textContent = data.base_angle + '°';
                logMsg('Base centered');
            }}
        }}

        async function baseSweep() {{
            logMsg('Sweeping base...');
            const data = await api('/base/sweep');
            if (data) {{
                baseValue.textContent = data.base_angle + '°';
                logMsg(data.note || 'Sweep done');
            }}
        }}
        
        async function stepperLeft() {{
            const data = await api('/stepper/left');
            if (data) {{
                stepperValue.textContent = 'LEFT';
                logMsg('M2 ← pulse (' + data.pwm + ' pwm, ' + data.ms + ' ms)');
            }}
        }}
        
        async function stepperRight() {{
            const data = await api('/stepper/right');
            if (data) {{
                stepperValue.textContent = 'RIGHT';
                logMsg('M2 → pulse (' + data.pwm + ' pwm, ' + data.ms + ' ms)');
            }}
        }}
        
        async function stepperStop() {{
            await api('/stepper/stop');
            stepperValue.textContent = 'PULSE';
            logMsg('M2 stopped');
        }}
        
        async function connect() {{
            const data = await api('/connect');
            if (data && data.success) {{
                logMsg('Connected to ' + data.port);
                updateStatus();
            }}
        }}
        
        async function disconnect() {{
            await api('/disconnect');
            logMsg('Disconnected');
            updateStatus();
        }}
        
        // Initial status
        updateStatus();
        
        // Poll status every 5 seconds
        setInterval(updateStatus, 5000);
    </script>
</body>
</html>
"""
    return HTMLResponse(content=html)


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    import uvicorn
    
    print("=" * 50)
    print("  TE Node - Simple Control (手)")
    print("  http://0.0.0.0:8027")
    print("=" * 50)
    print("  Dashboard: http://te.local:8027/dashboard")
    print("=" * 50)
    
    uvicorn.run(app, host="0.0.0.0", port=8027)

