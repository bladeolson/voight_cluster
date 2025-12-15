#!/usr/bin/env python3
"""
Zen TE Node Server - Robot Arm Control (手)
============================================
FastAPI server for the TE node with robot arm control.
Integrates with the VOIGHT CLUSTER orchestration.

Node: TE (手 - hand/limbs)
Role: Robot arm manipulation
Port: 8027
"""

import asyncio
from contextlib import asynccontextmanager
from datetime import datetime
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field
from typing import Optional
import logging
import psutil
import os
import glob

from te_arm_bridge import TeArmBridge, ArmState


def get_hardware_stats():
    """Get current hardware statistics."""
    try:
        import subprocess
        # GPU detection
        gpus = []
        gpus_detected = 0
        try:
            result = subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total,memory.used', '--format=csv,noheader,nounits'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line:
                        parts = line.split(', ')
                        if len(parts) >= 3:
                            gpus.append({
                                'name': parts[0],
                                'memory_total_mb': int(parts[1]),
                                'memory_used_mb': int(parts[2])
                            })
                gpus_detected = len(gpus)
        except:
            pass
        
        return {
            "cpu_percent": psutil.cpu_percent(),
            "ram_total_gb": round(psutil.virtual_memory().total / (1024**3), 1),
            "ram_used_gb": round(psutil.virtual_memory().used / (1024**3), 1),
            "gpus_detected": gpus_detected,
            "gpus": gpus,
        }
    except:
        return {}

def get_lidar_status() -> dict:
    """
    Best-effort LiDAR detection on TE.
    We avoid hard-coding a model; instead we:
    - Prefer /dev/serial/by-id entries with lidar-ish keywords
    - Fall back to any ttyUSB/ttyACM port that is NOT the Arduino arm port
    """
    lidar_keywords = [
        "lidar", "rplidar", "ydlidar", "slamtec", "s2", "a2", "a1", "x4", "ld",
        "scan", "laser", "cp210", "silabs", "prolific", "pl2303"
    ]

    arduino_port = None
    try:
        # Prefer the actual connected port (if any), otherwise best-effort detect
        if arm_bridge is not None and getattr(arm_bridge, "serial", None) is not None:
            arduino_port = getattr(arm_bridge.serial, "port", None)
        if not arduino_port:
            arduino_port = TeArmBridge.find_arduino_port()
    except Exception:
        arduino_port = None

    # 1) Prefer stable by-id symlinks
    by_id_paths = sorted(glob.glob("/dev/serial/by-id/*"))
    by_id_matches: list[dict] = []
    for p in by_id_paths:
        name = os.path.basename(p).lower()
        if any(k in name for k in lidar_keywords):
            by_id_matches.append(
                {
                    "by_id": p,
                    "port": os.path.realpath(p),
                    "match": "keyword",
                    "name": os.path.basename(p),
                }
            )

    # Filter out Arduino port if it matches
    if arduino_port:
        by_id_matches = [m for m in by_id_matches if m.get("port") != arduino_port]

    if by_id_matches:
        # If multiple, return the first (most deterministic)
        m = by_id_matches[0]
        return {
            "connected": True,
            "port": m["port"],
            "by_id": m["by_id"],
            "note": f"matched by-id keyword: {m.get('name','')}",
        }

    # Special case: many RPLidar units enumerate as generic USB-serial (FTDI/CP210x/etc.)
    # If the arm is NOT connected, and we only see one serial device by-id, treat it as LiDAR.
    try:
        arm_connected = bool(arm_bridge and getattr(arm_bridge, "state", None) and arm_bridge.state.connected)
    except Exception:
        arm_connected = False

    if (not arm_connected) and len(by_id_paths) == 1:
        p = by_id_paths[0]
        port = os.path.realpath(p)
        # If the only device isn't explicitly known to be the Arduino port, assume it's the LiDAR.
        if not arduino_port or port != arduino_port:
            return {
                "connected": True,
                "port": port,
                "by_id": p,
                "note": "single serial device present while arm disconnected; assuming LiDAR",
            }

    # 2) Fallback: scan ttyUSB/ttyACM and exclude Arduino port
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if arduino_port:
        candidates = [c for c in candidates if c != arduino_port]

    if candidates:
        return {
            "connected": True,
            "port": candidates[0],
            "by_id": None,
            "note": "heuristic: first non-arduino serial port",
            "candidates": candidates[:5],
        }

    return {
        "connected": False,
        "port": None,
        "by_id": None,
    }

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("zen_te_server")

# -----------------------------------------------------------------------------
# Global State
# -----------------------------------------------------------------------------

arm_bridge: Optional[TeArmBridge] = None


# -----------------------------------------------------------------------------
# Lifespan (startup/shutdown)
# -----------------------------------------------------------------------------

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage arm connection lifecycle."""
    global arm_bridge
    
    arm_bridge = TeArmBridge()
    connected = await arm_bridge.connect(auto_detect=True)
    
    if connected:
        logger.info("TE arm connected successfully")
    else:
        logger.warning("TE arm not connected - running in simulation mode")
    
    yield
    
    if arm_bridge:
        await arm_bridge.disconnect()
        logger.info("TE arm disconnected")


# -----------------------------------------------------------------------------
# App Configuration
# -----------------------------------------------------------------------------

app = FastAPI(
    title="Zen TE Node - Robot Arm",
    description="Robot arm control for VOIGHT CLUSTER (手)",
    version="0.1.0",
    lifespan=lifespan,
)


# -----------------------------------------------------------------------------
# Pydantic Models
# -----------------------------------------------------------------------------

class JointPositions(BaseModel):
    """Joint positions for the 6-DOF arm."""
    j1: float = Field(90, ge=0, le=180, description="Base rotation")
    j2: float = Field(90, ge=0, le=180, description="Shoulder")
    j3: float = Field(90, ge=0, le=180, description="Elbow")
    j4: float = Field(90, ge=0, le=180, description="Wrist pitch")
    j5: float = Field(90, ge=0, le=180, description="Wrist roll")
    j6: float = Field(90, ge=0, le=180, description="Gripper")
    smooth: bool = Field(True, description="Smooth movement")
    
    def to_list(self) -> list[float]:
        return [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]


class GripperCommand(BaseModel):
    """Gripper control command."""
    position: float = Field(..., ge=0, le=180, description="Gripper position 0-180")


class ArmStatusResponse(BaseModel):
    """Current arm status."""
    connected: bool
    enabled: bool
    joints: list[float]
    # Servo channel labeling (matches current TE wiring observed in KOKORO UI):
    # 0=elbow, 1=shoulder, 2=wrist_swivel, 3=base, 4=grip, 5=wrist_bend
    joint_names: list[str] = [
        "elbow",
        "shoulder",
        "wrist_swivel",
        "base",
        "grip",
        "wrist_bend",
    ]


class CommandResponse(BaseModel):
    """Generic command response."""
    success: bool
    message: str
    joints: Optional[list[float]] = None


# -----------------------------------------------------------------------------
# API Routes
# -----------------------------------------------------------------------------

@app.get("/")
async def root():
    """Root - redirect to docs."""
    return {"node": "te", "kanji": "手", "docs": "/docs"}


@app.get("/status")
async def status() -> dict:
    """
    Node status endpoint for cluster monitoring.
    """
    arm_status = arm_bridge.state if arm_bridge else ArmState(joints=[90.0]*6)
    lidar = get_lidar_status()
    
    return {
        "node": "te",
        "kanji": "手",
        "role": "limbs",
        "capabilities": {
            "robot_arm": True,
            "gripper": True,
            "dof": 6,
            "lidar": True,
        },
        "hardware": get_hardware_stats(),
        "arm": {
            "connected": arm_status.connected,
            "enabled": arm_status.enabled,
            "joints": arm_status.joints,
        },
        "lidar": lidar,
        "version": "0.1.0",
    }


@app.get("/arm/status", response_model=ArmStatusResponse)
async def arm_status():
    """Get current arm status."""
    if not arm_bridge:
        raise HTTPException(status_code=503, detail="Arm bridge not initialized")
    
    state = await arm_bridge.get_status()
    return ArmStatusResponse(
        connected=state.connected,
        enabled=state.enabled,
        joints=state.joints,
    )


@app.post("/arm/move", response_model=CommandResponse)
async def arm_move(positions: JointPositions):
    """
    Move arm to specified joint positions.
    All positions in degrees (0-180).
    """
    if not arm_bridge or not arm_bridge.state.connected:
        raise HTTPException(status_code=503, detail="Arm not connected")
    
    success = await arm_bridge.move_joints(
        positions.to_list(), 
        smooth=positions.smooth
    )
    
    return CommandResponse(
        success=success,
        message="Move complete" if success else "Move failed",
        joints=arm_bridge.state.joints,
    )


@app.post("/arm/home", response_model=CommandResponse)
async def arm_home():
    """Move arm to home position (all joints at 90°)."""
    if not arm_bridge or not arm_bridge.state.connected:
        raise HTTPException(status_code=503, detail="Arm not connected")
    
    success = await arm_bridge.home()
    
    return CommandResponse(
        success=success,
        message="Moved to home" if success else "Home failed",
        joints=arm_bridge.state.joints,
    )


@app.post("/arm/gripper", response_model=CommandResponse)
async def arm_gripper(cmd: GripperCommand):
    """Control gripper position."""
    if not arm_bridge or not arm_bridge.state.connected:
        raise HTTPException(status_code=503, detail="Arm not connected")
    
    success = await arm_bridge.set_gripper(int(cmd.position))
    
    return CommandResponse(
        success=success,
        message=f"Gripper set to {cmd.position}" if success else "Gripper command failed",
        joints=arm_bridge.state.joints,
    )


@app.post("/arm/enable", response_model=CommandResponse)
async def arm_enable():
    """Enable (attach) servos."""
    if not arm_bridge:
        raise HTTPException(status_code=503, detail="Arm bridge not initialized")
    
    success = await arm_bridge.enable()
    
    return CommandResponse(
        success=success,
        message="Servos enabled" if success else "Enable failed",
    )


@app.post("/arm/disable", response_model=CommandResponse)
async def arm_disable():
    """Disable (detach) servos - arm will go limp."""
    if not arm_bridge:
        raise HTTPException(status_code=503, detail="Arm bridge not initialized")
    
    success = await arm_bridge.disable()
    
    return CommandResponse(
        success=success,
        message="Servos disabled" if success else "Disable failed",
    )


@app.get("/dashboard", response_class=HTMLResponse)
async def dashboard():
    """Simple control dashboard."""
    state = arm_bridge.state if arm_bridge else ArmState(joints=[90.0]*6)
    
    html = """
<!DOCTYPE html>
<html>
<head>
    <title>TE Arm Control - 手</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600&family=Noto+Sans+JP:wght@300&display=swap');
        
        :root {
            --bg: #0a0a0f;
            --card: #1a1a24;
            --accent: #e63946;
            --text: #f1f1f1;
            --dim: #666;
            --online: #2ecc71;
            --offline: #e74c3c;
        }
        
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'JetBrains Mono', monospace;
            background: var(--bg);
            color: var(--text);
            min-height: 100vh;
            padding: 2rem;
        }
        
        header {
            text-align: center;
            margin-bottom: 2rem;
        }
        
        .kanji {
            font-family: 'Noto Sans JP', sans-serif;
            font-size: 4rem;
            color: var(--accent);
        }
        
        h1 { font-weight: 400; color: var(--dim); letter-spacing: 0.2em; }
        
        .status {
            display: flex;
            justify-content: center;
            gap: 2rem;
            margin-bottom: 2rem;
        }
        
        .status-item {
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }
        
        .dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
        }
        
        .dot.online { background: var(--online); box-shadow: 0 0 10px var(--online); }
        .dot.offline { background: var(--offline); }
        
        .controls {
            max-width: 600px;
            margin: 0 auto;
            background: var(--card);
            padding: 2rem;
            border-radius: 12px;
        }
        
        .joint {
            margin-bottom: 1.5rem;
        }
        
        .joint-header {
            display: flex;
            justify-content: space-between;
            margin-bottom: 0.5rem;
        }
        
        .joint-name { color: var(--dim); }
        .joint-value { color: var(--accent); }
        
        input[type="range"] {
            width: 100%;
            height: 8px;
            border-radius: 4px;
            background: #333;
            outline: none;
            -webkit-appearance: none;
        }
        
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: var(--accent);
            cursor: pointer;
        }
        
        .buttons {
            display: flex;
            gap: 1rem;
            margin-top: 2rem;
        }
        
        button {
            flex: 1;
            padding: 1rem;
            border: 1px solid #333;
            border-radius: 8px;
            background: transparent;
            color: var(--text);
            font-family: inherit;
            cursor: pointer;
            transition: all 0.2s;
        }
        
        button:hover {
            background: var(--accent);
            border-color: var(--accent);
        }
        
        button.danger:hover {
            background: var(--offline);
            border-color: var(--offline);
        }
    </style>
</head>
<body>
    <header>
        <div class="kanji">手</div>
        <h1>TE ARM CONTROL</h1>
    </header>
    
    <div class="status">
        <div class="status-item">
            <div class="dot """ + ("online" if state.connected else "offline") + """"></div>
            <span>""" + ("Connected" if state.connected else "Disconnected") + """</span>
        </div>
        <div class="status-item">
            <div class="dot """ + ("online" if state.enabled else "offline") + """"></div>
            <span>""" + ("Enabled" if state.enabled else "Disabled") + """</span>
        </div>
    </div>
    
    <div class="controls">
        <div class="joint">
            <div class="joint-header">
                <span class="joint-name">J1 - Base</span>
                <span class="joint-value" id="v1">""" + str(int(state.joints[0])) + """°</span>
            </div>
            <input type="range" id="j1" min="0" max="180" value=\"""" + str(int(state.joints[0])) + """\" oninput="updateJoint(1)">
        </div>
        <div class="joint">
            <div class="joint-header">
                <span class="joint-name">J2 - Shoulder</span>
                <span class="joint-value" id="v2">""" + str(int(state.joints[1])) + """°</span>
            </div>
            <input type="range" id="j2" min="0" max="180" value=\"""" + str(int(state.joints[1])) + """\" oninput="updateJoint(2)">
        </div>
        <div class="joint">
            <div class="joint-header">
                <span class="joint-name">J3 - Elbow</span>
                <span class="joint-value" id="v3">""" + str(int(state.joints[2])) + """°</span>
            </div>
            <input type="range" id="j3" min="0" max="180" value=\"""" + str(int(state.joints[2])) + """\" oninput="updateJoint(3)">
        </div>
        <div class="joint">
            <div class="joint-header">
                <span class="joint-name">J4 - Wrist Pitch</span>
                <span class="joint-value" id="v4">""" + str(int(state.joints[3])) + """°</span>
            </div>
            <input type="range" id="j4" min="0" max="180" value=\"""" + str(int(state.joints[3])) + """\" oninput="updateJoint(4)">
        </div>
        <div class="joint">
            <div class="joint-header">
                <span class="joint-name">J5 - Wrist Roll</span>
                <span class="joint-value" id="v5">""" + str(int(state.joints[4])) + """°</span>
            </div>
            <input type="range" id="j5" min="0" max="180" value=\"""" + str(int(state.joints[4])) + """\" oninput="updateJoint(5)">
        </div>
        <div class="joint">
            <div class="joint-header">
                <span class="joint-name">J6 - Gripper</span>
                <span class="joint-value" id="v6">""" + str(int(state.joints[5])) + """°</span>
            </div>
            <input type="range" id="j6" min="0" max="180" value=\"""" + str(int(state.joints[5])) + """\" oninput="updateJoint(6)">
        </div>
        
        <div class="buttons">
            <button onclick="sendMove()">MOVE</button>
            <button onclick="goHome()">HOME</button>
            <button onclick="enable()">ENABLE</button>
            <button onclick="disable()" class="danger">DISABLE</button>
        </div>
    </div>
    
    <script>
        function updateJoint(n) {
            const val = document.getElementById('j' + n).value;
            document.getElementById('v' + n).textContent = val + '°';
        }
        
        async function sendMove() {
            const data = {
                j1: parseInt(document.getElementById('j1').value),
                j2: parseInt(document.getElementById('j2').value),
                j3: parseInt(document.getElementById('j3').value),
                j4: parseInt(document.getElementById('j4').value),
                j5: parseInt(document.getElementById('j5').value),
                j6: parseInt(document.getElementById('j6').value),
                smooth: true
            };
            await fetch('/arm/move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(data)
            });
        }
        
        async function goHome() {
            await fetch('/arm/home', {method: 'POST'});
            for (let i = 1; i <= 6; i++) {
                document.getElementById('j' + i).value = 90;
                document.getElementById('v' + i).textContent = '90°';
            }
        }
        
        async function enable() {
            await fetch('/arm/enable', {method: 'POST'});
            location.reload();
        }
        
        async function disable() {
            await fetch('/arm/disable', {method: 'POST'});
            location.reload();
        }
    </script>
</body>
</html>
"""
    return HTMLResponse(content=html)


# -----------------------------------------------------------------------------
# Main Entry Point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    import uvicorn
    
    print("=" * 60)
    print("  TE Node - Robot Arm Server (手)")
    print("  Starting on http://0.0.0.0:8027")
    print("=" * 60)
    
    uvicorn.run(app, host="0.0.0.0", port=8027)

