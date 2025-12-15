#!/usr/bin/env python3
"""
Zen KOKORO Node Server
======================
Orchestration node for the VOIGHT CLUSTER.
Provides dashboard, cluster status, and job dispatch.

Node: KOKORO (心 - heart/mind)
Role: Orchestration / Dashboard / Coordination
Port: 8025
"""

import asyncio
from datetime import datetime
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, RedirectResponse, FileResponse
from pydantic import BaseModel
import httpx
import requests
import psutil
import os


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

# -----------------------------------------------------------------------------
# App Configuration
# -----------------------------------------------------------------------------

app = FastAPI(
    title="Zen KOKORO Node",
    description="Orchestration node for the VOIGHT CLUSTER (心)",
    version="0.1.0",
)

# Known nodes in the cluster
CLUSTER_NODES = {
    "kokoro": {"url": "https://kokoro.local", "kanji": "心", "role": "orchestration"},
    "me": {"url": "http://me.local:8026", "kanji": "目", "role": "vision"},
    "te": {"url": "http://te.local:8027", "kanji": "手", "role": "limbs"},
}

# Timeout for node health checks (seconds)
NODE_TIMEOUT = 2.0


# -----------------------------------------------------------------------------
# Pydantic Models
# -----------------------------------------------------------------------------

class NodeStatus(BaseModel):
    """Status of a single node."""
    name: str
    kanji: str
    role: str
    url: str
    online: bool
    status: dict | None = None
    error: str | None = None


class ClusterStatus(BaseModel):
    """Status of the entire VOIGHT CLUSTER."""
    timestamp: str
    orchestrator: str
    nodes: list[NodeStatus]
    summary: dict


class DispatchRequest(BaseModel):
    """Request to dispatch a job to a node."""
    target_node: str
    endpoint: str = "/compute"
    payload: dict


class DispatchResult(BaseModel):
    """Result from a dispatched job."""
    success: bool
    target_node: str
    endpoint: str
    response: dict | None = None
    error: str | None = None


# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------

async def check_node_status(name: str, info: dict) -> NodeStatus:
    """
    Check the status of a single node.
    Returns a NodeStatus with online=True/False and status data or error.
    """
    # Skip self-check for kokoro - but include hardware stats
    if name == "kokoro":
        return NodeStatus(
            name=name,
            kanji=info["kanji"],
            role=info["role"],
            url=info["url"],
            online=True,
            status={
                "node": "kokoro", 
                "note": "This is the orchestrator",
                "hardware": get_hardware_stats(),
                "capabilities": {
                    "dashboard": True,
                    "cluster_monitor": True,
                    "job_dispatch": True,
                },
                "voice_ui": {
                    "enabled": True,
                    "url": "http://kokoro.local:3000",
                }
            },
        )
    
    try:
        async with httpx.AsyncClient(timeout=NODE_TIMEOUT) as client:
            response = await client.get(f"{info['url']}/status")
            response.raise_for_status()
            status_json = response.json()

            # TE compatibility:
            # Some TE builds expose servo state at /arm (and may not include "arm" in /status).
            # Normalize here so the dashboard always renders servo details + control link.
            if name == "te" and isinstance(status_json, dict) and "arm" not in status_json:
                try:
                    arm_resp = await client.get(f"{info['url']}/arm")
                    if arm_resp.status_code == 200:
                        arm_json = arm_resp.json()
                        joints = None
                        if isinstance(arm_json, dict) and "servos" in arm_json:
                            joints = [float(s.get("angle", 90)) for s in arm_json.get("servos", [])]
                        elif isinstance(arm_json, dict) and "joints" in arm_json:
                            joints = [float(x) for x in arm_json.get("joints", [])]

                        status_json["arm"] = {
                            "connected": bool(arm_json.get("connected", False)) if isinstance(arm_json, dict) else False,
                            # Some firmware doesn't expose "enabled" separately; treat connected as enabled.
                            "enabled": bool(arm_json.get("connected", False)) if isinstance(arm_json, dict) else False,
                            "joints": joints if joints is not None else [90.0] * 6,
                            "port": arm_json.get("port") if isinstance(arm_json, dict) else None,
                            "servos": arm_json.get("servos") if isinstance(arm_json, dict) else None,
                        }
                except Exception:
                    pass
            return NodeStatus(
                name=name,
                kanji=info["kanji"],
                role=info["role"],
                url=info["url"],
                online=True,
                status=status_json,
            )
    except httpx.TimeoutException:
        return NodeStatus(
            name=name,
            kanji=info["kanji"],
            role=info["role"],
            url=info["url"],
            online=False,
            error="Timeout - node not responding",
        )
    except httpx.ConnectError:
        return NodeStatus(
            name=name,
            kanji=info["kanji"],
            role=info["role"],
            url=info["url"],
            online=False,
            error="Connection refused - node may be offline",
        )
    except Exception as e:
        return NodeStatus(
            name=name,
            kanji=info["kanji"],
            role=info["role"],
            url=info["url"],
            online=False,
            error=str(e),
        )

async def get_network_services() -> list[NodeStatus]:
    """
    Discover services on the local network using avahi-browse.
    """
    try:
        # Run avahi-browse to find _http._tcp services
        cmd = ["avahi-browse", "-r", "-p", "-t", "_http._tcp"]
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE
        )
        stdout, stderr = await proc.communicate()
        
        services = []
        seen_urls = set()
        
        if stdout:
            lines = stdout.decode().splitlines()
            for line in lines:
                if line.startswith("="):
                    parts = line.split(";")
                    if len(parts) >= 9:
                        # Format: =;interface;protocol;name;service_type;domain;hostname;ip;port;txt
                        name = parts[3].replace("\\032", " ")
                        hostname = parts[6]
                        ip = parts[7]
                        port = parts[8]
                        
                        url = f"http://{ip}:{port}"
                        if url in seen_urls:
                            continue
                        seen_urls.add(url)

                        # Check if this is already a known cluster node
                        is_known = False
                        for node_info in CLUSTER_NODES.values():
                            # Normalize URLs for comparison (remove trailing slash, etc)
                            if node_info["url"].rstrip('/') == url.rstrip('/'):
                                is_known = True
                                break
                        if is_known:
                            continue
                            
                        # Generate a Kanji-like symbol or initial
                        kanji = name[0].upper() if name else "?"
                        
                        services.append(NodeStatus(
                            name=name,
                            kanji=kanji,
                            role="Network Service",
                            url=url,
                            online=True,
                            status={"type": "discovered", "hostname": hostname}
                        ))
        return services
    except Exception as e:
        print(f"Error discovering services: {e}")
        return []

# -----------------------------------------------------------------------------
# API Routes
# -----------------------------------------------------------------------------

@app.get("/")
async def root():
    """Root endpoint - redirect to dashboard."""
    return RedirectResponse(url="/dashboard")


@app.get("/cert")
async def download_certificate():
    """Download SSL certificate for installation on mobile devices."""
    import os
    from fastapi.responses import FileResponse
    
    cert_path = os.path.join(os.path.dirname(__file__), "ssl", "cert.pem")
    if os.path.exists(cert_path):
        return FileResponse(
            cert_path,
            media_type="application/x-pem-file",
            filename="kokoro-cert.pem"
        )
    return {"error": "Certificate not found"}


@app.get("/assets/Skeleton.usdz")
async def asset_skeleton_usdz():
    """Serve Skeleton.usdz for BODY (身) mode (iOS Quick Look / AR)."""
    repo_root = os.path.dirname(os.path.dirname(__file__))
    path = os.path.join(repo_root, "te_arm", "Skeleton.usdz")
    if not os.path.exists(path):
        return {"error": "Skeleton.usdz not found", "path": path}
    return FileResponse(
        path,
        media_type="model/vnd.usdz+zip",
        filename="Skeleton.usdz",
        headers={"Cache-Control": "public, max-age=3600"},
    )


@app.get("/assets/AnatomyGeometry-Full-GLB.glb")
async def asset_anatomy_glb():
    """Serve AnatomyGeometry-Full-GLB.glb for BODY (身) mode (web-friendly)."""
    repo_root = os.path.dirname(os.path.dirname(__file__))
    path = os.path.join(repo_root, "te_arm", "AnatomyGeometry-Full-GLB.glb")
    if not os.path.exists(path):
        return {"error": "AnatomyGeometry-Full-GLB.glb not found", "path": path}
    return FileResponse(
        path,
        media_type="model/gltf-binary",
        filename="AnatomyGeometry-Full-GLB.glb",
        headers={"Cache-Control": "public, max-age=3600"},
    )


@app.get("/setup", response_class=HTMLResponse)
async def setup_page():
    """Setup instructions for all platforms."""
    return """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Zen Setup - VOIGHT CLUSTER</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: linear-gradient(135deg, #0a0a12 0%, #1a1a2e 100%);
            color: #e0e0e0;
            min-height: 100vh;
            padding: 20px;
        }
        .container { max-width: 800px; margin: 0 auto; }
        h1 { 
            text-align: center; 
            color: #6366f1; 
            margin-bottom: 10px;
            font-size: 2rem;
        }
        .subtitle {
            text-align: center;
            color: #888;
            margin-bottom: 30px;
        }
        .platform {
            background: rgba(255,255,255,0.05);
            border-radius: 12px;
            padding: 20px;
            margin-bottom: 20px;
            border: 1px solid rgba(99, 102, 241, 0.2);
        }
        .platform h2 {
            display: flex;
            align-items: center;
            gap: 10px;
            color: #f1f1f1;
            margin-bottom: 15px;
            font-size: 1.3rem;
        }
        .platform h2 .icon { font-size: 1.5rem; }
        ol { padding-left: 20px; }
        li { margin-bottom: 10px; line-height: 1.6; }
        code {
            background: rgba(99, 102, 241, 0.2);
            padding: 2px 8px;
            border-radius: 4px;
            font-family: 'SF Mono', Monaco, monospace;
        }
        .url-box {
            background: #1a1a2e;
            border: 1px solid #6366f1;
            border-radius: 8px;
            padding: 15px;
            margin: 15px 0;
            text-align: center;
        }
        .url-box a {
            color: #6366f1;
            font-size: 1.1rem;
            text-decoration: none;
        }
        .url-box a:hover { text-decoration: underline; }
        .download-btn {
            display: inline-block;
            background: #6366f1;
            color: white;
            padding: 12px 24px;
            border-radius: 8px;
            text-decoration: none;
            margin: 10px 0;
            font-weight: 500;
        }
        .download-btn:hover { background: #5558e3; }
        .note {
            background: rgba(230, 57, 70, 0.1);
            border-left: 3px solid #e63946;
            padding: 10px 15px;
            margin: 15px 0;
            font-size: 0.9rem;
        }
        .success {
            background: rgba(34, 197, 94, 0.1);
            border-left: 3px solid #22c55e;
            padding: 10px 15px;
            margin: 15px 0;
        }
        .quick-test {
            text-align: center;
            margin-top: 30px;
            padding-top: 20px;
            border-top: 1px solid rgba(255,255,255,0.1);
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>⟁ Zen Setup</h1>
        <p class="subtitle">Enable voice & camera access on your device</p>
        
        <div class="url-box">
            <strong>Dashboard URL:</strong><br>
            <a href="/dashboard">https://kokoro.local:8025/dashboard</a><br>
            <small style="color:#888">or https://10.0.0.205:8025/dashboard</small>
        </div>

        <div class="platform">
            <h2><span class="icon">🍎</span> iPhone / iPad</h2>
            <ol>
                <li>Open <strong>Safari</strong> (not Chrome) and tap: <a href="/cert" class="download-btn">Download Certificate</a></li>
                <li>Tap <strong>Allow</strong> when prompted to download the profile</li>
                <li>Open <strong>Settings</strong> → tap <strong>Profile Downloaded</strong> (near top)</li>
                <li>Tap <strong>Install</strong> → enter passcode → <strong>Install</strong> again</li>
                <li>Go to <strong>Settings → General → About → Certificate Trust Settings</strong></li>
                <li>Toggle ON <strong>"Zen KOKORO"</strong> → tap <strong>Continue</strong></li>
                <li>Open Safari → go to the dashboard URL above</li>
            </ol>
            <div class="success">✓ Microphone and camera will now work!</div>
        </div>

        <div class="platform">
            <h2><span class="icon">🤖</span> Android</h2>
            <ol>
                <li>Tap to download: <a href="/cert" class="download-btn">Download Certificate</a></li>
                <li>Open <strong>Settings → Security → Encryption & credentials</strong></li>
                <li>Tap <strong>Install a certificate → CA certificate</strong></li>
                <li>Select the downloaded <code>kokoro-cert.pem</code> file</li>
                <li>Name it "Zen KOKORO" and tap OK</li>
                <li>Open Chrome → go to the dashboard URL</li>
            </ol>
            <div class="note">On some Android versions: Settings → Biometrics & Security → Other security settings → Install from storage</div>
        </div>

        <div class="platform">
            <h2><span class="icon">🍎</span> Mac</h2>
            <ol>
                <li>Download: <a href="/cert" class="download-btn">Download Certificate</a></li>
                <li>Double-click <code>kokoro-cert.pem</code> to open Keychain Access</li>
                <li>Find "Zen KOKORO" in the list, double-click it</li>
                <li>Expand <strong>Trust</strong> → set "When using this certificate" to <strong>Always Trust</strong></li>
                <li>Close and enter your password</li>
                <li>Restart your browser, then visit the dashboard</li>
            </ol>
            <div class="success">✓ Works in Safari, Chrome, Firefox, and all browsers!</div>
        </div>

        <div class="platform">
            <h2><span class="icon">🪟</span> Windows</h2>
            <ol>
                <li>Download: <a href="/cert" class="download-btn">Download Certificate</a></li>
                <li>Double-click <code>kokoro-cert.pem</code></li>
                <li>Click <strong>Install Certificate</strong></li>
                <li>Select <strong>Local Machine</strong> → Next</li>
                <li>Select <strong>Place all certificates in the following store</strong></li>
                <li>Click Browse → select <strong>Trusted Root Certification Authorities</strong></li>
                <li>Click Next → Finish → Yes to confirm</li>
                <li>Restart your browser</li>
            </ol>
        </div>

        <div class="platform">
            <h2><span class="icon">🐧</span> Linux</h2>
            <ol>
                <li>Download and install:
                    <pre style="background:#111;padding:10px;border-radius:4px;margin:10px 0;overflow-x:auto"><code>wget https://kokoro.local:8025/cert -O kokoro-cert.pem --no-check-certificate
sudo cp kokoro-cert.pem /usr/local/share/ca-certificates/kokoro-cert.crt
sudo update-ca-certificates</code></pre>
                </li>
                <li>Restart your browser</li>
            </ol>
            <div class="note">For Chrome only: Go to chrome://settings/certificates → Authorities → Import</div>
        </div>

        <div class="quick-test">
            <h3 style="margin-bottom:15px">Quick Access (Accept Warning)</h3>
            <p style="color:#888;margin-bottom:15px">Don't want to install the certificate? Just accept the browser warning:</p>
            <a href="/dashboard" class="download-btn" style="background:#22c55e">→ Go to Dashboard</a>
            <p style="color:#666;font-size:0.85rem;margin-top:10px">
                Click "Advanced" → "Proceed to kokoro.local"<br>
                (Microphone may not work on mobile without installing cert)
            </p>
        </div>
    </div>
</body>
</html>
"""


@app.get("/proxy/me/stream")
def proxy_me_stream():
    """Proxy ME camera stream through KOKORO for HTTPS access."""
    import requests
    from fastapi.responses import StreamingResponse
    
    def stream_generator():
        try:
            # No timeout for continuous MJPEG stream
            with requests.get("http://me.local:8028/stream", stream=True, timeout=None) as r:
                for chunk in r.iter_content(chunk_size=8192):
                    if chunk:
                        yield chunk
        except Exception as e:
            print(f"Stream proxy error: {e}")
    
    return StreamingResponse(
        stream_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


@app.get("/proxy/me/snapshot")
async def proxy_me_snapshot():
    """Proxy ME camera snapshot through KOKORO."""
    import httpx
    from fastapi.responses import Response
    
    async with httpx.AsyncClient(timeout=30) as client:
        response = await client.get("http://me.local:8028/snapshot")
        return Response(content=response.content, media_type="image/jpeg")


@app.get("/proxy/me/analyze")
async def proxy_me_analyze(prompt: str = "Describe what you see"):
    """Proxy ME vision analysis through KOKORO."""
    import httpx
    
    async with httpx.AsyncClient(timeout=120) as client:
        response = await client.get(f"http://me.local:8028/analyze?prompt={prompt}")
        return response.json()


@app.get("/proxy/te/arm")
async def proxy_te_arm():
    """Proxy TE arm state through KOKORO (avoids HTTPS mixed-content/CORS issues in browsers)."""
    import httpx
    async with httpx.AsyncClient(timeout=5) as client:
        resp = await client.get("http://te.local:8027/arm")
        resp.raise_for_status()
        return resp.json()


@app.post("/proxy/te/move")
async def proxy_te_move(request: Request):
    """Proxy TE /move (servo commands) through KOKORO."""
    import httpx
    body = await request.json()
    async with httpx.AsyncClient(timeout=10) as client:
        resp = await client.post("http://te.local:8027/move", json=body)
        resp.raise_for_status()
        return resp.json()


@app.post("/proxy/te/gripper")
async def proxy_te_gripper(request: Request):
    """Proxy TE /gripper through KOKORO."""
    import httpx
    body = await request.json()
    async with httpx.AsyncClient(timeout=10) as client:
        resp = await client.post("http://te.local:8027/gripper", json=body)
        resp.raise_for_status()
        return resp.json()


@app.post("/proxy/te/connect")
async def proxy_te_connect():
    """Proxy TE /connect through KOKORO."""
    import httpx
    async with httpx.AsyncClient(timeout=10) as client:
        resp = await client.post("http://te.local:8027/connect")
        resp.raise_for_status()
        return resp.json()


@app.post("/proxy/te/disconnect")
async def proxy_te_disconnect():
    """Proxy TE /disconnect through KOKORO."""
    import httpx
    async with httpx.AsyncClient(timeout=10) as client:
        resp = await client.post("http://te.local:8027/disconnect")
        resp.raise_for_status()
        return resp.json()


@app.post("/proxy/te/home")
async def proxy_te_home():
    """Proxy TE /home through KOKORO."""
    import httpx
    async with httpx.AsyncClient(timeout=10) as client:
        resp = await client.post("http://te.local:8027/home")
        resp.raise_for_status()
        return resp.json()


@app.post("/proxy/llm/generate")
async def proxy_llm_generate(request: Request):
    """Proxy Ollama LLM requests through KOKORO for HTTPS compatibility."""
    import httpx
    
    body = await request.json()
    
    async with httpx.AsyncClient(timeout=120) as client:
        response = await client.post(
            "http://localhost:11434/api/generate",
            json=body
        )
        return response.json()


@app.get("/status")
def status():
    """
    Status endpoint - returns node identity and info.
    """
    return {
        "node": "kokoro",
        "kanji": "心",
        "role": "orchestration",
        "capabilities": {
            "dashboard": True,
            "cluster_monitor": True,
            "job_dispatch": True,
        },
        "hardware": get_hardware_stats(),
        "cluster_nodes": list(CLUSTER_NODES.keys()),
        "version": "0.1.0",
        "note": "Zen orchestration node (KOKORO) - VOIGHT CLUSTER",
    }


@app.get("/cluster")
async def cluster_status() -> ClusterStatus:
    """
    Get the status of all nodes in the cluster.
    Queries each node in parallel and returns aggregated status.
    """
    # Check all nodes in parallel
    tasks = [
        check_node_status(name, info) 
        for name, info in CLUSTER_NODES.items()
    ]
    nodes = await asyncio.gather(*tasks)
    
    # Build summary
    online_count = sum(1 for n in nodes if n.online)
    
    return ClusterStatus(
        timestamp=datetime.now().isoformat(),
        orchestrator="kokoro",
        nodes=nodes,
        summary={
            "total_nodes": len(nodes),
            "online": online_count,
            "offline": len(nodes) - online_count,
        },
    )


@app.get("/dashboard", response_class=HTMLResponse)
async def dashboard(request: Request):
    """
    Simple HTML dashboard showing cluster status.
    """
    # Get cluster status
    cluster = await cluster_status()
    
    # Get discovered services
    discovered = await get_network_services()
    
    # Combine info for stats
    total_nodes = cluster.summary["total_nodes"] + len(discovered)
    online_count = cluster.summary["online"] + len(discovered) # Discovered are online by definition
    offline_count = cluster.summary["offline"]

    # Build HTML
    html = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>VOIGHT CLUSTER Dashboard</title>
    <script>
        // Zen Voice System
        window.zenListening = false;
        window.zenStream = null;
        window.zenRecognition = null;

        // Zen Stereo Eye Visualizer (side-by-side stereo MJPEG split)
        // ipdMeters is exposed per request; we map it to a small pixel shift tweak for aligning stereo halves.
        window.zenStereo = {
            ipdMeters: 0.064,
            shiftPx: 0,  // positive shifts both eyes right; fine-tune if your stereo feed is slightly off-center
        };

        window.applyZenStereo = function() {
            const leftImg = document.getElementById('left-eye-img');
            const rightImg = document.getElementById('right-eye-img');
            if (!leftImg || !rightImg) return;

            const leftLabel = leftImg.parentElement ? leftImg.parentElement.querySelector('.eye-label') : null;
            const rightLabel = rightImg.parentElement ? rightImg.parentElement.querySelector('.eye-label') : null;

            // Keep this intentionally simple: we split by object-position (left/right) and optionally nudge via CSS var.
            const shift = (window.zenStereo && Number.isFinite(window.zenStereo.shiftPx)) ? window.zenStereo.shiftPx : 0;
            leftImg.style.setProperty('--stereo-shift', `${shift}px`);
            rightImg.style.setProperty('--stereo-shift', `${shift}px`);

            // Debug: show which pane + current shift value
            if (leftLabel) leftLabel.textContent = `L`;
            if (rightLabel) rightLabel.textContent = `R`;
            if (leftLabel) leftLabel.title = `Left eye (shiftPx=${shift})`;
            if (rightLabel) rightLabel.title = `Right eye (shiftPx=${shift})`;
        };
        
        window.toggleZenVoice = async function() {
            // Support both NODES and DANWA views - check which is visible
            const danwaView = document.getElementById('danwa-view');
            const isDanwa = danwaView && danwaView.style.display !== 'none';
            
            const btn = isDanwa ? document.getElementById('danwa-talk-btn') : document.getElementById('zen-talk-btn');
            const status = isDanwa ? document.getElementById('danwa-status') : document.getElementById('zen-status');
            const thought = document.getElementById('zen-thought');
            
            if (window.zenListening) {
                // Stop listening
                window.zenListening = false;
                if (window.zenRecognition) window.zenRecognition.stop();
                if (window.zenStream) window.zenStream.getTracks().forEach(t => t.stop());
                if (window.zenDisconnectAudio) window.zenDisconnectAudio();
                if (window.zenDisconnectDanwaAudio) window.zenDisconnectDanwaAudio();
                if (typeof setZenState === 'function') setZenState('idle');
                if (btn) {
                    if (btn.id === 'danwa-talk-btn') {
                        btn.querySelector('.btn-icon').textContent = '🎤';
                        btn.querySelector('.btn-status').textContent = 'Speak';
                    } else {
                        btn.setAttribute('aria-label', 'Talk to Zen');
                        btn.innerHTML = '<span style="font-size:1.1rem;">🎤</span>';
                    }
                }
                if (status && status.id !== 'danwa-status') status.textContent = 'Click to speak with Zen';
                return;
            }
            
            // Start listening
            try {
                window.zenStream = await navigator.mediaDevices.getUserMedia({ audio: true });
                window.zenListening = true;
                if (window.zenConnectAudio) window.zenConnectAudio(window.zenStream);
                if (window.zenConnectDanwaAudio) window.zenConnectDanwaAudio(window.zenStream);
                if (typeof setZenState === 'function') setZenState('listening');
                if (btn) {
                    if (btn.id === 'danwa-talk-btn') {
                        btn.querySelector('.btn-icon').textContent = '◉';
                        btn.querySelector('.btn-status').textContent = 'Listening';
                    } else {
                        btn.innerHTML = '<span style="font-size:1.1rem;">⏹</span> Stop';
                    }
                }
                if (status && status.id !== 'danwa-status') status.textContent = 'Listening...';
                if (thought) {
                    thought.textContent = '';
                    thought.classList.remove('active');
                }
                
                // Setup speech recognition
                const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
                if (!SpeechRecognition) {
                    alert('Speech recognition not supported in this browser. Try Chrome.');
                    return;
                }
                
                window.zenRecognition = new SpeechRecognition();
                window.zenRecognition.continuous = false;
                window.zenRecognition.interimResults = true;
                
                window.zenRecognition.onresult = async (event) => {
                    const transcript = event.results[0][0].transcript;
                    if (status) status.textContent = transcript;
                    
                    if (event.results[0].isFinal) {
                        if (typeof setZenState === 'function') setZenState('thinking');
                        if (btn && btn.id === 'danwa-talk-btn') {
                            btn.querySelector('.btn-status').textContent = 'Thinking';
                        }
                        if (status) {
                            if (status.id === 'danwa-status') status.textContent = 'Thinking';
                            else status.textContent = 'Thinking...';
                        }
                        
                        // Check for vision request
                        const isVision = /see|look|view|camera|eyes|watch/i.test(transcript);
                        let visionContext = '';
                        
                        if (isVision) {
                            if (status) status.textContent = '👁️ Looking';
                            try {
                                const vr = await fetch('/proxy/me/analyze?prompt=Describe+what+you+see');
                                const vd = await vr.json();
                                if (vd.description) visionContext = '\\n[VISION]: ' + vd.description;
                            } catch(e) { console.log('Vision unavailable'); }
                        }
                        
                        // Send to LLM (proxied through KOKORO for HTTPS)
                        try {
                            const resp = await fetch('/proxy/llm/generate', {
                                method: 'POST',
                                headers: {
                                    'Content-Type': 'application/json'
                                },
                                body: JSON.stringify({
                                    model: 'llama3.1:8b',
                                    prompt: 'You are Zen, AI of the VOIGHT CLUSTER robot system. KOKORO=brain, ME=eyes, TE=hands. Be concise (1-2 sentences).' + visionContext + '\\n\\nUser: ' + transcript + '\\n\\nZen:',
                                    stream: false
                                })
                            });
                            const data = await resp.json();
                            const reply = data.response || 'I could not generate a response.';
                            
                            if (typeof setZenState === 'function') setZenState('speaking');
                            if (btn && btn.id === 'danwa-talk-btn') {
                                btn.querySelector('.btn-status').textContent = 'Speaking';
                            }
                            if (status) status.textContent = reply;
                            if (thought) {
                                thought.textContent = reply;
                                thought.classList.add('active');
                            }
                            
                            // Detect emotion from response
                            const lowerReply = reply.toLowerCase();
                            if (lowerReply.includes('sorry') || lowerReply.includes('unfortunately') || lowerReply.includes('afraid')) {
                                setZenEmotion('concern');
                            } else if (lowerReply.includes('interesting') || lowerReply.includes('curious') || lowerReply.includes('wonder')) {
                                setZenEmotion('curious');
                            } else if (lowerReply.includes('not sure') || lowerReply.includes('unclear') || lowerReply.includes('hmm')) {
                                setZenEmotion('confused');
                            } else if (lowerReply.includes('!') || lowerReply.includes('great') || lowerReply.includes('excellent')) {
                                setZenEmotion('joy');
                            } else if (lowerReply.includes('alert') || lowerReply.includes('warning') || lowerReply.includes('careful')) {
                                setZenEmotion('alert');
                            } else {
                                setZenEmotion('calm');
                            }
                            
                            // Speak response
                            const utterance = new SpeechSynthesisUtterance(reply);
                            utterance.onend = () => {
                                setZenEmotion('calm');
                                window.toggleZenVoice();
                            };
                            speechSynthesis.speak(utterance);
                            
                        } catch(e) {
                            if (status) status.textContent = 'LLM error: ' + e.message;
                        }
                    }
                };
                
                window.zenRecognition.onend = () => {
                    if (window.zenListening && !speechSynthesis.speaking) {
                        window.zenRecognition.start();
                    }
                };
                
                window.zenRecognition.start();
                
            } catch(e) {
                alert('Microphone error: ' + e.message);
            }
        };
    </script>
    <style>
        @import url('https://fonts.googleapis.com/css2?family=Noto+Sans+JP:wght@300;400;700&family=JetBrains+Mono:wght@400;600&display=swap');
        
        :root {
            --bg-primary: #0a0a0f;
            --bg-secondary: #12121a;
            --bg-card: #1a1a24;
            --accent: #e63946;
            --accent-glow: rgba(230, 57, 70, 0.3);
            --text-primary: #f1f1f1;
            --text-secondary: #888;
            --online: #2ecc71;
            --offline: #e74c3c;
            --border: #2a2a3a;
            --link-hover: #3498db;
        }
        
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'JetBrains Mono', monospace;
            background: var(--bg-primary);
            color: var(--text-primary);
            min-height: 100vh;
            background-image: 
                radial-gradient(ellipse at top, rgba(230, 57, 70, 0.08) 0%, transparent 50%),
                radial-gradient(ellipse at bottom right, rgba(46, 204, 113, 0.05) 0%, transparent 50%);
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
            padding: 2rem;
        }
        
        header {
            text-align: center;
            margin-bottom: 3rem;
            padding: 2rem 0;
            border-bottom: 1px solid var(--border);
        }
        
        .title-kanji {
            font-family: 'Noto Sans JP', sans-serif;
            font-size: 4rem;
            font-weight: 300;
            color: var(--accent);
            text-shadow: 0 0 30px var(--accent-glow);
            margin-bottom: 0.5rem;
        }
        
        h1 {
            font-size: 1.5rem;
            font-weight: 400;
            letter-spacing: 0.3em;
            text-transform: uppercase;
            color: var(--text-secondary);
        }
        
        .summary {
            display: flex;
            justify-content: center;
            gap: 3rem;
            margin-bottom: 3rem;
        }
        
        .stat {
            text-align: center;
        }
        
        .stat-value {
            font-size: 2.5rem;
            font-weight: 600;
        }
        
        .stat-value.online { color: var(--online); }
        .stat-value.offline { color: var(--offline); }
        
        .stat-label {
            font-size: 0.75rem;
            color: var(--text-secondary);
            text-transform: uppercase;
            letter-spacing: 0.1em;
        }
        
        .section-title {
            margin: 2rem 0 1rem 0;
            font-size: 1.2rem;
            color: var(--text-secondary);
            border-bottom: 1px solid var(--border);
            padding-bottom: 0.5rem;
        }

        /* Collapsible discovered services */
        details.discovered-panel {
            margin: 2rem 0 1rem 0;
            border: 1px solid rgba(255,255,255,0.06);
            border-radius: 12px;
            background: rgba(255,255,255,0.02);
            overflow: hidden;
        }
        details.discovered-panel > summary {
            cursor: pointer;
            list-style: none;
            padding: 0.9rem 1.1rem;
            display: flex;
            align-items: baseline;
            justify-content: space-between;
            gap: 12px;
            user-select: none;
        }
        details.discovered-panel > summary::-webkit-details-marker {
            display: none;
        }
        .discovered-title {
            font-size: 1.05rem;
            color: var(--text-secondary);
            letter-spacing: 0.04em;
        }
        .discovered-meta {
            font-size: 0.8rem;
            color: rgba(255,255,255,0.55);
            display: flex;
            align-items: center;
            gap: 8px;
        }
        .discovered-caret {
            width: 10px;
            height: 10px;
            border-right: 2px solid rgba(255,255,255,0.45);
            border-bottom: 2px solid rgba(255,255,255,0.45);
            transform: rotate(-45deg);
            transition: transform 0.18s ease;
            margin-left: 6px;
        }
        details.discovered-panel[open] .discovered-caret {
            transform: rotate(45deg);
        }
        .discovered-body {
            padding: 0 1.1rem 1.1rem 1.1rem;
        }

        .nodes {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
            gap: 1.5rem;
            margin-bottom: 2rem;
        }
        
        .node-card {
            background: var(--bg-card);
            border: 1px solid var(--border);
            border-radius: 12px;
            padding: 1.5rem;
            position: relative;
            overflow: hidden;
            transition: transform 0.2s, box-shadow 0.2s;
        }
        
        .node-card:hover {
            transform: translateY(-2px);
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
        }
        
        .node-card.online {
            border-color: rgba(46, 204, 113, 0.3);
        }
        
        .node-card.offline {
            border-color: rgba(231, 76, 60, 0.3);
            opacity: 0.7;
        }
        
        .node-header {
            display: flex;
            align-items: center;
            gap: 1rem;
            margin-bottom: 1rem;
        }
        
        .node-kanji {
            font-family: 'Noto Sans JP', sans-serif;
            font-size: 2.5rem;
            font-weight: 300;
            opacity: 0.9;
        }
        
        .node-info h2 {
            font-size: 1.2rem;
            font-weight: 600;
            text-transform: uppercase;
            letter-spacing: 0.1em;
        }
        
        .node-role {
            font-size: 0.75rem;
            color: var(--text-secondary);
        }
        
        .node-status {
            display: flex;
            align-items: center;
            gap: 0.5rem;
            margin-bottom: 1rem;
        }
        
        .status-dot {
            width: 10px;
            height: 10px;
            border-radius: 50%;
            animation: pulse 2s infinite;
        }
        
        .status-dot.online {
            background: var(--online);
            box-shadow: 0 0 10px var(--online);
        }
        
        .status-dot.offline {
            background: var(--offline);
            box-shadow: 0 0 10px var(--offline);
            animation: none;
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        .node-url {
            font-size: 0.8rem;
            color: var(--text-secondary);
            font-family: 'JetBrains Mono', monospace;
            text-decoration: none;
            display: inline-block;
            padding: 2px 6px;
            background: rgba(255,255,255,0.05);
            border-radius: 4px;
            transition: color 0.2s, background 0.2s;
        }
        
        .node-url:hover {
            color: var(--link-hover);
            background: rgba(52, 152, 219, 0.1);
        }
        
        .node-details {
            margin-top: 1rem;
            padding-top: 1rem;
            border-top: 1px solid var(--border);
            font-size: 0.8rem;
        }
        
        .detail-row {
            display: flex;
            justify-content: space-between;
            padding: 0.25rem 0;
            align-items: center;
        }
        
        .detail-label {
            color: var(--text-secondary);
            flex-shrink: 0;
            margin-right: 1rem;
        }
        
        .detail-value {
            text-align: right;
            word-break: break-word;
        }

        /* ME stereo thumbs inside node card (HTTPS-safe via /proxy/me/stream) */
        .stereo-thumbs {
            width: 100%;
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }
        .stereo-eye {
            position: relative;
            width: 100%;
            height: 120px;
            border-radius: 10px;
            overflow: hidden;
            border: 1px solid var(--border);
            background: #050508;
        }
        .stereo-eye img {
            width: 200%;
            height: 100%;
            object-fit: cover;
            opacity: 0.92;
        }
        .stereo-eye.left img { object-position: left center; }
        .stereo-eye.right img { object-position: right center; }
        .stereo-eye.offline::after {
            content: 'Stream offline';
            position: absolute;
            inset: 0;
            display: flex;
            align-items: center;
            justify-content: center;
            color: rgba(235,238,255,0.55);
            font-size: 0.75rem;
            letter-spacing: 0.12em;
            text-transform: uppercase;
            background: radial-gradient(ellipse at center, rgba(0,0,0,0.2), rgba(0,0,0,0.75));
        }
        .pane-badge {
            position: absolute;
            top: 8px;
            left: 10px;
            z-index: 2;
            width: 22px;
            height: 22px;
            border-radius: 999px;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.8rem;
            color: rgba(235,238,255,0.9);
            background: rgba(0,0,0,0.35);
            border: 1px solid rgba(255,255,255,0.10);
            backdrop-filter: blur(6px);
        }

        /* TE inline controls inside node card */
        details.te-inline-controls {
            margin-top: 0.8rem;
            border: 1px solid rgba(255,255,255,0.08);
            border-radius: 12px;
            background: rgba(0,0,0,0.18);
            overflow: hidden;
        }
        details.te-inline-controls > summary {
            cursor: pointer;
            list-style: none;
            padding: 10px 12px;
            color: rgba(235,238,255,0.78);
            font-size: 0.78rem;
            letter-spacing: 0.12em;
            text-transform: uppercase;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }
        details.te-inline-controls > summary::-webkit-details-marker { display:none; }
        .te-inline-body { padding: 10px 12px 12px 12px; }
        .te-inline-grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
        .te-inline-servo { background: rgba(255,255,255,0.03); border: 1px solid rgba(255,255,255,0.06); border-radius: 12px; padding: 10px; }
        .te-inline-head { display:flex; justify-content: space-between; align-items: baseline; gap: 10px; margin-bottom: 8px; }
        .te-inline-name { font-family: 'JetBrains Mono', monospace; font-size: 0.72rem; letter-spacing: 0.12em; text-transform: uppercase; color: rgba(235,238,255,0.62); }
        .te-inline-val { font-family: 'JetBrains Mono', monospace; font-size: 0.85rem; color: rgba(235,238,255,0.9); }
        .te-inline-servo input[type="range"] { width: 100%; }
        .te-inline-actions { display:flex; gap: 10px; margin-top: 10px; }
        .te-inline-btn {
            border: 1px solid var(--border);
            background: rgba(255,255,255,0.04);
            color: rgba(235,238,255,0.92);
            border-radius: 10px;
            padding: 8px 10px;
            font-size: 0.8rem;
            cursor: pointer;
        }
        .te-inline-btn.primary { background: rgba(230,57,70,0.85); border-color: rgba(230,57,70,0.6); }
        .te-inline-btn:hover { opacity: 0.92; }

        .progress-bar-bg {
            width: 100px;
            height: 6px;
            background: #333;
            border-radius: 3px;
            overflow: hidden;
            display: inline-block;
        }
        
        .progress-bar-fill {
            height: 100%;
            background: var(--online);
            border-radius: 3px;
        }
        
        .error-msg {
            color: var(--offline);
            font-size: 0.75rem;
            margin-top: 0.5rem;
        }
        
        footer {
            text-align: center;
            margin-top: 3rem;
            padding-top: 2rem;
            border-top: 1px solid var(--border);
            color: var(--text-secondary);
            font-size: 0.75rem;
        }
        
        .refresh-note {
            opacity: 0.5;
        }
        
        /* Zen Presence Colors */
        :root {
            --zen-red: #e63946;
            --zen-indigo: #6366f1;
            --zen-bg: #0a0a0f;
            --zen-glow: rgba(99, 102, 241, 0.15);
        }
        
        /* Logo */
        .zen-logo {
            position: fixed;
            top: 15px;
            left: 20px;
            z-index: 1000;
            font-size: 2rem;
            color: var(--zen-red);
            opacity: 0.6;
            transition: opacity 0.4s ease;
            text-shadow: 0 0 25px rgba(230, 57, 70, 0.4);
        }
        
        .zen-logo:hover {
            opacity: 1;
        }
        
        /* Kanji Mode Selector */
        .zen-mode-selector {
            position: fixed;
            top: 20px;
            right: 20px;
            z-index: 1000;
            display: flex;
            gap: 4px;
            padding: 4px;
            opacity: 0.75;
            transition: opacity 0.4s ease;
        }
        
        .zen-mode-selector:hover {
            opacity: 1;
        }
        
        .zen-mode-btn {
            padding: 10px 14px;
            border: 1px solid rgba(255,255,255,0.08);
            background: rgba(0, 0, 0, 0.25);
            color: rgba(255,255,255,0.4);
            font-size: 1.1rem;
            cursor: pointer;
            border-radius: 8px;
            transition: all 0.3s ease;
            font-family: 'Noto Sans JP', sans-serif;
            position: relative;
        }
        
        .zen-mode-btn:hover {
            color: rgba(255,255,255,0.7);
            border-color: rgba(99, 102, 241, 0.3);
        }
        
        .zen-mode-btn:focus {
            outline: 2px solid var(--zen-indigo);
            outline-offset: 2px;
        }
        
        .zen-mode-btn[aria-selected="true"] {
            color: rgba(255,255,255,0.9);
            border-color: transparent;
        }
        
        .zen-mode-btn[aria-selected="true"]::after {
            content: '';
            position: absolute;
            bottom: 2px;
            left: 50%;
            transform: translateX(-50%);
            width: 50%;
            height: 2px;
            background: var(--zen-indigo);
            box-shadow: 0 0 8px var(--zen-indigo);
            border-radius: 2px;
        }
        
        /* Tooltip */
        .zen-mode-btn .tooltip {
            position: absolute;
            top: 100%;
            left: 50%;
            transform: translateX(-50%);
            padding: 4px 8px;
            background: rgba(0, 0, 0, 0.85);
            color: rgba(255,255,255,0.6);
            font-size: 0.6rem;
            font-family: 'JetBrains Mono', monospace;
            border-radius: 4px;
            white-space: nowrap;
            opacity: 0;
            pointer-events: none;
            transition: opacity 0.2s ease;
            margin-top: 6px;
            letter-spacing: 0.03em;
        }
        
        .zen-mode-btn:hover .tooltip,
        .zen-mode-btn:focus .tooltip {
            opacity: 1;
        }
        
        /* Legacy view-toggle support */
        .view-toggle { display: none; }
        
        /* DANWA View - Zen Presence */
        .danwa-view {
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            background: var(--zen-bg);
            padding: 20px;
            position: relative;
            overflow: hidden;
        }
        
        /* Micro grain overlay */
        .danwa-view::before {
            content: '';
            position: absolute;
            inset: 0;
            background-image: url("data:image/svg+xml,%3Csvg viewBox='0 0 256 256' xmlns='http://www.w3.org/2000/svg'%3E%3Cfilter id='noise'%3E%3CfeTurbulence type='fractalNoise' baseFrequency='0.9' numOctaves='4' stitchTiles='stitch'/%3E%3C/filter%3E%3Crect width='100%25' height='100%25' filter='url(%23noise)'/%3E%3C/svg%3E");
            opacity: 0.03;
            pointer-events: none;
        }
        
        /* Presence bloom */
        .danwa-view::after {
            content: '';
            position: absolute;
            top: 30%;
            left: 50%;
            transform: translateX(-50%);
            width: 80%;
            height: 60%;
            background: radial-gradient(ellipse, var(--zen-glow) 0%, transparent 70%);
            pointer-events: none;
            opacity: 0.8;
        }
        
        .zen-face {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 40px;
            max-width: 900px;
            width: 100%;
            position: relative;
            z-index: 1;
        }
        
        /* State Ring - binds eyes together */
        .zen-state-ring {
            position: absolute;
            top: 80px;
            left: 50%;
            transform: translateX(-50%);
            width: 750px;
            height: 300px;
            border: 1px solid rgba(99, 102, 241, 0.2);
            border-radius: 50%;
            pointer-events: none;
            box-shadow: 0 0 40px rgba(99, 102, 241, 0.1),
                        inset 0 0 60px rgba(99, 102, 241, 0.05);
            animation: stateRingBreath 4s ease-in-out infinite;
        }
        
        @keyframes stateRingBreath {
            0%, 100% { opacity: 0.4; transform: translateX(-50%) scale(1); }
            50% { opacity: 0.7; transform: translateX(-50%) scale(1.02); }
        }
        
        /* Eyes Container with Eyebrows */
        .zen-eyes-container {
            display: flex;
            flex-direction: column;
            align-items: center;
            position: relative;
        }

        /* BODY (身) - Anatomy/Skeleton model panel */
        .zen-body { display: none; width: 100%; max-width: 900px; margin: 0 auto 14px auto; }
        .danwa-view[data-mode="mi"] .zen-body { display: block; }
        .danwa-view[data-mode="mi"] .zen-eyebrows,
        .danwa-view[data-mode="mi"] .zen-eyes { display: none; }

        .zen-body-card {
            background: rgba(255,255,255,0.04);
            border: 1px solid rgba(99,102,241,0.18);
            border-radius: 16px;
            padding: 14px;
            box-shadow: 0 22px 70px rgba(0,0,0,0.45);
        }
        .zen-body-header {
            display: flex;
            align-items: center;
            justify-content: space-between;
            gap: 12px;
            margin-bottom: 10px;
        }
        .zen-body-title {
            font-family: 'JetBrains Mono', monospace;
            letter-spacing: 0.12em;
            text-transform: uppercase;
            font-size: 0.85rem;
            color: rgba(235,238,255,0.8);
        }
        .zen-body-actions { display: flex; gap: 10px; flex-wrap: wrap; justify-content: flex-end; }
        .zen-body-btn {
            display: inline-block;
            padding: 8px 12px;
            border-radius: 999px;
            border: 1px solid rgba(99,102,241,0.25);
            background: rgba(255,255,255,0.03);
            color: rgba(235,238,255,0.9);
            text-decoration: none;
            font-size: 0.82rem;
        }
        .zen-body-btn:hover { border-color: rgba(99,102,241,0.55); background: rgba(99,102,241,0.08); }
        .zen-body-hint {
            margin-top: 10px;
            color: rgba(235,238,255,0.55);
            font-size: 0.82rem;
        }
        .zen-body-hint code {
            font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
            color: rgba(235,238,255,0.8);
        }
        .asset-status {
            display: flex;
            gap: 14px;
            align-items: center;
            flex-wrap: wrap;
        }
        .asset-pill {
            display: inline-flex;
            align-items: center;
            gap: 8px;
            padding: 6px 10px;
            border-radius: 999px;
            border: 1px solid rgba(255,255,255,0.10);
            background: rgba(0,0,0,0.25);
            color: rgba(235,238,255,0.75);
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.78rem;
            letter-spacing: 0.06em;
            text-transform: uppercase;
        }
        .asset-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: rgba(230,57,70,0.85); /* default red */
            box-shadow: 0 0 14px rgba(230,57,70,0.18);
        }
        .asset-pill.ok .asset-dot {
            background: rgba(34,197,94,0.85);
            box-shadow: 0 0 14px rgba(34,197,94,0.18);
        }
        
        /* Zen Eyebrows - Wave-based emotion */
        .zen-eyebrows {
            width: 100%;
            max-width: 700px;
            height: 50px;
            margin-bottom: -15px;
            position: relative;
            z-index: 10;
        }
        
        .eyebrow {
            stroke: rgba(99, 102, 241, 0.85);
            stroke-width: 3.6;
            fill: none;
            filter: drop-shadow(0 0 7px rgba(99, 102, 241, 0.55));
            transition: stroke 0.4s ease;
        }
        
        .danwa-view[data-state="speaking"] .eyebrow {
            stroke: rgba(200, 100, 120, 0.75);
            filter: drop-shadow(0 0 4px rgba(200, 100, 120, 0.4));
        }
        
        @media (prefers-reduced-motion: reduce) {
            .zen-eyebrows { opacity: 0.7; }
        }
        
        /* Eyes - Living Organs */
        .zen-eyes {
            display: flex;
            gap: 80px;
            justify-content: center;
            position: relative;
        }
        
        .zen-emotion-indicator {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            z-index: 20;
            font-size: 2.5rem;
            opacity: 0.85;
            transition: all 0.4s ease;
            filter: drop-shadow(0 2px 8px rgba(0, 0, 0, 0.6));
        }
        
        .zen-emotion-emoji {
            display: block;
            transition: transform 0.3s ease;
        }
        
        .zen-emotion-indicator:hover .zen-emotion-emoji {
            transform: scale(1.15);
        }
        
        .eye {
            width: 320px;
            height: 220px;
            border-radius: 50% / 45%;
            overflow: hidden;
            border: 1px solid rgba(99, 102, 241, 0.3);
            background: #050508;
            position: relative;
            transition: all 0.4s ease;
        }
        
        /* Inner vignette */
        .eye::before {
            content: '';
            position: absolute;
            inset: 0;
            background: radial-gradient(ellipse at center, transparent 50%, rgba(0,0,0,0.6) 100%);
            pointer-events: none;
            z-index: 2;
        }
        
        /* Photonic inner halo */
        .eye::after {
            content: '';
            position: absolute;
            inset: -2px;
            border-radius: inherit;
            box-shadow: inset 0 0 30px rgba(99, 102, 241, 0.2);
            pointer-events: none;
            z-index: 1;
            animation: eyeHaloBreath 3s ease-in-out infinite;
        }
        
        @keyframes eyeHaloBreath {
            0%, 100% { opacity: 0.5; }
            50% { opacity: 1; }
        }
        
        .eye.left-eye {
            animation: eyeDriftLeft 8s ease-in-out infinite;
        }
        
        .eye.right-eye {
            animation: eyeDriftRight 8s ease-in-out infinite;
        }
        
        @keyframes eyeDriftLeft {
            0%, 100% { transform: translate(0, 0); }
            50% { transform: translate(-1px, 0.5px); }
        }
        
        @keyframes eyeDriftRight {
            0%, 100% { transform: translate(0, 0); }
            50% { transform: translate(1px, -0.5px); }
        }
        
        .eye img {
            width: 200%;          /* stereo split: show half-frame per eye */
            height: 100%;
            object-fit: cover;
            opacity: 0.9;
            transform: translateX(var(--stereo-shift, 0px));
        }

        /* Stereo crop: left eye shows left half, right eye shows right half */
        .eye.left-eye img {
            object-position: left center;
        }
        .eye.right-eye img {
            object-position: right center;
        }

        /* Debug overlay labels */
        .eye-label {
            position: absolute;
            top: 10px;
            left: 12px;
            z-index: 3;
            width: 22px;
            height: 22px;
            border-radius: 999px;
            display: inline-flex;
            align-items: center;
            justify-content: center;
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.8rem;
            color: rgba(235,238,255,0.92);
            background: rgba(0,0,0,0.35);
            border: 1px solid rgba(255,255,255,0.10);
            backdrop-filter: blur(6px);
        }
        
        /* Reduced motion */
        @media (prefers-reduced-motion: reduce) {
            .eye.left-eye, .eye.right-eye { animation: none; }
            .zen-state-ring { animation: none; opacity: 0.5; }
            .eye::after { animation: none; opacity: 0.7; }
        }
        
        /* Mouth - Voice Field */
        .zen-mouth {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 35px;
            width: 100%;
            max-width: 700px;
            margin-top: 20px;
        }
        
        .mouth-container {
            width: 100%;
            background: rgba(5, 5, 10, 0.9);
            border-radius: 100px;
            border: 1px solid rgba(99, 102, 241, 0.15);
            box-shadow: 0 0 50px rgba(99, 102, 241, 0.08);
            transition: all 0.4s ease;
            overflow: hidden;
            position: relative;
        }
        
        /* Wave clip container - perfect pill clipping */
        .wave-clip {
            position: relative;
            width: 100%;
            height: 130px;
            border-radius: inherit;
            overflow: hidden;
        }
        
        #danwa-waveform {
            position: absolute;
            inset: 0;
            width: 100%;
            height: 100%;
            display: block;
        }
        
        /* State-based mouth styling */
        .danwa-view[data-state="listening"] .mouth-container {
            border-color: rgba(99, 102, 241, 0.3);
            box-shadow: 0 0 70px rgba(99, 102, 241, 0.12);
        }
        
        .danwa-view[data-state="speaking"] .mouth-container {
            border-color: rgba(230, 57, 70, 0.2);
            box-shadow: 0 0 70px rgba(230, 57, 70, 0.1);
        }
        
        .danwa-status {
            display: none;
        }
        
        /* Speak Button - Subtle Zen-Consent */
        .danwa-talk-btn {
            padding: 18px 50px;
            font-size: 0.9rem;
            background: rgba(15, 15, 20, 0.9);
            color: rgba(255, 255, 255, 0.75);
            border: 1px solid rgba(230, 57, 70, 0.25);
            border-radius: 40px;
            cursor: pointer;
            transition: all 0.25s ease;
            font-weight: 400;
            box-shadow: none;
            display: flex;
            align-items: center;
            gap: 12px;
            min-width: 200px;
            justify-content: center;
            letter-spacing: 0.04em;
            position: relative;
        }
        
        .danwa-talk-btn:hover {
            border-color: rgba(230, 57, 70, 0.4);
            box-shadow: 0 0 20px rgba(230, 57, 70, 0.1);
            color: rgba(255, 255, 255, 0.9);
        }
        
        .danwa-talk-btn:focus {
            outline: 2px solid rgba(230, 57, 70, 0.4);
            outline-offset: 3px;
        }
        
        .danwa-talk-btn:active {
            background: rgba(20, 20, 25, 0.95);
        }
        
        /* Button state variations - subtle */
        .danwa-view[data-state="listening"] .danwa-talk-btn {
            background: rgba(15, 15, 25, 0.9);
            border-color: rgba(99, 102, 241, 0.3);
            color: rgba(255, 255, 255, 0.8);
        }
        
        .danwa-view[data-state="listening"] .danwa-talk-btn:hover {
            border-color: rgba(99, 102, 241, 0.45);
            box-shadow: 0 0 20px rgba(99, 102, 241, 0.1);
        }
        
        .danwa-view[data-state="thinking"] .danwa-talk-btn {
            background: rgba(12, 12, 20, 0.9);
            border-color: rgba(99, 102, 241, 0.2);
            color: rgba(255, 255, 255, 0.6);
        }
        
        .danwa-view[data-state="speaking"] .danwa-talk-btn {
            border-color: rgba(230, 57, 70, 0.35);
        }
        
        .danwa-talk-btn .btn-icon {
            font-size: 1.1rem;
            opacity: 0.8;
        }
        
        .danwa-talk-btn .btn-status {
            max-width: 260px;
            overflow: hidden;
            text-overflow: ellipsis;
            white-space: nowrap;
        }
        
        /* State ring variations */
        .danwa-view[data-state="listening"] .zen-state-ring {
            border-color: rgba(99, 102, 241, 0.4);
            box-shadow: 0 0 60px rgba(99, 102, 241, 0.2);
            animation: stateRingListen 2s ease-in-out infinite;
        }
        
        .danwa-view[data-state="speaking"] .zen-state-ring {
            border-color: rgba(230, 57, 70, 0.3);
            box-shadow: 0 0 60px rgba(230, 57, 70, 0.15);
            animation: stateRingSpeak 1.5s ease-in-out infinite;
        }
        
        @keyframes stateRingListen {
            0%, 100% { opacity: 0.6; transform: translateX(-50%) scale(1); }
            50% { opacity: 0.9; transform: translateX(-50%) scale(0.99); }
        }
        
        @keyframes stateRingSpeak {
            0%, 100% { opacity: 0.5; transform: translateX(-50%) scale(1); }
            50% { opacity: 0.8; transform: translateX(-50%) scale(1.01); }
        }
        
        /* Mode-based variations */
        
        /* 点 ten - nodes: minimal, constellation */
        .danwa-view[data-mode="ten"] .eye::after {
            opacity: 0.3;
        }
        .danwa-view[data-mode="ten"] .zen-state-ring {
            opacity: 0.3;
        }
        
        /* 形 kata - form: default */
        
        /* 身 mi - body: deeper, stronger vignette */
        .danwa-view[data-mode="mi"] .eye::before {
            background: radial-gradient(ellipse at center, transparent 40%, rgba(0,0,0,0.75) 100%);
        }
        .danwa-view[data-mode="mi"] .eye {
            animation-duration: 6s;
        }
        .danwa-view[data-mode="mi"] .mouth-container {
            transform: scale(0.95);
        }
        
        /* 間 ma - space: reduced chrome, ambient */
        .danwa-view[data-mode="ma"] .eye {
            border-color: rgba(99, 102, 241, 0.15);
        }
        .danwa-view[data-mode="ma"] .mouth-container {
            border-color: rgba(99, 102, 241, 0.1);
        }
        .danwa-view[data-mode="ma"] .zen-state-ring {
            opacity: 0.25;
        }
        .danwa-view[data-mode="ma"]::after {
            width: 90%;
            height: 70%;
        }
        
        /* 場 ba - field: calmer, breath baseline */
        .danwa-view[data-mode="ba"] .eye::after {
            animation: none;
            opacity: 0.4;
        }
        .danwa-view[data-mode="ba"] .zen-state-ring {
            opacity: 0.2;
            animation: none;
        }
        .danwa-view[data-mode="ba"] .eye {
            animation: none;
        }
        
        .zen-thought-bubble {
            min-height: 50px;
            max-width: 600px;
            margin: 0 auto;
            text-align: center;
            color: rgba(200, 205, 220, 0.95);
            font-size: 1.05rem;
            font-family: 'Noto Sans JP', sans-serif;
            font-weight: 300;
            letter-spacing: 0.02em;
            line-height: 1.6;
            max-width: 500px;
            line-height: 1.5;
            padding: 15px 25px;
            background: rgba(230, 57, 70, 0.1);
            border: 1px solid rgba(230, 57, 70, 0.3);
            border-radius: 20px;
            margin-bottom: 20px;
            opacity: 0;
            transition: opacity 0.3s;
        }
        
        .zen-thought-bubble.active {
            opacity: 1;
        }
        
        
        /* Adjust nodes view for toggle */
        .nodes-view {
            padding-top: 60px;
        }
    </style>
</head>
<body>
    <!-- Logo -->
    <div class="zen-logo">⟁</div>
    
    <!-- Kanji Mode Selector -->
    <div class="zen-mode-selector" role="tablist" aria-label="Zen mode selector">
        <button class="zen-mode-btn" role="tab" aria-selected="false" aria-label="Nodes (Ten)" data-mode="ten" onclick="setZenMode('ten')">
            点<span class="tooltip">Nodes (TEN)</span>
        </button>
        <button class="zen-mode-btn" role="tab" aria-selected="true" aria-label="Form (Kata)" data-mode="kata" onclick="setZenMode('kata')">
            形<span class="tooltip">Form (KATA)</span>
        </button>
        <button class="zen-mode-btn" role="tab" aria-selected="false" aria-label="Body (Mi)" data-mode="mi" onclick="setZenMode('mi')">
            身<span class="tooltip">Body (MI)</span>
        </button>
        <button class="zen-mode-btn" role="tab" aria-selected="false" aria-label="Space (Ma)" data-mode="ma" onclick="setZenMode('ma')">
            間<span class="tooltip">Space (MA)</span>
        </button>
        <button class="zen-mode-btn" role="tab" aria-selected="false" aria-label="Field (Ba)" data-mode="ba" onclick="setZenMode('ba')">
            場<span class="tooltip">Field (BA)</span>
        </button>
    </div>
    
    <!-- DANWA Mode - Zen's Face -->
    <div id="danwa-view" class="danwa-view" data-state="idle" style="display: none;">
        <div class="zen-face">
            
            <!-- State Ring - binds eyes -->
            <div class="zen-state-ring"></div>
            
            <!-- Thought Bubble - Above Eyes (Brain) -->
            <div class="zen-thought-container">
                <div class="zen-thought-bubble" id="zen-thought"></div>
            </div>
            
            <!-- Eyes with Eyebrows -->
            <div class="zen-eyes-container">
                <!-- BODY (身) - Model viewer (GLB cross-platform, USDZ for iOS AR) -->
                <div class="zen-body" id="zen-body">
                    <div class="zen-body-card">
                        <div class="zen-body-header">
                            <div class="zen-body-title">Anatomy</div>
                            <div class="zen-body-actions">
                                <a class="zen-body-btn" href="/assets/Skeleton.usdz" rel="ar">Open in AR (iOS)</a>
                                <a class="zen-body-btn" href="/assets/Skeleton.usdz" download>Download USDZ</a>
                            </div>
                        </div>
                        <script type="module" src="https://unpkg.com/@google/model-viewer/dist/model-viewer.min.js"></script>
                        <model-viewer
                            id="skeleton-viewer"
                            src="/assets/AnatomyGeometry-Full-GLB.glb"
                            ios-src="/assets/Skeleton.usdz"
                            alt="Anatomy / Skeleton model"
                            camera-controls
                            autoplay
                            shadow-intensity="0.6"
                            style="width:100%; height:520px; background: rgba(0,0,0,0.35); border-radius: 14px;"
                        ></model-viewer>
                        <div class="zen-body-hint">
                            <div class="asset-status">
                                <span class="asset-pill ok" id="asset-glb"><span class="asset-dot"></span>GLB</span>
                                <span class="asset-pill ok" id="asset-usdz"><span class="asset-dot"></span>iOS AR</span>
                            </div>
                        </div>
                    </div>
                </div>
                <!-- Eyebrows - Wave-based emotion indicators -->
                <svg class="zen-eyebrows" viewBox="0 0 700 50" preserveAspectRatio="xMidYMid meet">
                    <path id="eyebrow-left" class="eyebrow" 
                          d="M 50 35 Q 130 20 210 35" 
                          stroke-linecap="round"/>
                    <path id="eyebrow-right" class="eyebrow" 
                          d="M 490 35 Q 570 20 650 35" 
                          stroke-linecap="round"/>
                </svg>
                
                <!-- Eyes - Camera Feed -->
                <div class="zen-eyes">
                    <div class="eye left-eye">
                        <div class="eye-label" title="Left eye">L</div>
                        <img id="left-eye-img" src="/proxy/me/stream" alt="Left Eye">
                    </div>
                    
                    <!-- Mood Emoji - Between Eyes -->
                    <div class="zen-emotion-indicator" id="zen-emotion-display" data-emotion="calm">
                        <span class="zen-emotion-emoji">😌</span>
                    </div>
                    
                    <div class="eye right-eye">
                        <div class="eye-label" title="Right eye">R</div>
                        <img id="right-eye-img" src="/proxy/me/stream" alt="Right Eye">
                    </div>
                </div>
            </div>
            
            <!-- Mouth - Voice Interface -->
            <div class="zen-mouth">
                <div class="mouth-container">
                    <div class="wave-clip">
                        <canvas id="danwa-waveform"></canvas>
                    </div>
                </div>
                <button id="danwa-talk-btn" class="danwa-talk-btn" onclick="toggleZenVoice()">
                    <span class="btn-icon">🎤</span>
                    <span id="danwa-status" class="btn-status">Speak</span>
                </button>
            </div>
        </div>
    </div>
    
    <!-- NODES Mode - Original Dashboard -->
    <div id="nodes-view" class="nodes-view">
        <div class="container">
            <header>
                <div class="title-kanji">恕</div>
                <h1>VOIGHT CLUSTER</h1>
            </header>
        
        <div class="summary">
            <div class="stat">
                <div class="stat-value">""" + str(total_nodes) + """</div>
                <div class="stat-label">Total Nodes</div>
            </div>
            <div class="stat">
                <div class="stat-value online">""" + str(online_count) + """</div>
                <div class="stat-label">Online</div>
            </div>
            <div class="stat">
                <div class="stat-value offline">""" + str(offline_count) + """</div>
                <div class="stat-label">Offline</div>
            </div>
        </div>
        
        <div class="section-title">Cluster Nodes</div>
        <div class="nodes">
"""
    
    for node in cluster.nodes:
        status_class = "online" if node.online else "offline"
        status_text = "Online" if node.online else "Offline"
        
        html += f"""
            <div class="node-card {status_class}">
                <div class="node-header">
                    <div class="node-kanji">{node.kanji}</div>
                    <div class="node-info">
                        <h2>{node.name}</h2>
                        <div class="node-role">{node.role}</div>
                    </div>
                </div>
                <div class="node-status">
                    <div class="status-dot {status_class}"></div>
                    <span>{status_text}</span>
                </div>
                <a href="{node.url}" target="_blank" class="node-url">{node.url}</a>
"""

        # TE quick control link (always inside the TE card)
        if node.name == "te":
            html += """
                <div class="node-details" style="padding-top:0.6rem;">
                    <div style="text-align:center;">
                        <a href="/te"
                           style="display:inline-block; padding:8px 16px; background:var(--accent); color:white;
                                  text-decoration:none; border-radius:6px; font-size:0.8rem;
                                  transition: opacity 0.2s;"
                           onmouseover="this.style.opacity='0.85'" onmouseout="this.style.opacity='1'">
                            Open TE Controls
                        </a>
                    </div>
                </div>
"""
        
        if node.online and node.status:
            # Show some details if available
            if "hardware" in node.status:
                hw = node.status["hardware"]
                gpus = hw.get("gpus_detected", 0)
                ram_total = hw.get("ram_total_gb", "?")
                ram_used = hw.get("ram_used_gb", 0)
                cpu = hw.get("cpu_percent", 0)
                
                # Helper for progress bar
                def progress_bar(percent):
                    return f'<div class="progress-bar-bg"><div class="progress-bar-fill" style="width: {percent}%"></div></div>'
                
                gpu_list = hw.get("gpus", [])
                
                html += f"""
                <div class="node-details">
                    <div class="detail-row">
                        <span class="detail-label">CPU</span>
                        <div style="display:flex; gap:10px; align-items:center">
                            {progress_bar(cpu)} <span>{cpu}%</span>
                        </div>
                    </div>
                    <div class="detail-row">
                        <span class="detail-label">RAM</span>
                        <span>{ram_used} / {ram_total} GB</span>
                    </div>
                    <div class="detail-row">
                        <span class="detail-label">GPU</span>
                        <span>{gpus} detected</span>
                    </div>
"""
                # Show GPU details - always show 2 rows for visual alignment
                for i in range(2):
                    if i < len(gpu_list):
                        gpu = gpu_list[i]
                        gpu_mem_used = gpu.get('memory_used_mb', 0)
                        gpu_mem_total = gpu.get('memory_total_mb', 0)
                        gpu_mem_pct = (gpu_mem_used / gpu_mem_total * 100) if gpu_mem_total > 0 else 0
                        html += f"""
                    <div class="detail-row" style="padding-left: 1rem;">
                        <span class="detail-label" style="font-size:0.7rem;">GPU {i}</span>
                        <div style="display:flex; gap:8px; align-items:center; font-size:0.75rem;">
                            {progress_bar(gpu_mem_pct)} <span>{gpu_mem_used}/{gpu_mem_total} MB</span>
                        </div>
                    </div>
"""
                    else:
                        # Empty placeholder row for alignment
                        html += f"""
                    <div class="detail-row" style="padding-left: 1rem; opacity: 0.3;">
                        <span class="detail-label" style="font-size:0.7rem;">GPU {i}</span>
                        <div style="display:flex; gap:8px; align-items:center; font-size:0.75rem;">
                            {progress_bar(0)} <span>—</span>
                        </div>
                    </div>
"""
                html += """
                </div>
"""

            if "capabilities" in node.status:
                caps = node.status["capabilities"]
                cap_list = [k for k, v in caps.items() if v]
                # Always show 3 capability lines for visual alignment
                html += """
                <div class="node-details">
"""
                for i in range(3):
                    if i < len(cap_list):
                        cap_name = cap_list[i].replace('_', ' ').title()
                        html += f"""
                    <div class="detail-row">
                        <span class="detail-label">{"Capabilities" if i == 0 else ""}</span>
                        <span class="detail-value">{cap_name}</span>
                    </div>
"""
                    else:
                        html += f"""
                    <div class="detail-row" style="opacity: 0.3;">
                        <span class="detail-label">{"Capabilities" if i == 0 else ""}</span>
                        <span class="detail-value">—</span>
                    </div>
"""
                html += """
                </div>
"""
            
            # Show sensor status for ME node (3D Camera, Lidar)
            if node.name == "me":
                hw = node.status.get("hardware", {})
                cameras = hw.get("cameras_detected", 0)
                lidar_connected = node.status.get("lidar", {}).get("connected", False) if "lidar" in node.status else False
                
                cam_class = "online" if cameras > 0 else "offline"
                lidar_class = "online" if lidar_connected else "offline"
                
                html += f"""
                <div class="node-details">
                    <div class="detail-row">
                        <span class="detail-label">3D Camera</span>
                        <div style="display:flex; align-items:center; gap:6px;">
                            <div class="status-dot {cam_class}" style="width:8px;height:8px;"></div>
                            <span>{"Connected" if cameras > 0 else "Disconnected"}</span>
                        </div>
                    </div>
                    <div class="detail-row">
                        <span class="detail-label">Lidar</span>
                        <div style="display:flex; align-items:center; gap:6px;">
                            <div class="status-dot {lidar_class}" style="width:8px;height:8px;"></div>
                            <span>{"Connected" if lidar_connected else "Disconnected"}</span>
                        </div>
                    </div>
                </div>
"""
            
            # Show camera feed if available
            if "capabilities" in node.status and node.status["capabilities"].get("camera_array", False):
                # Use HTTPS-safe proxy for MJPEG stream (avoids mixed content on https://kokoro.local)
                stream_url = "/proxy/me/stream" if node.name == "me" else f"http://{node.url.split('//')[1].split(':')[0]}:8028/stream"
                html += f"""
                <div class="node-details">
                    <div class="detail-row" style="flex-direction: column; align-items: flex-start; gap: 0.5rem;">
                        <span class="detail-label">Eyes</span>
                        <div class="stereo-thumbs">
                            <div class="stereo-eye left">
                                <div class="pane-badge" title="Left eye">L</div>
                                <img src="{stream_url}" alt="Left eye stream"
                                     onerror="this.style.display='none'; this.parentElement.classList.add('offline');">
                            </div>
                            <div class="stereo-eye right">
                                <div class="pane-badge" title="Right eye">R</div>
                                <img src="{stream_url}" alt="Right eye stream"
                                     onerror="this.style.display='none'; this.parentElement.classList.add('offline');">
                            </div>
                        </div>
                    </div>
                </div>
"""
            
            # Show voice UI for KOKORO node
            if node.name == "kokoro" and node.status.get("voice_ui", {}).get("enabled", False):
                voice_url = node.status.get("voice_ui", {}).get("url", "http://kokoro.local:3000")
                html += f"""
                <div class="node-details">
                    <div class="detail-row" style="flex-direction: column; align-items: flex-start; gap: 0.5rem;">
                        <span class="detail-label">🎙️ Zen Voice Interface</span>
                        
                        <!-- Voice Visualizer Canvas -->
                        <div id="zen-voice-container" style="width: 100%; background: linear-gradient(180deg, #0a0a0a 0%, #111 100%); 
                                border-radius: 12px; border: 1px solid var(--border); padding: 20px; position: relative; overflow: hidden;">
                            
                            <!-- Ambient glow effect -->
                            <div id="zen-glow" style="position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%);
                                    width: 200px; height: 200px; background: radial-gradient(circle, rgba(230,57,70,0.15) 0%, transparent 70%);
                                    border-radius: 50%; pointer-events: none; transition: all 0.3s;"></div>
                            
                            <!-- Main waveform canvas -->
                            <canvas id="zen-waveform" width="600" height="120" style="width: 100%; height: 120px; display: block;"></canvas>
                            
                            <!-- Status text -->
                            <div id="zen-status" style="text-align: center; margin-top: 12px; font-size: 0.8rem; color: var(--text-secondary);
                                    letter-spacing: 2px; text-transform: uppercase;">
                                Click to activate voice
                            </div>
                            
                            <!-- Listening indicator -->
                            <div id="zen-listening" style="display: none; text-align: center; margin-top: 8px;">
                                <span style="display: inline-block; width: 8px; height: 8px; background: var(--accent); 
                                        border-radius: 50%; animation: pulse 1.5s infinite;"></span>
                                <span style="margin-left: 8px; color: var(--accent); font-size: 0.75rem;">LISTENING</span>
                            </div>
                        </div>
                        
                        <!-- Control buttons -->
                        <div style="display: flex; gap: 10px; width: 100%; justify-content: center; margin-top: 8px;">
                            <button id="zen-talk-btn" onclick="toggleZenVoice()" aria-label="Talk to Zen"
                                    style="padding: 10px 16px; background: var(--accent); color: white; border: none; 
                                           border-radius: 999px; cursor: pointer; font-size: 0.95rem; font-weight: 600;
                                           transition: all 0.2s; display: inline-flex; align-items: center; justify-content:center; gap: 0;">
                                <span style="font-size: 1.15rem; line-height: 1;">🎤</span>
                            </button>
                            <a href="{voice_url}" target="_blank" 
                               style="padding: 10px 16px; background: transparent; color: var(--text-secondary); 
                                      border: 1px solid var(--border); border-radius: 6px; text-decoration: none;
                                      font-size: 0.8rem; transition: all 0.2s; display: flex; align-items: center;"
                               onmouseover="this.style.borderColor='var(--accent)'; this.style.color='var(--accent)'" 
                               onmouseout="this.style.borderColor='var(--border)'; this.style.color='var(--text-secondary)'">
                                ↗
                            </a>
                        </div>
                    </div>
                </div>
                
                <style>
                    @keyframes pulse {{
                        0%, 100% {{ opacity: 1; transform: scale(1); }}
                        50% {{ opacity: 0.5; transform: scale(1.2); }}
                    }}
                </style>
                
                <script>
                    // Zen Voice Visualizer - Voice Reactive
                    (function() {{
                        const canvas = document.getElementById('zen-waveform');
                        const glow = document.getElementById('zen-glow');
                        if (!canvas) return;
                        
                        // Retina scaling
                        const rect = canvas.getBoundingClientRect();
                        canvas.width = rect.width * 2;
                        canvas.height = rect.height * 2;
                        const ctx = canvas.getContext('2d');
                        ctx.scale(2, 2);
                        
                        const width = rect.width;
                        const height = rect.height;
                        let phase = 0;
                        
                        // Audio analysis for voice reactivity
                        let analyser = null;
                        let dataArray = null;
                        
                        // Expose function to connect audio
                        window.zenConnectAudio = function(stream) {{
                            const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
                            analyser = audioCtx.createAnalyser();
                            analyser.fftSize = 2048;
                            const source = audioCtx.createMediaStreamSource(stream);
                            source.connect(analyser);
                            dataArray = new Uint8Array(analyser.frequencyBinCount);
                        }};
                        
                        window.zenDisconnectAudio = function() {{
                            analyser = null;
                            dataArray = null;
                        }};
                        
                        function animate() {{
                            ctx.fillStyle = 'rgba(10, 10, 10, 0.25)';
                            ctx.fillRect(0, 0, width, height);
                            
                            const centerY = height / 2;
                            
                            if (analyser && dataArray) {{
                                // Voice-reactive mode
                                analyser.getByteTimeDomainData(dataArray);
                                
                                // Calculate amplitude
                                let sum = 0;
                                for (let i = 0; i < dataArray.length; i++) {{
                                    sum += Math.abs(dataArray[i] - 128);
                                }}
                                const avgAmp = sum / dataArray.length;
                                
                                // Glow reacts to voice
                                if (glow) {{
                                    const glowSize = 150 + avgAmp * 4;
                                    const glowOpacity = 0.15 + avgAmp * 0.008;
                                    glow.style.width = glowSize + 'px';
                                    glow.style.height = glowSize + 'px';
                                    glow.style.background = `radial-gradient(circle, rgba(230,57,70,${{glowOpacity}}) 0%, transparent 70%)`;
                                }}
                                
                                // Draw actual waveform
                                ctx.beginPath();
                                const gradient = ctx.createLinearGradient(0, 0, width, 0);
                                gradient.addColorStop(0, 'rgba(230, 57, 70, 0.3)');
                                gradient.addColorStop(0.5, 'rgba(230, 57, 70, 1)');
                                gradient.addColorStop(1, 'rgba(230, 57, 70, 0.3)');
                                ctx.strokeStyle = gradient;
                                ctx.lineWidth = 2.5;
                                
                                const sliceWidth = width / dataArray.length;
                                let x = 0;
                                for (let i = 0; i < dataArray.length; i++) {{
                                    const v = dataArray[i] / 128.0;
                                    const y = (v * height) / 2;
                                    if (i === 0) ctx.moveTo(x, y);
                                    else ctx.lineTo(x, y);
                                    x += sliceWidth;
                                }}
                                ctx.stroke();
                                
                                // Glow trail
                                ctx.beginPath();
                                ctx.strokeStyle = 'rgba(230, 57, 70, 0.15)';
                                ctx.lineWidth = 8;
                                x = 0;
                                for (let i = 0; i < dataArray.length; i++) {{
                                    const v = dataArray[i] / 128.0;
                                    const y = (v * height) / 2;
                                    if (i === 0) ctx.moveTo(x, y);
                                    else ctx.lineTo(x, y);
                                    x += sliceWidth;
                                }}
                                ctx.stroke();
                                
                            }} else {{
                                // Idle animation mode
                                const baseAmplitude = 18 + Math.sin(phase * 0.5) * 6;
                                
                                // Primary wave
                                ctx.beginPath();
                                ctx.strokeStyle = 'rgba(230, 57, 70, 0.7)';
                                ctx.lineWidth = 2.5;
                                for (let x = 0; x < width; x++) {{
                                    const y = centerY 
                                        + Math.sin(x * 0.018 + phase) * baseAmplitude
                                        + Math.sin(x * 0.045 + phase * 1.3) * (baseAmplitude * 0.35);
                                    if (x === 0) ctx.moveTo(x, y);
                                    else ctx.lineTo(x, y);
                                }}
                                ctx.stroke();
                                
                                // Secondary wave
                                ctx.beginPath();
                                ctx.strokeStyle = 'rgba(230, 57, 70, 0.25)';
                                ctx.lineWidth = 1.5;
                                for (let x = 0; x < width; x++) {{
                                    const y = centerY + Math.sin(x * 0.012 - phase * 0.7) * (baseAmplitude * 0.6);
                                    if (x === 0) ctx.moveTo(x, y);
                                    else ctx.lineTo(x, y);
                                }}
                                ctx.stroke();
                                
                                // Glow pulse
                                if (glow) {{
                                    const glowSize = 180 + Math.sin(phase * 0.8) * 20;
                                    glow.style.width = glowSize + 'px';
                                    glow.style.height = glowSize + 'px';
                                }}
                            }}
                            
                            phase += 0.035;
                            requestAnimationFrame(animate);
                        }}
                        animate();
                    }})();
                </script>
"""
            
            # Show robot arm status for TE node
            if "arm" in node.status:
                arm = node.status["arm"]
                arm_connected = arm.get("connected", False)
                arm_enabled = arm.get("enabled", False)
                joints = arm.get("joints", [90, 90, 90, 90, 90, 90])
                joint_names = ["Base", "Shoulder", "Elbow", "Wrist↕", "Wrist↻", "Grip"]
                
                conn_class = "online" if arm_connected else "offline"
                ena_class = "online" if arm_enabled else "offline"
                
                html += f"""
                <div class="node-details">
                    <div class="detail-row">
                        <span class="detail-label">Arduino</span>
                        <div style="display:flex; align-items:center; gap:6px;">
                            <div class="status-dot {conn_class}" style="width:8px;height:8px;"></div>
                            <span>{"Connected" if arm_connected else "Disconnected"}</span>
                        </div>
                    </div>
                    <div class="detail-row">
                        <span class="detail-label">Servos</span>
                        <div style="display:flex; align-items:center; gap:6px;">
                            <div class="status-dot {ena_class}" style="width:8px;height:8px;"></div>
                            <span>{"Enabled" if arm_enabled else "Disabled"}</span>
                        </div>
                    </div>
                </div>
"""
                # LiDAR status on TE (USB-connected)
                if node.status.get("lidar") is not None:
                    lidar_connected = bool(node.status.get("lidar", {}).get("connected", False))
                    lidar_port = node.status.get("lidar", {}).get("port") or "—"
                    lidar_class = "online" if lidar_connected else "offline"
                    html += f"""
                <div class="node-details" style="padding-top:0.5rem;">
                    <div class="detail-row">
                        <span class="detail-label">Lidar</span>
                        <div style="display:flex; align-items:center; gap:6px;">
                            <div class="status-dot {lidar_class}" style="width:8px;height:8px;"></div>
                            <span>{"Connected" if lidar_connected else "Disconnected"}</span>
                            <span style="color: var(--text-secondary); font-size: 0.75rem; margin-left: 6px;">{lidar_port}</span>
                        </div>
                    </div>
                </div>
                <div class="node-details" style="padding-top:0.75rem;">
                    <div style="display:grid; grid-template-columns: repeat(3, 1fr); gap: 8px;">
"""
                for i, (name, angle) in enumerate(zip(joint_names, joints)):
                    # Color based on position (green at center, yellow at extremes)
                    deviation = abs(angle - 90) / 90
                    if deviation < 0.3:
                        bar_color = "var(--online)"
                    elif deviation < 0.6:
                        bar_color = "#f39c12"
                    else:
                        bar_color = "var(--accent)"
                    
                    html += f"""
                        <div id="joint-{i}" style="text-align:center; padding:4px; background:rgba(255,255,255,0.03); border-radius:4px;">
                            <div style="font-size:0.65rem; color:var(--text-secondary); margin-bottom:2px;">{name}</div>
                            <div id="joint-val-{i}" style="font-size:1rem; font-weight:600; color:{bar_color};">{int(angle)}°</div>
                            <div style="height:3px; background:#333; border-radius:2px; margin-top:3px; overflow:hidden;">
                                <div id="joint-bar-{i}" style="height:100%; width:{angle/180*100}%; background:{bar_color}; border-radius:2px; transition: width 0.2s;"></div>
                            </div>
                        </div>
"""
                html += """
                    </div>
                </div>
"""

                # Inline TE controls inside the TE node card (no need to open /te)
                html += """
                <div class="node-details" style="padding-top:0.6rem;">
                  <details class="te-inline-controls">
                    <summary>
                      <span>Controls</span>
                      <span style="opacity:0.6">Sliders</span>
                    </summary>
                    <div class="te-inline-body">
                      <div class="te-inline-grid">
                        <div class="te-inline-servo">
                          <div class="te-inline-head"><span class="te-inline-name">Base</span><span class="te-inline-val" id="te-inline-val-0">90°</span></div>
                          <input id="te-inline-slider-0" type="range" min="0" max="180" value="90">
                        </div>
                        <div class="te-inline-servo">
                          <div class="te-inline-head"><span class="te-inline-name">Shoulder</span><span class="te-inline-val" id="te-inline-val-1">90°</span></div>
                          <input id="te-inline-slider-1" type="range" min="0" max="180" value="90">
                        </div>
                        <div class="te-inline-servo">
                          <div class="te-inline-head"><span class="te-inline-name">Elbow</span><span class="te-inline-val" id="te-inline-val-2">90°</span></div>
                          <input id="te-inline-slider-2" type="range" min="0" max="180" value="90">
                        </div>
                        <div class="te-inline-servo">
                          <div class="te-inline-head"><span class="te-inline-name">Wrist↕</span><span class="te-inline-val" id="te-inline-val-3">90°</span></div>
                          <input id="te-inline-slider-3" type="range" min="0" max="180" value="90">
                        </div>
                        <div class="te-inline-servo">
                          <div class="te-inline-head"><span class="te-inline-name">Wrist↻</span><span class="te-inline-val" id="te-inline-val-4">90°</span></div>
                          <input id="te-inline-slider-4" type="range" min="0" max="180" value="90">
                        </div>
                        <div class="te-inline-servo">
                          <div class="te-inline-head"><span class="te-inline-name">Grip</span><span class="te-inline-val" id="te-inline-val-5">90°</span></div>
                          <input id="te-inline-slider-5" type="range" min="0" max="180" value="90">
                        </div>
                      </div>
                      <div class="te-inline-actions">
                        <button class="te-inline-btn primary" id="te-inline-move" type="button">Move</button>
                        <button class="te-inline-btn" id="te-inline-home" type="button">Home</button>
                      </div>
                      <div style="margin-top:8px; color: rgba(235,238,255,0.55); font-size: 0.75rem;">
                        Uses HTTPS-safe proxies: <code>/proxy/te/arm</code>, <code>/proxy/te/move</code>, <code>/proxy/te/home</code>.
                      </div>
                    </div>
                  </details>
                </div>
"""
        
        if node.error:
            html += f"""
                <div class="error-msg">{node.error}</div>
"""
        
        html += """
            </div>
"""

    html += """
        </div>
"""

    if discovered:
        html += f"""
        <details class="discovered-panel">
            <summary>
                <div class="discovered-title">Discovered Services</div>
                <div class="discovered-meta">
                    <span>{len(discovered)} found</span>
                    <span class="discovered-caret"></span>
                </div>
            </summary>
            <div class="discovered-body">
                <div class="nodes">
"""
        for node in discovered:
            html += f"""
            <div class="node-card online">
                <div class="node-header">
                    <div class="node-kanji">{node.kanji}</div>
                    <div class="node-info">
                        <h2>{node.name}</h2>
                        <div class="node-role">{node.role}</div>
                    </div>
                </div>
                <div class="node-status">
                    <div class="status-dot online"></div>
                    <span>Online</span>
                </div>
                <a href="{node.url}" target="_blank" class="node-url">{node.url}</a>
                <div class="node-details">
                    <div class="detail-row">
                        <span class="detail-label">Hostname</span>
                        <span>{node.status.get("hostname", "?")}</span>
                    </div>
                </div>
            </div>
"""
        html += """
                </div>
            </div>
        </details>
"""

    html += """
        <footer>
            <p>KOKORO Orchestration Node • v0.1.1</p>
            <p class="refresh-note">Real-time dashboard</p>
        </footer>
    </div>
    
    <script>
        // Real-time TE arm updates
        const jointNames = ['Base', 'Shoulder', 'Elbow', 'Wrist↕', 'Wrist↻', 'Grip'];
        
        function getColor(angle) {
            const deviation = Math.abs(angle - 90) / 90;
            if (deviation < 0.3) return 'var(--online)';
            if (deviation < 0.6) return '#f39c12';
            return 'var(--accent)';
        }
        
        let teAvailable = true;
        let teRetryCount = 0;
        const teInline = {
            inited: false,
            dragging: new Set(),
            lastUserEdit: new Map(), // idx -> ts
            holdMs: 8000
        };

        function initTeInlineControls() {
            if (teInline.inited) return;
            const moveBtn = document.getElementById('te-inline-move');
            const homeBtn = document.getElementById('te-inline-home');
            if (!moveBtn) return; // controls not present (e.g. TE offline or arm section not rendered)

            for (let i = 0; i < 6; i++) {
                const slider = document.getElementById('te-inline-slider-' + i);
                const val = document.getElementById('te-inline-val-' + i);
                if (!slider || !val) continue;

                const mark = () => teInline.dragging.add(i);
                const unmark = () => teInline.dragging.delete(i);
                slider.addEventListener('pointerdown', mark);
                slider.addEventListener('pointerup', unmark);
                slider.addEventListener('pointercancel', unmark);
                slider.addEventListener('mousedown', mark);
                slider.addEventListener('mouseup', unmark);
                slider.addEventListener('touchstart', mark, { passive: true });
                slider.addEventListener('touchend', unmark, { passive: true });
                slider.addEventListener('touchcancel', unmark, { passive: true });

                slider.addEventListener('input', (e) => {
                    const a = parseInt(e.target.value, 10);
                    if (!Number.isFinite(a)) return;
                    teInline.lastUserEdit.set(i, Date.now());
                    val.textContent = a + '°';
                });
            }

            async function post(url, body) {
                const r = await fetch(url, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: body ? JSON.stringify(body) : null,
                });
                if (!r.ok) throw new Error('HTTP ' + r.status);
                return await r.json();
            }

            moveBtn.addEventListener('click', async () => {
                const commands = [];
                for (let i = 0; i < 6; i++) {
                    const slider = document.getElementById('te-inline-slider-' + i);
                    if (!slider) continue;
                    const a = parseInt(slider.value, 10);
                    commands.push({ servo_id: i, angle: Number.isFinite(a) ? a : 90, speed: 50 });
                }
                moveBtn.disabled = true;
                try {
                    await post('/proxy/te/move', { job_id: 'dash-' + Date.now(), commands });
                } finally {
                    moveBtn.disabled = false;
                }
            });

            homeBtn.addEventListener('click', async () => {
                homeBtn.disabled = true;
                try { await fetch('/proxy/te/home', { method: 'POST' }); } catch (e) {}
                finally { homeBtn.disabled = false; }
            });

            teInline.inited = true;
        }
        
        async function updateTeArm() {
            if (!teAvailable && teRetryCount < 10) {
                teRetryCount++;
                return; // Skip if TE was unavailable, retry every 5 seconds
            }
            teRetryCount = 0;
            
            try {
                const controller = new AbortController();
                const timeout = setTimeout(() => controller.abort(), 2000);
                
                const response = await fetch('/proxy/te/arm', {
                    signal: controller.signal,
                    cache: 'no-store'
                });
                clearTimeout(timeout);
                
                if (!response.ok) { teAvailable = false; return; }
                
                const data = await response.json();
                teAvailable = true;
                
                const joints = (data && data.joints) ? data.joints
                    : (data && data.servos) ? data.servos.map(s => s.angle)
                    : null;

                if (joints) {
                    joints.forEach((angle, i) => {
                        const valEl = document.getElementById('joint-val-' + i);
                        const barEl = document.getElementById('joint-bar-' + i);
                        if (valEl && barEl) {
                            const color = getColor(angle);
                            valEl.textContent = Math.round(angle) + '°';
                            valEl.style.color = color;
                            barEl.style.width = (angle / 180 * 100) + '%';
                            barEl.style.background = color;
                        }
                    });

                    // Update inline TE sliders if present (without snap-back while dragging/recently edited)
                    initTeInlineControls();
                    const now = Date.now();
                    for (let i = 0; i < Math.min(6, joints.length); i++) {
                        const slider = document.getElementById('te-inline-slider-' + i);
                        const val = document.getElementById('te-inline-val-' + i);
                        if (!slider || !val) continue;

                        const lastEdit = teInline.lastUserEdit.get(i) || 0;
                        const hold = (now - lastEdit) < teInline.holdMs;
                        const dragging = teInline.dragging.has(i);

                        const serverAngle = Math.round(Number(joints[i] ?? 90));
                        if (!dragging && !hold) {
                            slider.value = String(serverAngle);
                            val.textContent = serverAngle + '°';
                        } else {
                            const localAngle = parseInt(slider.value, 10);
                            val.textContent = (Number.isFinite(localAngle) ? localAngle : 90) + '°';
                        }
                    }
                }
            } catch (e) {
                teAvailable = false;
                // Silent fail
            }
        }
        
        // Update every 500ms
        setInterval(updateTeArm, 500);
    </script>
    </div> <!-- Close nodes-view -->
    
    <script>
        // Zen State, Mode & Emotion Management
        window.zenState = 'idle'; // idle | listening | thinking | speaking
        window.zenMode = 'kata'; // ten | kata | mi | ma | ba
        window.zenEmotion = 'calm'; // calm | curious | concern | focus | joy | confused | alert
        
        // Eyebrow Wave Parameters (current interpolated values)
        window.eyebrowParams = {
            amplitude: 10,
            baselineOffset: 0,
            innerBias: 0,
            outerBias: 0,
            frequency: 0.02,
            motionSpeed: 0.02,
            symmetry: 1.0,
            phase: 0
        };
        
        // Target parameters based on emotion
        const emotionPresets = {
            // More pronounced defaults (still smooth, no jitter)
            calm:     { amplitude: 8,  baselineOffset: 0,  innerBias: 0,   outerBias: 0,   frequency: 0.015, motionSpeed: 0.015, symmetry: 1.0 },
            curious:  { amplitude: 14, baselineOffset: 5,  innerBias: 0,   outerBias: 4,   frequency: 0.018, motionSpeed: 0.02,  symmetry: 0.95 },
            concern:  { amplitude: 7,  baselineOffset: -2, innerBias: -5,  outerBias: 3,   frequency: 0.02,  motionSpeed: 0.01,  symmetry: 0.98 },
            focus:    { amplitude: 5,  baselineOffset: -4, innerBias: 0,   outerBias: -2,  frequency: 0.03,  motionSpeed: 0.008, symmetry: 1.0 },
            joy:      { amplitude: 11, baselineOffset: 4,  innerBias: 3,   outerBias: 3,   frequency: 0.012, motionSpeed: 0.018, symmetry: 1.0 },
            confused: { amplitude: 9,  baselineOffset: 1,  innerBias: -2,  outerBias: 3,   frequency: 0.02,  motionSpeed: 0.012, symmetry: 0.7 },
            alert:    { amplitude: 16, baselineOffset: 6,  innerBias: 0,   outerBias: 0,   frequency: 0.035, motionSpeed: 0.05,  symmetry: 1.0 }
        };
        
        function setZenEmotion(emotion) {
            window.zenEmotion = emotion;
            
            // Emoji mapping for emotions
            const emotionEmojis = {
                calm: '😌',
                curious: '🤔',
                concern: '😟',
                focus: '🧐',
                joy: '😊',
                confused: '😕',
                alert: '😮'
            };
            
            // Update emotion display
            const emotionDisplay = document.getElementById('zen-emotion-display');
            if (emotionDisplay) {
                emotionDisplay.setAttribute('data-emotion', emotion);
                const emoji = emotionDisplay.querySelector('.zen-emotion-emoji');
                if (emoji) emoji.textContent = emotionEmojis[emotion] || '😌';
            }
        }
        
        // Interpolate eyebrow parameters toward target
        function updateEyebrowParams() {
            const target = emotionPresets[window.zenEmotion] || emotionPresets.calm;
            const lerp = 0.03; // Smooth interpolation
            
            for (const key in target) {
                window.eyebrowParams[key] += (target[key] - window.eyebrowParams[key]) * lerp;
            }
            window.eyebrowParams.phase += window.eyebrowParams.motionSpeed;
        }
        
        // Generate eyebrow wave path
        function generateEyebrowPath(isLeft, width, centerY) {
            const p = window.eyebrowParams;
            const points = [];
            const segments = 40;
            
            // Asymmetry offset for non-symmetric emotions
            const asymOffset = isLeft ? (1 - p.symmetry) * 4 : -(1 - p.symmetry) * 4;
            
            for (let i = 0; i <= segments; i++) {
                const t = i / segments;
                const x = t * width;
                
                // Position along eyebrow (0 = inner, 1 = outer)
                const innerWeight = 1 - t;
                const outerWeight = t;
                
                // Bias contribution
                const bias = p.innerBias * innerWeight + p.outerBias * outerWeight;
                
                // Wave motion
                const wave = Math.sin(t * Math.PI * 2 * p.frequency * 100 + p.phase) * p.amplitude * 0.3;
                
                // Arch shape (natural brow curve)
                const arch = Math.sin(t * Math.PI) * p.amplitude;
                
                // Combine all
                const y = centerY + p.baselineOffset + bias + arch + wave + asymOffset;
                
                points.push({ x, y });
            }
            
            // Build SVG path
            let d = `M ${points[0].x} ${points[0].y}`;
            for (let i = 1; i < points.length; i++) {
                d += ` L ${points[i].x} ${points[i].y}`;
            }
            
            return d;
        }
        
        // Render eyebrows
        let eyebrowsAnimating = false;
        
        function renderEyebrows() {
            if (eyebrowsAnimating) return;
            eyebrowsAnimating = true;
            
            function animate() {
                const leftPath = document.getElementById('eyebrow-left');
                const rightPath = document.getElementById('eyebrow-right');
                const danwaView = document.getElementById('danwa-view');
                
                if (!leftPath || !rightPath) {
                    eyebrowsAnimating = false;
                    return;
                }
                
                if (danwaView && danwaView.style.display === 'none') {
                    eyebrowsAnimating = false;
                    return;
                }
                
                // Check reduced motion preference
                const reducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
                if (reducedMotion) {
                    leftPath.setAttribute('d', 'M 50 35 Q 130 22 210 35');
                    rightPath.setAttribute('d', 'M 490 35 Q 570 22 650 35');
                    return;
                }
                
                updateEyebrowParams();
                const p = window.eyebrowParams;
                
                // Left eyebrow: 50-210 (outer to inner toward center)
                // Right eyebrow: 490-650 (inner to outer from center)
                
                const baseY = 35;
                const leftPoints = [];
                const rightPoints = [];
                const segments = 24;
                
                for (let i = 0; i <= segments; i++) {
                    const t = i / segments;
                    
                    // Left eyebrow
                    const lx = 50 + t * 160;
                    // Wave term scales with emotion amplitude (kept subtle vs arch)
                    const wave = Math.sin(t * Math.PI * 2 + p.phase) * (1.6 + (p.amplitude * 0.12));
                    // Arch is the main expressive shape
                    const arch = Math.sin(t * Math.PI) * (p.amplitude * 1.18);
                    // inner = right side of left brow (t=1), outer = left side (t=0)
                    const bias = (p.outerBias * (1-t) + p.innerBias * t) * 1.25;
                    const asym = (1 - p.symmetry) * 4 * (t - 0.5);
                    const ly = baseY - p.baselineOffset - arch - bias + wave + asym;
                    leftPoints.push({x: lx, y: Math.max(3, Math.min(48, ly))});
                    
                    // Right eyebrow (mirrored: inner at t=0, outer at t=1)
                    const rx = 490 + t * 160;
                    const rBias = (p.innerBias * (1-t) + p.outerBias * t) * 1.25;
                    const rAsym = (1 - p.symmetry) * 4 * (0.5 - t);
                    const ry = baseY - p.baselineOffset - arch - rBias + wave + rAsym;
                    rightPoints.push({x: rx, y: Math.max(3, Math.min(48, ry))});
                }
                
                // Build smooth quadratic bezier path
                function pointsToSmoothPath(pts) {
                    if (pts.length < 2) return '';
                    let d = `M ${pts[0].x.toFixed(1)} ${pts[0].y.toFixed(1)}`;
                    
                    // Use quadratic curves for smoothness
                    for (let i = 1; i < pts.length - 1; i++) {
                        const cp = pts[i];
                        const next = pts[i + 1];
                        const midX = (cp.x + next.x) / 2;
                        const midY = (cp.y + next.y) / 2;
                        d += ` Q ${cp.x.toFixed(1)} ${cp.y.toFixed(1)} ${midX.toFixed(1)} ${midY.toFixed(1)}`;
                    }
                    
                    // Final point
                    const last = pts[pts.length - 1];
                    d += ` L ${last.x.toFixed(1)} ${last.y.toFixed(1)}`;
                    
                    return d;
                }
                
                leftPath.setAttribute('d', pointsToSmoothPath(leftPoints));
                rightPath.setAttribute('d', pointsToSmoothPath(rightPoints));
                
                requestAnimationFrame(animate);
            }
            animate();
        }
        
        function setZenState(state) {
            window.zenState = state;
            const danwaView = document.getElementById('danwa-view');
            if (danwaView) {
                danwaView.setAttribute('data-state', state);
            }
            
            // Map state to emotion
            const stateEmotions = {
                idle: 'calm',
                listening: 'curious',
                thinking: 'focus',
                speaking: 'joy'
            };
            setZenEmotion(stateEmotions[state] || 'calm');
        }
        
        function setZenMode(mode) {
            window.zenMode = mode;
            const danwaView = document.getElementById('danwa-view');
            const nodesView = document.getElementById('nodes-view');
            
            // Update selector UI
            document.querySelectorAll('.zen-mode-btn').forEach(btn => {
                btn.setAttribute('aria-selected', btn.dataset.mode === mode ? 'true' : 'false');
            });
            
            // Set mode attribute
            if (danwaView) danwaView.setAttribute('data-mode', mode);
            
            // Switch view based on mode
            if (mode === 'ten') {
                // ten = nodes view
                if (nodesView) nodesView.style.display = 'block';
                if (danwaView) danwaView.style.display = 'none';
            } else {
                // Other modes = danwa/face view
                if (nodesView) nodesView.style.display = 'none';
                if (danwaView) {
                    danwaView.style.display = 'flex';
                    setZenState('idle');
                    initDanwaWaveform();
                    renderEyebrows(); // Start eyebrow animation
                    applyZenStereo(); // Ensure L/R stereo split is active
                }
            }

            // BODY (身) - asset indicators
            if (mode === 'mi') {
                const glbPill = document.getElementById('asset-glb');
                const usdzPill = document.getElementById('asset-usdz');

                async function checkAsset(url, pill) {
                    if (!pill) return;
                    try {
                        const r = await fetch(url, { method: 'HEAD', cache: 'no-store' });
                        if (r.ok) pill.classList.add('ok');
                        else pill.classList.remove('ok');
                    } catch (e) {
                        pill.classList.remove('ok');
                    }
                }

                // default to red until proven OK
                if (glbPill) glbPill.classList.remove('ok');
                if (usdzPill) usdzPill.classList.remove('ok');
                checkAsset('/assets/AnatomyGeometry-Full-GLB.glb', glbPill);
                checkAsset('/assets/Skeleton.usdz', usdzPill);
            }
        }
        
        // Legacy view switching (for compatibility)
        function switchView(view) {
            if (view === 'nodes') setZenMode('ten');
            else setZenMode('kata');
        }
        
        // DANWA waveform animation with voice reactivity
        let danwaAnimating = false;
        let danwaAnalyser = null;
        let danwaDataArray = null;
        
        // Connect audio to DANWA waveform
        window.zenConnectDanwaAudio = function(stream) {
            const audioCtx = new (window.AudioContext || window.webkitAudioContext)();
            danwaAnalyser = audioCtx.createAnalyser();
            danwaAnalyser.fftSize = 2048;
            const source = audioCtx.createMediaStreamSource(stream);
            source.connect(danwaAnalyser);
            danwaDataArray = new Uint8Array(danwaAnalyser.frequencyBinCount);
        };
        
        window.zenDisconnectDanwaAudio = function() {
            danwaAnalyser = null;
            danwaDataArray = null;
        };
        
        function initDanwaWaveform() {
            if (danwaAnimating) return;
            danwaAnimating = true;
            
            const canvas = document.getElementById('danwa-waveform');
            if (!canvas) return;
            
            // Size canvas to container
            const container = canvas.parentElement;
            const rect = container.getBoundingClientRect();
            canvas.width = rect.width * 2; // Retina
            canvas.height = rect.height * 2;
            
            const ctx = canvas.getContext('2d');
            ctx.scale(2, 2); // Retina scale
            
            const width = rect.width;
            const height = rect.height;
            let phase = 0;
            
            function animateMouth() {
                ctx.fillStyle = 'rgba(5, 5, 5, 0.3)';
                ctx.fillRect(0, 0, width, height);
                
                const centerY = height / 2;
                
                const state = window.zenState || 'idle';
                
                if (state === 'listening' && danwaAnalyser && danwaDataArray) {
                    // Listening - voice-reactive with indigo
                    danwaAnalyser.getByteTimeDomainData(danwaDataArray);
                    
                    ctx.beginPath();
                    const gradient = ctx.createLinearGradient(0, 0, width, 0);
                    gradient.addColorStop(0, 'rgba(99, 102, 241, 0.3)');
                    gradient.addColorStop(0.5, 'rgba(99, 102, 241, 1)');
                    gradient.addColorStop(1, 'rgba(99, 102, 241, 0.3)');
                    ctx.strokeStyle = gradient;
                    ctx.lineWidth = 3;
                    
                    const sliceWidth = width / danwaDataArray.length;
                    let x = 0;
                    for (let i = 0; i < danwaDataArray.length; i++) {
                        const v = danwaDataArray[i] / 128.0;
                        const y = (v * height) / 2;
                        if (i === 0) ctx.moveTo(x, y);
                        else ctx.lineTo(x, y);
                        x += sliceWidth;
                    }
                    ctx.stroke();
                    
                    // Glow
                    ctx.beginPath();
                    ctx.strokeStyle = 'rgba(99, 102, 241, 0.15)';
                    ctx.lineWidth = 10;
                    x = 0;
                    for (let i = 0; i < danwaDataArray.length; i++) {
                        const v = danwaDataArray[i] / 128.0;
                        const y = (v * height) / 2;
                        if (i === 0) ctx.moveTo(x, y);
                        else ctx.lineTo(x, y);
                        x += sliceWidth;
                    }
                    ctx.stroke();
                    
                } else if (state === 'thinking') {
                    // Thinking - dampened with shimmer
                    const amp = 3 + Math.sin(phase * 2) * 2;
                    const shimmer = Math.random() * 2 - 1;
                    
                    ctx.beginPath();
                    ctx.strokeStyle = 'rgba(99, 102, 241, 0.5)';
                    ctx.lineWidth = 2;
                    for (let x = 0; x < width; x++) {
                        const y = centerY + Math.sin(x * 0.03 + phase) * amp + shimmer;
                        if (x === 0) ctx.moveTo(x, y);
                        else ctx.lineTo(x, y);
                    }
                    ctx.stroke();
                    
                } else if (state === 'speaking') {
                    // Speaking - rounder, larger amplitude, red
                    const amp = 25 + Math.sin(phase * 3) * 10;
                    
                    ctx.beginPath();
                    const gradient = ctx.createLinearGradient(0, 0, width, 0);
                    gradient.addColorStop(0, 'rgba(230, 57, 70, 0.3)');
                    gradient.addColorStop(0.5, 'rgba(230, 57, 70, 0.9)');
                    gradient.addColorStop(1, 'rgba(230, 57, 70, 0.3)');
                    ctx.strokeStyle = gradient;
                    ctx.lineWidth = 4;
                    for (let x = 0; x < width; x++) {
                        const y = centerY + Math.sin(x * 0.015 + phase * 2) * amp 
                                  + Math.sin(x * 0.04 + phase * 3) * (amp * 0.4);
                        if (x === 0) ctx.moveTo(x, y);
                        else ctx.lineTo(x, y);
                    }
                    ctx.stroke();
                    
                    // Glow
                    ctx.beginPath();
                    ctx.strokeStyle = 'rgba(230, 57, 70, 0.15)';
                    ctx.lineWidth = 12;
                    for (let x = 0; x < width; x++) {
                        const y = centerY + Math.sin(x * 0.015 + phase * 2) * amp;
                        if (x === 0) ctx.moveTo(x, y);
                        else ctx.lineTo(x, y);
                    }
                    ctx.stroke();
                    
                } else {
                    // Idle - slow smooth breathing
                    const amp = 12 + Math.sin(phase * 0.4) * 6;
                    
                    ctx.beginPath();
                    ctx.strokeStyle = 'rgba(99, 102, 241, 0.5)';
                    ctx.lineWidth = 2;
                    for (let x = 0; x < width; x++) {
                        const y = centerY + Math.sin(x * 0.015 + phase) * amp 
                                  + Math.sin(x * 0.04 + phase * 0.7) * (amp * 0.3);
                        if (x === 0) ctx.moveTo(x, y);
                        else ctx.lineTo(x, y);
                    }
                    ctx.stroke();
                    
                    // Subtle secondary wave
                    ctx.beginPath();
                    ctx.strokeStyle = 'rgba(99, 102, 241, 0.2)';
                    ctx.lineWidth = 1;
                    for (let x = 0; x < width; x++) {
                        const y = centerY + Math.sin(x * 0.01 - phase * 0.5) * (amp * 0.5);
                        if (x === 0) ctx.moveTo(x, y);
                        else ctx.lineTo(x, y);
                    }
                    ctx.stroke();
                }
                
                phase += 0.04;
                if (document.getElementById('danwa-view').style.display !== 'none') {
                    requestAnimationFrame(animateMouth);
                } else {
                    danwaAnimating = false;
                }
            }
            animateMouth();
        }
        
        // Initialize on page load
        (function initZen() {
            // Start in kata (form) mode with DANWA view
            const danwaView = document.getElementById('danwa-view');
            const nodesView = document.getElementById('nodes-view');
            
            if (danwaView) {
                // Initial setup for danwa view
                danwaView.style.display = 'flex';
                if (nodesView) nodesView.style.display = 'none';
                
                setZenState('idle');
                setZenEmotion('calm');
                initDanwaWaveform();
                renderEyebrows();
                applyZenStereo();
            }
        })();
    </script>
</body>
</html>
"""
    
    return HTMLResponse(content=html)


@app.get("/me", response_class=HTMLResponse)
async def me_stream_page() -> HTMLResponse:
    """Standalone ME camera stream page (proxied through KOKORO, HTTPS-friendly)."""
    html = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>ME Stream • VOIGHT</title>
  <style>
    :root {
      --bg: #07070c;
      --panel: rgba(255,255,255,0.06);
      --border: rgba(99,102,241,0.22);
      --text: rgba(235,238,255,0.92);
      --muted: rgba(235,238,255,0.55);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background:
        radial-gradient(1200px 600px at 50% -10%, rgba(99,102,241,0.18), transparent 60%),
        radial-gradient(900px 520px at 20% 20%, rgba(230,57,70,0.10), transparent 55%),
        var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
    }
    header {
      position: sticky;
      top: 0;
      z-index: 10;
      backdrop-filter: blur(10px);
      background: rgba(7,7,12,0.65);
      border-bottom: 1px solid rgba(255,255,255,0.06);
    }
    .bar {
      max-width: 1100px;
      margin: 0 auto;
      padding: 14px 16px;
      display: flex;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
    }
    .brand {
      letter-spacing: 0.14em;
      text-transform: uppercase;
      font-size: 0.85rem;
      color: var(--muted);
    }
    .brand strong { color: var(--text); letter-spacing: 0.18em; }
    .actions { display: flex; gap: 10px; align-items: center; }
    a.btn {
      text-decoration: none;
      color: var(--text);
      border: 1px solid var(--border);
      background: rgba(255,255,255,0.04);
      padding: 9px 12px;
      border-radius: 999px;
      font-size: 0.85rem;
    }
    .wrap {
      max-width: 1100px;
      margin: 0 auto;
      padding: 16px;
    }
    .panel {
      background: var(--panel);
      border: 1px solid rgba(255,255,255,0.08);
      border-radius: 16px;
      overflow: hidden;
      box-shadow: 0 18px 60px rgba(0,0,0,0.45);
    }
    .meta {
      display: flex;
      gap: 10px;
      align-items: center;
      justify-content: space-between;
      padding: 12px 14px;
      border-bottom: 1px solid rgba(255,255,255,0.08);
      color: var(--muted);
      font-size: 0.85rem;
    }
    .dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: rgba(34,197,94,0.8);
      box-shadow: 0 0 14px rgba(34,197,94,0.25);
      display: inline-block;
      margin-right: 8px;
    }
    .stream {
      width: 100%;
      display: block;
      background: #050508;
    }
    .hint {
      padding: 12px 14px;
      color: var(--muted);
      font-size: 0.82rem;
      border-top: 1px solid rgba(255,255,255,0.08);
    }
    code {
      font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace;
      color: rgba(235,238,255,0.8);
    }
  </style>
</head>
<body>
  <header>
    <div class="bar">
      <div class="brand"><strong>me</strong> stream • proxied via kokoro</div>
      <div class="actions">
        <a class="btn" href="/dashboard">Dashboard</a>
        <a class="btn" href="/proxy/me/snapshot" target="_blank" rel="noopener">Snapshot</a>
      </div>
    </div>
  </header>

  <div class="wrap">
    <div class="panel">
      <div class="meta">
        <div><span class="dot"></span>Live • MJPEG</div>
        <div><code>/proxy/me/stream</code></div>
      </div>
      <img class="stream" src="/proxy/me/stream" alt="ME camera stream" />
      <div class="hint">
        If this spins forever: open <code>/proxy/me/stream</code> directly in a new tab to confirm frames are flowing.
      </div>
    </div>
  </div>
</body>
</html>
"""
    return HTMLResponse(content=html)


@app.get("/te", response_class=HTMLResponse)
async def te_controls_page() -> HTMLResponse:
    """HTTPS-friendly TE control panel (proxied through KOKORO)."""
    html = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>TE Controls • VOIGHT</title>
  <style>
    :root {
      --bg: #07070c;
      --panel: rgba(255,255,255,0.06);
      --border: rgba(99,102,241,0.22);
      --text: rgba(235,238,255,0.92);
      --muted: rgba(235,238,255,0.55);
      --accent: #e63946;
      --online: #2ecc71;
      --warn: #f39c12;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background:
        radial-gradient(1200px 600px at 50% -10%, rgba(99,102,241,0.18), transparent 60%),
        radial-gradient(900px 520px at 20% 20%, rgba(230,57,70,0.10), transparent 55%),
        var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
    }
    header {
      position: sticky;
      top: 0;
      z-index: 10;
      backdrop-filter: blur(10px);
      background: rgba(7,7,12,0.65);
      border-bottom: 1px solid rgba(255,255,255,0.06);
    }
    .bar {
      max-width: 1100px;
      margin: 0 auto;
      padding: 14px 16px;
      display: flex;
      gap: 12px;
      align-items: center;
      justify-content: space-between;
    }
    .brand {
      letter-spacing: 0.14em;
      text-transform: uppercase;
      font-size: 0.85rem;
      color: var(--muted);
    }
    .brand strong { color: var(--text); letter-spacing: 0.18em; }
    .actions { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
    a.btn, button.btn {
      text-decoration: none;
      color: var(--text);
      border: 1px solid var(--border);
      background: rgba(255,255,255,0.04);
      padding: 9px 12px;
      border-radius: 999px;
      font-size: 0.85rem;
      cursor: pointer;
    }
    button.btn.primary { background: rgba(230,57,70,0.90); border-color: rgba(230,57,70,0.6); }
    button.btn.primary:hover { opacity: 0.9; }
    button.btn.ghost:hover, a.btn:hover { border-color: rgba(99,102,241,0.55); background: rgba(99,102,241,0.08); }

    .wrap { max-width: 1100px; margin: 0 auto; padding: 16px; }
    .panel {
      background: var(--panel);
      border: 1px solid rgba(255,255,255,0.08);
      border-radius: 16px;
      padding: 14px;
      box-shadow: 0 18px 60px rgba(0,0,0,0.45);
    }
    .row { display: flex; gap: 12px; align-items: center; flex-wrap: wrap; justify-content: space-between; }
    .statusline { color: var(--muted); font-size: 0.9rem; display: flex; gap: 10px; align-items: center; }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: rgba(230,57,70,0.8); box-shadow: 0 0 14px rgba(230,57,70,0.15); }
    .dot.ok { background: rgba(34,197,94,0.85); box-shadow: 0 0 14px rgba(34,197,94,0.18); }

    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(240px, 1fr)); gap: 12px; margin-top: 14px; }
    .servo {
      background: rgba(0,0,0,0.22);
      border: 1px solid rgba(255,255,255,0.08);
      border-radius: 14px;
      padding: 12px;
    }
    .servo-head { display:flex; justify-content: space-between; gap: 10px; align-items: baseline; margin-bottom: 10px; }
    .servo-name { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; letter-spacing: 0.08em; text-transform: uppercase; color: rgba(235,238,255,0.75); font-size: 0.78rem; }
    .servo-val { font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace; color: rgba(235,238,255,0.92); font-size: 0.9rem; }
    input[type="range"] { width: 100%; }
    .hint { margin-top: 10px; color: rgba(235,238,255,0.55); font-size: 0.85rem; }
  </style>
</head>
<body>
  <header>
    <div class="bar">
      <div class="brand"><strong>te</strong> controls • proxied via kokoro</div>
      <div class="actions">
        <a class="btn" href="/dashboard">Dashboard</a>
        <button class="btn ghost" id="btn-connect">Connect</button>
        <button class="btn ghost" id="btn-home">Home</button>
        <button class="btn ghost" id="btn-disconnect">Disconnect</button>
        <button class="btn primary" id="btn-move">Move</button>
      </div>
    </div>
  </header>

  <div class="wrap">
    <div class="panel">
      <div class="row">
        <div class="statusline">
          <span class="dot" id="te-dot"></span>
          <span id="te-status">Loading…</span>
          <span style="opacity:0.5">•</span>
          <span id="te-port" style="opacity:0.8"></span>
        </div>
        <div class="statusline">
          <span id="te-last" style="opacity:0.75"></span>
        </div>
      </div>

      <div class="grid" id="servo-grid"></div>
      <div class="hint">
        This page talks to TE using HTTPS-safe proxies: <code>/proxy/te/arm</code>, <code>/proxy/te/move</code>, <code>/proxy/te/gripper</code>.
      </div>
    </div>
  </div>

  <script>
    const state = { servos: [], connected: false, port: null };
    const ui = {
      controls: new Map(),       // servo_id -> { card, nameEl, valEl, slider }
      lastUserEdit: new Map(),   // servo_id -> timestamp ms
      dragging: new Set(),       // servo_id currently being dragged
      userHoldMs: 8000,          // how long we "trust" UI value over polled status (prevents snap-back)
    };

    function nowStr() {
      const d = new Date();
      return d.toLocaleTimeString();
    }

    function getLocalAngle(servo_id, fallback = 90) {
      const c = ui.controls.get(servo_id);
      if (!c) return fallback;
      const v = parseInt(c.slider.value, 10);
      return Number.isFinite(v) ? v : fallback;
    }

    function setServoAngleInState(servo_id, angle) {
      const s = state.servos.find(x => x.servo_id === servo_id);
      if (s) s.angle = angle;
    }

    function upsertServoCard(s) {
      const grid = document.getElementById('servo-grid');
      let c = ui.controls.get(s.servo_id);
      if (!c) {
        const card = document.createElement('div');
        card.className = 'servo';

        const head = document.createElement('div');
        head.className = 'servo-head';

        const nameEl = document.createElement('div');
        nameEl.className = 'servo-name';

        const valEl = document.createElement('div');
        valEl.className = 'servo-val';

        head.appendChild(nameEl);
        head.appendChild(valEl);
        card.appendChild(head);

        const slider = document.createElement('input');
        slider.type = 'range';
        slider.min = '0';
        slider.max = '180';

        const sid = s.servo_id;
        const markDragging = () => ui.dragging.add(sid);
        const unmarkDragging = () => ui.dragging.delete(sid);
        slider.addEventListener('pointerdown', markDragging);
        slider.addEventListener('pointerup', unmarkDragging);
        slider.addEventListener('pointercancel', unmarkDragging);
        slider.addEventListener('mousedown', markDragging);
        slider.addEventListener('mouseup', unmarkDragging);
        slider.addEventListener('touchstart', markDragging, { passive: true });
        slider.addEventListener('touchend', unmarkDragging, { passive: true });
        slider.addEventListener('touchcancel', unmarkDragging, { passive: true });

        slider.addEventListener('input', (e) => {
          const a = parseInt(e.target.value, 10);
          if (!Number.isFinite(a)) return;
          ui.lastUserEdit.set(sid, Date.now());
          setServoAngleInState(sid, a);
          valEl.textContent = a + '°';
        });

        card.appendChild(slider);
        grid.appendChild(card);

        c = { card, nameEl, valEl, slider };
        ui.controls.set(s.servo_id, c);
      }

      // Always update name
      c.nameEl.textContent = s.label ?? ('servo ' + s.servo_id);

      // If the user has recently edited this slider, keep the UI value and don't overwrite from polling.
      const now = Date.now();
      const lastEdit = ui.lastUserEdit.get(s.servo_id) || 0;
      const hold = (now - lastEdit) < ui.userHoldMs;
      const dragging = ui.dragging.has(s.servo_id);

      const serverAngle = Number.isFinite(s.angle) ? s.angle : 90;
      if (!dragging && !hold) {
        c.slider.value = String(serverAngle);
        c.valEl.textContent = serverAngle + '°';
      } else {
        const localAngle = getLocalAngle(s.servo_id, serverAngle);
        c.valEl.textContent = localAngle + '°';
        setServoAngleInState(s.servo_id, localAngle);
      }
    }

    function syncGrid() {
      const seen = new Set();
      (state.servos || []).forEach((s) => {
        if (!s || typeof s.servo_id !== 'number') return;
        seen.add(s.servo_id);
        upsertServoCard(s);
      });

      // Remove cards for servos that disappeared
      for (const [sid, c] of ui.controls.entries()) {
        if (!seen.has(sid)) {
          try { c.card.remove(); } catch (e) {}
          ui.controls.delete(sid);
          ui.lastUserEdit.delete(sid);
          ui.dragging.delete(sid);
        }
      }
    }

    async function refresh() {
      const dot = document.getElementById('te-dot');
      const status = document.getElementById('te-status');
      const port = document.getElementById('te-port');
      const last = document.getElementById('te-last');

      try {
        const r = await fetch('/proxy/te/arm', { cache: 'no-store' });
        const j = await r.json();
        state.connected = !!j.connected;
        state.port = j.port || '';

        const incoming = Array.isArray(j.servos) ? j.servos : [];
        // Merge: if user recently edited a servo, keep their value so sliders don't "snap back".
        const now = Date.now();
        state.servos = incoming.map((s) => {
          const sid = s && typeof s.servo_id === 'number' ? s.servo_id : null;
          if (sid === null) return s;
          const lastEdit = ui.lastUserEdit.get(sid) || 0;
          const hold = (now - lastEdit) < ui.userHoldMs;
          if (hold) {
            return { ...s, angle: getLocalAngle(sid, s.angle ?? 90) };
          }
          return s;
        });

        dot.classList.toggle('ok', state.connected);
        status.textContent = state.connected ? 'Connected' : 'Disconnected';
        port.textContent = state.port ? ('port: ' + state.port) : '';
        last.textContent = 'updated: ' + nowStr();

        syncGrid();
      } catch (e) {
        dot.classList.remove('ok');
        status.textContent = 'TE unreachable';
        port.textContent = '';
        last.textContent = 'updated: ' + nowStr();
      }
    }

    async function post(url, body) {
      const r = await fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: body ? JSON.stringify(body) : null
      });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      return await r.json();
    }

    document.getElementById('btn-connect').addEventListener('click', async () => {
      try { await post('/proxy/te/connect'); } finally { await refresh(); }
    });
    document.getElementById('btn-disconnect').addEventListener('click', async () => {
      try { await post('/proxy/te/disconnect'); } finally { await refresh(); }
    });
    document.getElementById('btn-home').addEventListener('click', async () => {
      // Some TE builds may expose /home; if not, ignore.
      try { await post('/proxy/te/home'); } catch (e) {}
      await refresh();
    });

    document.getElementById('btn-move').addEventListener('click', async () => {
      // TE expects { job_id, commands: [{servo_id, angle, speed}] }
      const job_id = 'kokoro-' + Date.now();
      const commands = state.servos.map(s => ({ servo_id: s.servo_id, angle: s.angle ?? 90, speed: 50 }));
      try {
        await post('/proxy/te/move', { job_id, commands });
      } finally {
        await refresh();
      }
    });

    refresh();
    setInterval(refresh, 1500);
  </script>
</body>
</html>
"""
    return HTMLResponse(content=html)


@app.post("/dispatch")
async def dispatch_job(request: DispatchRequest) -> DispatchResult:
    """
    Dispatch a job to a target node.
    """
    if request.target_node not in CLUSTER_NODES:
        return DispatchResult(
            success=False,
            target_node=request.target_node,
            endpoint=request.endpoint,
            error=f"Unknown node: {request.target_node}",
        )
    
    node_info = CLUSTER_NODES[request.target_node]
    url = f"{node_info['url']}{request.endpoint}"
    
    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(url, json=request.payload)
            response.raise_for_status()
            return DispatchResult(
                success=True,
                target_node=request.target_node,
                endpoint=request.endpoint,
                response=response.json(),
            )
    except Exception as e:
        return DispatchResult(
            success=False,
            target_node=request.target_node,
            endpoint=request.endpoint,
            error=str(e),
        )


# -----------------------------------------------------------------------------
# Main Entry Point
# -----------------------------------------------------------------------------

if __name__ == "__main__":
    import uvicorn
    import os
    import sys
    
    # SSL certificate paths
    ssl_dir = os.path.join(os.path.dirname(__file__), "ssl")
    ssl_cert = os.path.join(ssl_dir, "cert.pem")
    ssl_key = os.path.join(ssl_dir, "key.pem")
    
    # Check if SSL certs exist
    use_ssl = os.path.exists(ssl_cert) and os.path.exists(ssl_key)
    
    print("=" * 60)
    print("  VOIGHT CLUSTER Orchestration Node (KOKORO) - 心")
    if use_ssl:
        print("  Starting server on https://0.0.0.0:8025 (SSL enabled)")
        print("  Microphone access available from any device!")
    else:
        print("  Starting server on http://0.0.0.0:8025")
        print("  ⚠️  No SSL - microphone only works on localhost")
    print("=" * 60)
    
    if use_ssl:
        uvicorn.run(
            app,
            host="0.0.0.0",
            port=8025,
            ssl_certfile=ssl_cert,
            ssl_keyfile=ssl_key,
        )
    else:
        uvicorn.run(
            app,
            host="0.0.0.0",
            port=8025,
        )
