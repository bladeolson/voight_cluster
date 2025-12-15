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
from fastapi.responses import HTMLResponse, RedirectResponse
from pydantic import BaseModel
import httpx
import psutil


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
            return NodeStatus(
                name=name,
                kanji=info["kanji"],
                role=info["role"],
                url=info["url"],
                online=True,
                status=response.json(),
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


@app.get("/proxy/me/stream")
async def proxy_me_stream():
    """Proxy ME camera stream through KOKORO for HTTPS access."""
    import httpx
    from fastapi.responses import StreamingResponse
    
    async def stream_generator():
        async with httpx.AsyncClient(timeout=None) as client:
            async with client.stream("GET", "http://me.local:8028/stream") as response:
                async for chunk in response.aiter_bytes():
                    yield chunk
    
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
                        btn.innerHTML = '<span style="font-size:1.1rem;">🎤</span> Talk to Zen';
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
                        
                        // Send to LLM (use Ollama directly for reliability)
                        try {
                            const resp = await fetch('http://kokoro.local:11434/api/generate', {
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
            stroke-width: 3;
            fill: none;
            filter: drop-shadow(0 0 4px rgba(99, 102, 241, 0.5));
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
            gap: 50px;
            justify-content: center;
            position: relative;
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
            width: 100%;
            height: 100%;
            object-fit: cover;
            opacity: 0.9;
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
        
        .zen-emotion-indicator {
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 8px;
            margin-bottom: 8px;
            font-size: 0.75rem;
            text-transform: uppercase;
            letter-spacing: 0.15em;
            color: rgba(99, 102, 241, 0.7);
            font-family: 'JetBrains Mono', monospace;
        }
        
        .zen-emotion-dot {
            width: 6px;
            height: 6px;
            border-radius: 50%;
            background: rgba(99, 102, 241, 0.6);
            transition: all 0.4s ease;
        }
        
        .zen-emotion-indicator[data-emotion="calm"] .zen-emotion-dot { background: rgba(99, 102, 241, 0.5); }
        .zen-emotion-indicator[data-emotion="curious"] .zen-emotion-dot { background: rgba(120, 200, 180, 0.7); }
        .zen-emotion-indicator[data-emotion="concern"] .zen-emotion-dot { background: rgba(180, 140, 200, 0.7); }
        .zen-emotion-indicator[data-emotion="focus"] .zen-emotion-dot { background: rgba(100, 120, 180, 0.7); }
        .zen-emotion-indicator[data-emotion="joy"] .zen-emotion-dot { background: rgba(200, 180, 120, 0.7); }
        .zen-emotion-indicator[data-emotion="confused"] .zen-emotion-dot { background: rgba(180, 160, 140, 0.7); }
        .zen-emotion-indicator[data-emotion="alert"] .zen-emotion-dot { background: rgba(200, 120, 120, 0.7); }
        
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
                <div class="zen-emotion-indicator" id="zen-emotion-display" data-emotion="calm">
                    <span class="zen-emotion-dot"></span>
                    <span class="zen-emotion-label">calm</span>
                </div>
                <div class="zen-thought-bubble" id="zen-thought"></div>
            </div>
            
            <!-- Eyes with Eyebrows -->
            <div class="zen-eyes-container">
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
                        <img src="/proxy/me/stream" alt="Left Eye" onerror="this.src=''; this.alt='👁️'">
                    </div>
                    <div class="eye right-eye">
                        <img src="/proxy/me/stream" alt="Right Eye" onerror="this.src=''; this.alt='👁️'">
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
                # Use MJPEG stream for smooth video (port 8028)
                stream_url = f"http://{node.url.split('//')[1].split(':')[0]}:8028/stream"
                html += f"""
                <div class="node-details">
                    <div class="detail-row" style="flex-direction: column; align-items: flex-start; gap: 0.5rem;">
                        <span class="detail-label">Live Stream</span>
                        <img src="{stream_url}" style="width: 100%; border-radius: 4px; border: 1px solid var(--border);" onerror="this.src=''; this.alt='Stream offline'">
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
                            <button id="zen-talk-btn" onclick="toggleZenVoice()" 
                                    style="padding: 10px 24px; background: var(--accent); color: white; border: none; 
                                           border-radius: 6px; cursor: pointer; font-size: 0.85rem; font-weight: 500;
                                           transition: all 0.2s; display: flex; align-items: center; gap: 8px;">
                                <span style="font-size: 1.1rem;">🎤</span> Talk to Zen
                            </button>
                            <a href="{voice_url}" target="_blank" 
                               style="padding: 10px 16px; background: transparent; color: var(--text-secondary); 
                                      border: 1px solid var(--border); border-radius: 6px; text-decoration: none;
                                      font-size: 0.8rem; transition: all 0.2s; display: flex; align-items: center;"
                               onmouseover="this.style.borderColor='var(--accent)'; this.style.color='var(--accent)'" 
                               onmouseout="this.style.borderColor='var(--border)'; this.style.color='var(--text-secondary)'">
                                Open Chat ↗
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
                    <div style="margin-top:10px; text-align:center;">
                        <a href="http://te.local:8027/dashboard" target="_blank" 
                           style="display:inline-block; padding:6px 16px; background:var(--accent); color:white; 
                                  text-decoration:none; border-radius:4px; font-size:0.75rem; 
                                  transition: opacity 0.2s;"
                           onmouseover="this.style.opacity='0.8'" onmouseout="this.style.opacity='1'">
                            Open Arm Control
                        </a>
                    </div>
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
        html += """
        <div class="section-title">Discovered Services</div>
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
        
        async function updateTeArm() {
            if (!teAvailable && teRetryCount < 10) {
                teRetryCount++;
                return; // Skip if TE was unavailable, retry every 5 seconds
            }
            teRetryCount = 0;
            
            try {
                const controller = new AbortController();
                const timeout = setTimeout(() => controller.abort(), 2000);
                
                const response = await fetch('http://te.local:8027/arm/status', {
                    signal: controller.signal,
                    mode: 'cors'
                });
                clearTimeout(timeout);
                
                if (!response.ok) { teAvailable = false; return; }
                
                const data = await response.json();
                teAvailable = true;
                
                if (data.joints) {
                    data.joints.forEach((angle, i) => {
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
            amplitude: 8,
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
            calm:     { amplitude: 6,  baselineOffset: 0,  innerBias: 0,   outerBias: 0,   frequency: 0.015, motionSpeed: 0.015, symmetry: 1.0 },
            curious:  { amplitude: 10, baselineOffset: 4,  innerBias: 0,   outerBias: 3,   frequency: 0.018, motionSpeed: 0.02,  symmetry: 0.95 },
            concern:  { amplitude: 5,  baselineOffset: -2, innerBias: -4,  outerBias: 2,   frequency: 0.02,  motionSpeed: 0.01,  symmetry: 0.98 },
            focus:    { amplitude: 3,  baselineOffset: -3, innerBias: 0,   outerBias: -1,  frequency: 0.03,  motionSpeed: 0.008, symmetry: 1.0 },
            joy:      { amplitude: 8,  baselineOffset: 3,  innerBias: 2,   outerBias: 2,   frequency: 0.012, motionSpeed: 0.018, symmetry: 1.0 },
            confused: { amplitude: 6,  baselineOffset: 0,  innerBias: -1,  outerBias: 2,   frequency: 0.02,  motionSpeed: 0.012, symmetry: 0.7 },
            alert:    { amplitude: 12, baselineOffset: 5,  innerBias: 0,   outerBias: 0,   frequency: 0.035, motionSpeed: 0.05,  symmetry: 1.0 }
        };
        
        function setZenEmotion(emotion) {
            window.zenEmotion = emotion;
            
            // Update emotion display
            const emotionDisplay = document.getElementById('zen-emotion-display');
            if (emotionDisplay) {
                emotionDisplay.setAttribute('data-emotion', emotion);
                const label = emotionDisplay.querySelector('.zen-emotion-label');
                if (label) label.textContent = emotion;
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
                    const wave = Math.sin(t * Math.PI * 2 + p.phase) * 1.5;
                    const arch = Math.sin(t * Math.PI) * p.amplitude;
                    // inner = right side of left brow (t=1), outer = left side (t=0)
                    const bias = p.outerBias * (1-t) + p.innerBias * t;
                    const asym = (1 - p.symmetry) * 4 * (t - 0.5);
                    const ly = baseY - p.baselineOffset - arch - bias + wave + asym;
                    leftPoints.push({x: lx, y: Math.max(5, Math.min(45, ly))});
                    
                    // Right eyebrow (mirrored: inner at t=0, outer at t=1)
                    const rx = 490 + t * 160;
                    const rBias = p.innerBias * (1-t) + p.outerBias * t;
                    const rAsym = (1 - p.symmetry) * 4 * (0.5 - t);
                    const ry = baseY - p.baselineOffset - arch - rBias + wave + rAsym;
                    rightPoints.push({x: rx, y: Math.max(5, Math.min(45, ry))});
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
                }
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
            }
        })();
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
