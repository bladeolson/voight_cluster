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
    # Skip self-check for kokoro
    if name == "kokoro":
        return NodeStatus(
            name=name,
            kanji=info["kanji"],
            role=info["role"],
            url=info["url"],
            online=True,
            status={"node": "kokoro", "note": "This is the orchestrator"},
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
    <meta http-equiv="refresh" content="10">
    <title>VOIGHT CLUSTER Dashboard</title>
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
    </style>
</head>
<body>
    <div class="container">
        <header>
            <div class="title-kanji">心</div>
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
"""
                if gpus > 0:
                    html += f"""
                    <div class="detail-row">
                        <span class="detail-label">GPUs</span>
                        <span>{gpus}</span>
                    </div>
"""
                html += """
                </div>
"""

            if "capabilities" in node.status:
                caps = node.status["capabilities"]
                cap_list = [k for k, v in caps.items() if v]
                if cap_list:
                    html += f"""
                <div class="node-details">
                    <div class="detail-row">
                        <span class="detail-label">Capabilities</span>
                        <span class="detail-value">{", ".join(cap_list[:2])}</span>
                    </div>
                </div>
"""
            
            # Show camera feed if available
            if "capabilities" in node.status and node.status["capabilities"].get("camera_array", False):
                snapshot_url = f"{node.url}/snapshot?camera=0"
                # Use a timestamp to prevent caching
                html += f"""
                <div class="node-details">
                    <div class="detail-row" style="flex-direction: column; align-items: flex-start; gap: 0.5rem;">
                        <span class="detail-label">Live Feed</span>
                        <img src="{snapshot_url}" style="width: 100%; border-radius: 4px; border: 1px solid var(--border);" onload="setTimeout(() => this.src = '{snapshot_url}&t=' + new Date().getTime(), 1000)" onerror="this.style.display='none'">
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
            <p class="refresh-note">Auto-refreshes every 10 seconds</p>
        </footer>
    </div>
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
    
    print("=" * 60)
    print("  Zen Orchestration Node (KOKORO) - 心")
    print("  Starting server on http://0.0.0.0:8025")
    print("=" * 60)
    
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8025,
    )
