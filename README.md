# Zen KOKORO Node (心)

**Orchestration node for the Zen cluster.**

| Property | Value |
|----------|-------|
| Node Name | KOKORO |
| Kanji | 心 (heart/mind) |
| Hostname | `kokoro.local` |
| Port | `8025` |
| Role | Orchestration / Dashboard / Coordination |

## Overview

The KOKORO node is the "heart" of the Zen cluster. Its responsibilities include:

- **Dashboard**: Web UI to monitor all nodes in real-time
- **Cluster Monitor**: Query all nodes for their status
- **Job Dispatch**: Send compute jobs to worker nodes (ME, TE)
- **Coordination**: Central point for cluster management

## Cluster Context

```
KOKORO (心) ──── ME (目) ──── TE (手)
 orchestrate      vision       robot arm
   :8025          :8026         :8027
     ★
```

## Setup

### 1. Create a Virtual Environment

```bash
cd zen_kokoro_node
python3 -m venv .venv
source .venv/bin/activate
```

### 2. Install Dependencies

```bash
pip install -r requirements.txt
```

### 3. Run the Server

Option A - Direct Python:

```bash
python zen_kokoro_server.py
```

Option B - Using uvicorn directly:

```bash
uvicorn zen_kokoro_server:app --host 0.0.0.0 --port 8025
```

Option C - With auto-reload for development:

```bash
uvicorn zen_kokoro_server:app --host 0.0.0.0 --port 8025 --reload
```

## API Endpoints

### `GET /`

Simple alive check.

```bash
curl http://kokoro.local:8025/
```

Response:

```json
{
  "message": "Zen Orchestration Node (KOKORO) online.",
  "endpoints": ["/status", "/cluster", "/dashboard", "/dispatch"]
}
```

### `GET /status`

Returns node identity and info.

```bash
curl http://kokoro.local:8025/status
```

### `GET /cluster`

Returns aggregated status of all nodes in the cluster.

```bash
curl http://kokoro.local:8025/cluster
```

Response:

```json
{
  "timestamp": "2024-01-15T10:30:00.000000",
  "orchestrator": "kokoro",
  "nodes": [
    {
      "name": "kokoro",
      "kanji": "心",
      "role": "orchestration",
      "url": "http://localhost:8025",
      "online": true,
      "status": {...}
    },
    {
      "name": "me",
      "kanji": "目",
      "role": "vision",
      "url": "http://me.local:8026",
      "online": true,
      "status": {...}
    }
  ],
  "summary": {
    "total_nodes": 3,
    "online": 2,
    "offline": 1
  }
}
```

### `GET /dashboard`

**Web Dashboard** - Open in a browser for a live view of the cluster.

```
http://kokoro.local:8025/dashboard
```

Features:
- Real-time node status
- Auto-refresh every 5 seconds
- Shows GPU/RAM info for compute nodes
- Visual online/offline indicators

### `POST /dispatch`

Dispatch a job to a target node.

```bash
curl -X POST http://kokoro.local:8025/dispatch \
  -H "Content-Type: application/json" \
  -d '{
    "target_node": "me",
    "endpoint": "/compute",
    "payload": {"job_id": "test-1", "numbers": [1, 2, 3]}
  }'
```

Response:

```json
{
  "success": true,
  "target_node": "me",
  "endpoint": "/compute",
  "response": {
    "job_id": "test-1",
    "input": [1.0, 2.0, 3.0],
    "result": [1.0, 4.0, 9.0],
    "node": "me",
    "note": "Super rudimentary Zen vision-node compute stub"
  }
}
```

## Cluster Configuration

The cluster nodes are defined in `zen_kokoro_server.py`:

```python
CLUSTER_NODES = {
    "kokoro": {"url": "http://localhost:8025", "kanji": "心", "role": "orchestration"},
    "me": {"url": "http://me.local:8026", "kanji": "目", "role": "vision"},
    "te": {"url": "http://te.local:8027", "kanji": "手", "role": "limbs"},
}
```

## Running as a Service

See `docs/kokoro_node_setup.md` for systemd service configuration.

## Requirements

- Python 3.10+
- Network access to other cluster nodes

## License

Part of the Zen cluster project.

