#!/usr/bin/env python3
"""
VOIGHT CLUSTER Control CLI
=======================
Manage the entire VOIGHT CLUSTER from the Orchestrator (Kokoro).

Usage:
  ./zen_ctl.py restart [all|me|te|kokoro]
  ./zen_ctl.py status
  ./zen_ctl.py stop [all|me|te|kokoro]
"""

import sys
import subprocess
import argparse

# Cluster definition
NODES = {
    "me": {
        "host": "me.local",
        "user": "bladeolson",
        "service": "zen-me-node",
        "path": "~/zen_me_node"
    },
    "te": {
        "host": "te.local",
        "user": "bladeolson",
        "service": "zen-te-node",
        "path": "~/zen_te_node"
    },
    "kokoro": {
        "host": "localhost",
        "user": "bladeolson",
        "service": "zen-kokoro-node",
        "path": "~/voight_cluster/kokoro"
    }
}

def run_ssh(node_name, cmd):
    """Run a command on a node via SSH (or locally for Kokoro)."""
    node = NODES[node_name]
    
    if node["host"] == "localhost":
        print(f"[{node_name}] Running: {cmd}")
        subprocess.run(cmd, shell=True)
    else:
        ssh_cmd = f"ssh {node['user']}@{node['host']} '{cmd}'"
        print(f"[{node_name}] {ssh_cmd}")
        subprocess.run(ssh_cmd, shell=True)

def restart_service(node_name):
    """Restart the systemd service on a node."""
    service = NODES[node_name]["service"]
    run_ssh(node_name, f"sudo systemctl restart {service}")

def stop_service(node_name):
    """Stop the systemd service on a node."""
    service = NODES[node_name]["service"]
    run_ssh(node_name, f"sudo systemctl stop {service}")

def check_status():
    """Check status of all nodes."""
    print("\n=== VOIGHT CLUSTER Status ===")
    for name, node in NODES.items():
        service = node["service"]
        if node["host"] == "localhost":
            res = subprocess.run(f"systemctl is-active {service}", shell=True, capture_output=True, text=True)
            status = res.stdout.strip()
        else:
            ssh_cmd = f"ssh {node['user']}@{node['host']} 'systemctl is-active {service}'"
            res = subprocess.run(ssh_cmd, shell=True, capture_output=True, text=True)
            status = res.stdout.strip()
        
        icon = "✅" if status == "active" else "❌"
        print(f"{icon} {name.upper():<10} : {status}")

def main():
    parser = argparse.ArgumentParser(description="VOIGHT CLUSTER Control")
    parser.add_argument("action", choices=["restart", "stop", "status"], help="Action to perform")
    parser.add_argument("target", nargs="?", default="all", help="Target node (all, me, te, kokoro)")
    
    args = parser.parse_args()
    
    if args.action == "status":
        check_status()
        return

    targets = [args.target] if args.target != "all" else NODES.keys()
    
    for target in targets:
        if target not in NODES:
            print(f"Unknown node: {target}")
            continue
            
        if args.action == "restart":
            restart_service(target)
        elif args.action == "stop":
            stop_service(target)

if __name__ == "__main__":
    main()

