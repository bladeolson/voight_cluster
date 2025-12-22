#!/usr/bin/env python3
"""
VOIGHT CLUSTER Control CLI
===========================
Manage the entire VOIGHT CLUSTER from the Orchestrator (KOKORO).
Requires passwordless SSH to be set up (run setup_ssh.sh first).

Usage:
  ./voight_ctl.py status                    # Show cluster status
  ./voight_ctl.py ssh <node>                # Open SSH session to node
  ./voight_ctl.py restart [all|me|te|kokoro]
  ./voight_ctl.py stop [all|me|te|kokoro]
  ./voight_ctl.py start [all|me|te|kokoro]
  ./voight_ctl.py logs <node> [lines]       # Show service logs
  ./voight_ctl.py deploy <node>             # Deploy latest code to node
  ./voight_ctl.py health                    # Full health check
"""

import sys
import os
import subprocess
import argparse
import json
from datetime import datetime
from typing import Optional, Dict, Any

# ANSI colors
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    MAGENTA = '\033[0;35m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    NC = '\033[0m'  # No Color


# Cluster definition
NODES = {
    "kokoro": {
        "host": "kokoro.local",
        "user": "bladeolson",
        "service": "zen-kokoro-node",
        "path": "~/zen_kokoro_node",
        "port": 8025,
        "kanji": "心",
        "role": "orchestration",
        "alt_ip": "10.0.0.205",
    },
    "me": {
        "host": "me.local",
        "user": "bladeolson",
        "service": "zen-me-node",
        "path": "~/Documents/zen_me_node",
        "port": 8028,
        "kanji": "目",
        "role": "vision",
        "alt_ip": "10.0.0.30",
    },
    "te": {
        "host": "te.local",
        "user": "bladeolson",
        "service": "zen-te-node",
        "path": "~/zen_te_node",
        "port": 8027,
        "kanji": "手",
        "role": "limbs",
        "alt_ip": "10.0.0.242",
    },
}


def print_header():
    """Print the cluster header."""
    print()
    print(f"{Colors.CYAN}╔═══════════════════════════════════════════════════════════════╗{Colors.NC}")
    print(f"{Colors.CYAN}║{Colors.NC}              {Colors.YELLOW}VOIGHT CLUSTER{Colors.NC} Control                          {Colors.CYAN}║{Colors.NC}")
    print(f"{Colors.CYAN}║{Colors.NC}                   心 → 目 → 手                               {Colors.CYAN}║{Colors.NC}")
    print(f"{Colors.CYAN}╚═══════════════════════════════════════════════════════════════╝{Colors.NC}")
    print()


def run_ssh(node_name: str, cmd: str, capture: bool = False, timeout: int = 30) -> subprocess.CompletedProcess:
    """Run a command on a node via SSH."""
    node = NODES[node_name]
    
    # Build SSH command
    ssh_target = f"{node['user']}@{node['host']}"
    ssh_opts = "-o BatchMode=yes -o ConnectTimeout=10 -o StrictHostKeyChecking=no"
    ssh_cmd = f"ssh {ssh_opts} {ssh_target} '{cmd}'"
    
    if not capture:
        print(f"{Colors.DIM}[{node_name}] {ssh_cmd}{Colors.NC}")
    
    try:
        result = subprocess.run(ssh_cmd, shell=True, capture_output=capture, text=True, timeout=timeout)
        return result
    except subprocess.TimeoutExpired:
        # Try alternate IP
        if node.get("alt_ip"):
            ssh_target = f"{node['user']}@{node['alt_ip']}"
            ssh_cmd = f"ssh {ssh_opts} {ssh_target} '{cmd}'"
            return subprocess.run(ssh_cmd, shell=True, capture_output=capture, text=True, timeout=timeout)
        raise


def test_ssh_connection(node_name: str) -> bool:
    """Test if we can SSH to a node without password."""
    try:
        result = run_ssh(node_name, "echo ok", capture=True, timeout=10)
        return result.returncode == 0 and "ok" in result.stdout
    except Exception:
        return False


def get_service_status(node_name: str) -> Dict[str, Any]:
    """Get detailed status of a node's service."""
    node = NODES[node_name]
    service = node["service"]
    
    status = {
        "name": node_name,
        "kanji": node["kanji"],
        "role": node["role"],
        "port": node["port"],
        "ssh_ok": False,
        "service_active": False,
        "service_status": "unknown",
        "uptime": None,
        "memory": None,
        "cpu": None,
    }
    
    # Test SSH
    if not test_ssh_connection(node_name):
        return status
    status["ssh_ok"] = True
    
    # Get service status
    result = run_ssh(node_name, f"systemctl is-active {service}", capture=True, timeout=10)
    service_state = result.stdout.strip() if result.returncode == 0 else result.stderr.strip()
    status["service_status"] = service_state
    status["service_active"] = service_state == "active"
    
    # Get system info
    try:
        # Uptime
        result = run_ssh(node_name, "uptime -p", capture=True, timeout=5)
        if result.returncode == 0:
            status["uptime"] = result.stdout.strip()
        
        # Memory
        result = run_ssh(node_name, "free -h | awk 'NR==2{print $3\"/\"$2}'", capture=True, timeout=5)
        if result.returncode == 0:
            status["memory"] = result.stdout.strip()
        
        # CPU
        result = run_ssh(node_name, "top -bn1 | grep 'Cpu(s)' | awk '{print $2}'", capture=True, timeout=5)
        if result.returncode == 0:
            status["cpu"] = result.stdout.strip()
    except Exception:
        pass
    
    return status


def cmd_status():
    """Show status of all nodes."""
    print_header()
    print(f"{Colors.BOLD}Cluster Status{Colors.NC}")
    print(f"{Colors.DIM}{'─' * 63}{Colors.NC}")
    print()
    
    for name, node in NODES.items():
        status = get_service_status(name)
        
        # Node header
        kanji = node["kanji"]
        ssh_icon = f"{Colors.GREEN}●{Colors.NC}" if status["ssh_ok"] else f"{Colors.RED}○{Colors.NC}"
        svc_icon = f"{Colors.GREEN}●{Colors.NC}" if status["service_active"] else f"{Colors.RED}○{Colors.NC}"
        
        print(f"  {Colors.CYAN}{kanji}{Colors.NC} {Colors.BOLD}{name.upper():<10}{Colors.NC} [{node['role']}]")
        print(f"     SSH: {ssh_icon}  Service: {svc_icon} ({status['service_status']})")
        
        if status["ssh_ok"]:
            host = f"{node['user']}@{node['host']}:{node['port']}"
            print(f"     {Colors.DIM}Host: {host}{Colors.NC}")
            if status["uptime"]:
                print(f"     {Colors.DIM}Uptime: {status['uptime']}{Colors.NC}")
            if status["memory"]:
                print(f"     {Colors.DIM}Memory: {status['memory']}{Colors.NC}")
        else:
            print(f"     {Colors.YELLOW}⚠ Cannot reach node (SSH failed){Colors.NC}")
        
        print()
    
    print(f"{Colors.DIM}{'─' * 63}{Colors.NC}")
    print(f"{Colors.DIM}Tip: Run 'setup_ssh.sh' if SSH connections are failing{Colors.NC}")


def cmd_restart(target: str):
    """Restart service on target node(s)."""
    print_header()
    targets = [target] if target != "all" else list(NODES.keys())
    
    for name in targets:
        if name not in NODES:
            print(f"{Colors.RED}✗ Unknown node: {name}{Colors.NC}")
            continue
        
        node = NODES[name]
        service = node["service"]
        kanji = node["kanji"]
        
        print(f"  {Colors.CYAN}{kanji}{Colors.NC} Restarting {name.upper()}...")
        
        if not test_ssh_connection(name):
            print(f"     {Colors.RED}✗ Cannot connect to {name}{Colors.NC}")
            continue
        
        result = run_ssh(name, f"sudo systemctl restart {service}", capture=True)
        if result.returncode == 0:
            print(f"     {Colors.GREEN}✓ Service restarted{Colors.NC}")
        else:
            print(f"     {Colors.RED}✗ Failed: {result.stderr.strip()}{Colors.NC}")


def cmd_stop(target: str):
    """Stop service on target node(s)."""
    print_header()
    targets = [target] if target != "all" else list(NODES.keys())
    
    for name in targets:
        if name not in NODES:
            print(f"{Colors.RED}✗ Unknown node: {name}{Colors.NC}")
            continue
        
        node = NODES[name]
        service = node["service"]
        kanji = node["kanji"]
        
        print(f"  {Colors.CYAN}{kanji}{Colors.NC} Stopping {name.upper()}...")
        
        if not test_ssh_connection(name):
            print(f"     {Colors.RED}✗ Cannot connect to {name}{Colors.NC}")
            continue
        
        result = run_ssh(name, f"sudo systemctl stop {service}", capture=True)
        if result.returncode == 0:
            print(f"     {Colors.GREEN}✓ Service stopped{Colors.NC}")
        else:
            print(f"     {Colors.RED}✗ Failed: {result.stderr.strip()}{Colors.NC}")


def cmd_start(target: str):
    """Start service on target node(s)."""
    print_header()
    targets = [target] if target != "all" else list(NODES.keys())
    
    for name in targets:
        if name not in NODES:
            print(f"{Colors.RED}✗ Unknown node: {name}{Colors.NC}")
            continue
        
        node = NODES[name]
        service = node["service"]
        kanji = node["kanji"]
        
        print(f"  {Colors.CYAN}{kanji}{Colors.NC} Starting {name.upper()}...")
        
        if not test_ssh_connection(name):
            print(f"     {Colors.RED}✗ Cannot connect to {name}{Colors.NC}")
            continue
        
        result = run_ssh(name, f"sudo systemctl start {service}", capture=True)
        if result.returncode == 0:
            print(f"     {Colors.GREEN}✓ Service started{Colors.NC}")
        else:
            print(f"     {Colors.RED}✗ Failed: {result.stderr.strip()}{Colors.NC}")


def cmd_logs(node_name: str, lines: int = 50):
    """Show service logs for a node."""
    if node_name not in NODES:
        print(f"{Colors.RED}✗ Unknown node: {node_name}{Colors.NC}")
        return
    
    node = NODES[node_name]
    service = node["service"]
    
    print_header()
    print(f"  {Colors.CYAN}{node['kanji']}{Colors.NC} Logs for {node_name.upper()} ({service})")
    print(f"{Colors.DIM}{'─' * 63}{Colors.NC}")
    print()
    
    if not test_ssh_connection(node_name):
        print(f"{Colors.RED}✗ Cannot connect to {node_name}{Colors.NC}")
        return
    
    run_ssh(node_name, f"sudo journalctl -u {service} -n {lines} --no-pager")


def cmd_ssh(node_name: str):
    """Open interactive SSH session to a node."""
    if node_name not in NODES:
        print(f"{Colors.RED}✗ Unknown node: {node_name}{Colors.NC}")
        return
    
    node = NODES[node_name]
    target = f"{node['user']}@{node['host']}"
    print(f"{Colors.CYAN}Connecting to {node['kanji']} {node_name.upper()}...{Colors.NC}")
    
    os.execvp("ssh", ["ssh", "-o", "StrictHostKeyChecking=no", target])


def cmd_health():
    """Run full health check on all nodes."""
    print_header()
    print(f"{Colors.BOLD}Health Check{Colors.NC}")
    print(f"{Colors.DIM}{'─' * 63}{Colors.NC}")
    print()
    
    all_healthy = True
    
    for name, node in NODES.items():
        kanji = node["kanji"]
        print(f"  {Colors.CYAN}{kanji}{Colors.NC} {Colors.BOLD}{name.upper()}{Colors.NC}")
        
        # SSH Check
        ssh_ok = test_ssh_connection(name)
        if ssh_ok:
            print(f"     {Colors.GREEN}✓{Colors.NC} SSH connection")
        else:
            print(f"     {Colors.RED}✗{Colors.NC} SSH connection failed")
            all_healthy = False
            print()
            continue
        
        # Service Check
        service = node["service"]
        result = run_ssh(name, f"systemctl is-active {service}", capture=True)
        if result.stdout.strip() == "active":
            print(f"     {Colors.GREEN}✓{Colors.NC} Service {service} is running")
        else:
            print(f"     {Colors.RED}✗{Colors.NC} Service {service} is not running ({result.stdout.strip()})")
            all_healthy = False
        
        # HTTP Check (if not localhost or if we want to check API)
        port = node["port"]
        if name == "kokoro":
            url = f"http://localhost:{port}/status"
        else:
            url = f"http://{node['host']}:{port}/status"
        
        try:
            import urllib.request
            with urllib.request.urlopen(url, timeout=5) as response:
                if response.status == 200:
                    print(f"     {Colors.GREEN}✓{Colors.NC} API responding on port {port}")
                else:
                    print(f"     {Colors.YELLOW}!{Colors.NC} API returned {response.status}")
        except Exception as e:
            print(f"     {Colors.YELLOW}!{Colors.NC} API not responding ({type(e).__name__})")
        
        # Disk Space Check
        result = run_ssh(name, "df -h / | awk 'NR==2{print $5}'", capture=True)
        if result.returncode == 0:
            usage = result.stdout.strip()
            usage_int = int(usage.replace('%', '')) if usage else 0
            if usage_int > 90:
                print(f"     {Colors.RED}✗{Colors.NC} Disk usage: {usage} (CRITICAL)")
                all_healthy = False
            elif usage_int > 80:
                print(f"     {Colors.YELLOW}!{Colors.NC} Disk usage: {usage} (WARNING)")
            else:
                print(f"     {Colors.GREEN}✓{Colors.NC} Disk usage: {usage}")
        
        print()
    
    print(f"{Colors.DIM}{'─' * 63}{Colors.NC}")
    if all_healthy:
        print(f"{Colors.GREEN}All health checks passed!{Colors.NC}")
    else:
        print(f"{Colors.YELLOW}Some health checks failed. Review above.{Colors.NC}")


def cmd_deploy(node_name: str):
    """Deploy latest code to a node (placeholder for git pull + restart)."""
    if node_name not in NODES:
        print(f"{Colors.RED}✗ Unknown node: {node_name}{Colors.NC}")
        return
    
    node = NODES[node_name]
    kanji = node["kanji"]
    path = node["path"]
    service = node["service"]
    
    print_header()
    print(f"  {Colors.CYAN}{kanji}{Colors.NC} Deploying to {node_name.upper()}...")
    
    if not test_ssh_connection(node_name):
        print(f"     {Colors.RED}✗ Cannot connect to {node_name}{Colors.NC}")
        return
    
    # Pull latest code
    print(f"     {Colors.DIM}→ Pulling latest code...{Colors.NC}")
    result = run_ssh(node_name, f"cd {path} && git pull", capture=True)
    if result.returncode != 0:
        print(f"     {Colors.YELLOW}! Git pull: {result.stderr.strip()}{Colors.NC}")
    else:
        print(f"     {Colors.GREEN}✓ Code updated{Colors.NC}")
    
    # Restart service
    print(f"     {Colors.DIM}→ Restarting service...{Colors.NC}")
    result = run_ssh(node_name, f"sudo systemctl restart {service}", capture=True)
    if result.returncode == 0:
        print(f"     {Colors.GREEN}✓ Service restarted{Colors.NC}")
    else:
        print(f"     {Colors.RED}✗ Restart failed: {result.stderr.strip()}{Colors.NC}")


def main():
    parser = argparse.ArgumentParser(
        description="VOIGHT CLUSTER Control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s status              Show cluster status
  %(prog)s restart me          Restart ME node
  %(prog)s restart all         Restart all nodes
  %(prog)s logs te 100         Show last 100 lines of TE logs
  %(prog)s ssh me              SSH into ME node
  %(prog)s health              Full health check
"""
    )
    
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # status
    subparsers.add_parser("status", help="Show cluster status")
    
    # restart
    p_restart = subparsers.add_parser("restart", help="Restart service(s)")
    p_restart.add_argument("target", nargs="?", default="all", 
                           choices=["all", "me", "te", "kokoro"],
                           help="Node to restart (default: all)")
    
    # stop
    p_stop = subparsers.add_parser("stop", help="Stop service(s)")
    p_stop.add_argument("target", nargs="?", default="all",
                        choices=["all", "me", "te", "kokoro"],
                        help="Node to stop (default: all)")
    
    # start
    p_start = subparsers.add_parser("start", help="Start service(s)")
    p_start.add_argument("target", nargs="?", default="all",
                         choices=["all", "me", "te", "kokoro"],
                         help="Node to start (default: all)")
    
    # logs
    p_logs = subparsers.add_parser("logs", help="Show service logs")
    p_logs.add_argument("node", choices=["me", "te", "kokoro"], help="Node to show logs for")
    p_logs.add_argument("lines", nargs="?", type=int, default=50, help="Number of lines (default: 50)")
    
    # ssh
    p_ssh = subparsers.add_parser("ssh", help="Open SSH session to node")
    p_ssh.add_argument("node", choices=["kokoro", "me", "te"], help="Node to SSH into")
    
    # health
    subparsers.add_parser("health", help="Full health check")
    
    # deploy
    p_deploy = subparsers.add_parser("deploy", help="Deploy latest code to node")
    p_deploy.add_argument("node", choices=["me", "te", "kokoro"], help="Node to deploy to")
    
    args = parser.parse_args()
    
    if not args.command:
        # Default to status
        cmd_status()
        return
    
    if args.command == "status":
        cmd_status()
    elif args.command == "restart":
        cmd_restart(args.target)
    elif args.command == "stop":
        cmd_stop(args.target)
    elif args.command == "start":
        cmd_start(args.target)
    elif args.command == "logs":
        cmd_logs(args.node, args.lines)
    elif args.command == "ssh":
        cmd_ssh(args.node)
    elif args.command == "health":
        cmd_health()
    elif args.command == "deploy":
        cmd_deploy(args.node)


if __name__ == "__main__":
    main()

