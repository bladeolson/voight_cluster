#!/bin/zsh
#
# VOIGHT CLUSTER SSH Setup
# =========================
# Sets up passwordless SSH authentication to all cluster nodes.
# Run this from your control station (the machine you use to manage the cluster).
#
# Usage:
#   ./setup_ssh.sh           # Setup SSH for all nodes
#   ./setup_ssh.sh me        # Setup SSH for ME node only
#   ./setup_ssh.sh te        # Setup SSH for TE node only
#   ./setup_ssh.sh test      # Test SSH connectivity to all nodes
#

set -e

# ANSI colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Node definitions
KOKORO_USER="bladeolson"
KOKORO_HOST="kokoro.local"
KOKORO_ALT_IP="10.0.0.205"
KOKORO_SERVICE="zen-kokoro-node"
KOKORO_KANJI="心"

ME_USER="bladeolson"
ME_HOST="me.local"
ME_ALT_IP="10.0.0.30"
ME_SERVICE="zen-me-node"
ME_KANJI="目"

TE_USER="bladeolson"
TE_HOST="te.local"
TE_ALT_IP="10.0.0.242"
TE_SERVICE="zen-te-node"
TE_KANJI="手"

# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------

print_header() {
    echo ""
    echo "${CYAN}╔═══════════════════════════════════════════════════════════════╗${NC}"
    echo "${CYAN}║${NC}          ${YELLOW}VOIGHT CLUSTER${NC} - SSH Setup Utility                  ${CYAN}║${NC}"
    echo "${CYAN}║${NC}                   心 → 目 → 手                               ${CYAN}║${NC}"
    echo "${CYAN}╚═══════════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

success() {
    echo "  ${GREEN}✓${NC} $1"
}

error() {
    echo "  ${RED}✗${NC} $1"
}

warn() {
    echo "  ${YELLOW}!${NC} $1"
}

info() {
    echo "  ${CYAN}→${NC} $1"
}

# -----------------------------------------------------------------------------
# SSH Key Management
# -----------------------------------------------------------------------------

# SSH key to use for cluster (prefer zen-specific key)
SSH_KEY="$HOME/.ssh/id_ed25519_zen"

ensure_ssh_key() {
    echo "\n${YELLOW}[1/3]${NC} Checking SSH keys..."
    
    # Check for zen-specific key first
    if [[ -f "${SSH_KEY}.pub" ]]; then
        success "SSH key exists: ${SSH_KEY}.pub"
        return 0
    fi
    
    # Check for default keys
    if [[ -f "$HOME/.ssh/id_ed25519.pub" ]]; then
        SSH_KEY="$HOME/.ssh/id_ed25519"
        success "SSH key exists: ${SSH_KEY}.pub"
        return 0
    fi
    
    if [[ -f "$HOME/.ssh/id_rsa.pub" ]]; then
        SSH_KEY="$HOME/.ssh/id_rsa"
        success "SSH key exists: ${SSH_KEY}.pub"
        return 0
    fi
    
    warn "No SSH key found. Generating new Ed25519 key for VOIGHT CLUSTER..."
    
    mkdir -p "$HOME/.ssh"
    chmod 700 "$HOME/.ssh"
    
    ssh-keygen -t ed25519 -f "$SSH_KEY" -N "" -C "voight_cluster"
    
    if [[ -f "${SSH_KEY}.pub" ]]; then
        success "Generated new SSH key: ${SSH_KEY}.pub"
    else
        error "Failed to generate SSH key"
        exit 1
    fi
}

ensure_ssh_config() {
    local config_file="$HOME/.ssh/config"
    
    # Check if cluster hosts are already configured
    if grep -q "# VOIGHT CLUSTER" "$config_file" 2>/dev/null; then
        info "SSH config already has VOIGHT CLUSTER entries"
        return 0
    fi
    
    info "Adding VOIGHT CLUSTER hosts to SSH config..."
    
    cat >> "$config_file" << EOF

# VOIGHT CLUSTER
Host kokoro kokoro.local
    HostName kokoro.local
    User bladeolson
    IdentityFile ${SSH_KEY}
    StrictHostKeyChecking no

Host me me.local
    HostName me.local
    User bladeolson
    IdentityFile ${SSH_KEY}
    StrictHostKeyChecking no

Host te te.local
    HostName te.local
    User bladeolson
    IdentityFile ${SSH_KEY}
    StrictHostKeyChecking no
EOF
    
    chmod 600 "$config_file"
    success "Added cluster hosts to SSH config"
}

# -----------------------------------------------------------------------------
# Node Connection Testing
# -----------------------------------------------------------------------------

test_ssh_connection() {
    local user=$1
    local host=$2
    local alt_ip=$3
    
    # First try hostname.local
    if ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${user}@${host}" "echo ok" &>/dev/null; then
        return 0
    fi
    
    # Try alternate IP
    if [[ -n "$alt_ip" ]]; then
        if ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${user}@${alt_ip}" "echo ok" &>/dev/null; then
            return 0
        fi
    fi
    
    return 1
}

can_reach_node() {
    local host=$1
    local alt_ip=$2
    
    # Try ping
    if ping -c 1 -W 2 "$host" &>/dev/null; then
        return 0
    fi
    
    if [[ -n "$alt_ip" ]] && ping -c 1 -W 2 "$alt_ip" &>/dev/null; then
        return 0
    fi
    
    return 1
}

# -----------------------------------------------------------------------------
# SSH Key Copy
# -----------------------------------------------------------------------------

copy_ssh_key_to_node() {
    local node_name=$1
    local user=$2
    local host=$3
    local alt_ip=$4
    local kanji=$5
    
    echo ""
    echo "  ${CYAN}${kanji}${NC} Setting up SSH for ${YELLOW}${node_name:u}${NC} node..."
    
    # Check if we can already connect
    if test_ssh_connection "$user" "$host" "$alt_ip"; then
        success "SSH already configured for $node_name (passwordless access works)"
        return 0
    fi
    
    # Check if node is reachable
    if ! can_reach_node "$host" "$alt_ip"; then
        error "Cannot reach $node_name ($host or $alt_ip)"
        warn "Make sure the node is powered on and connected to the network"
        return 1
    fi
    
    info "Node is reachable. Copying SSH key..."
    info "You will be prompted for the password for ${user}@${node_name}"
    echo ""
    
    # Try hostname.local first
    if ssh-copy-id -i "${SSH_KEY}.pub" -o StrictHostKeyChecking=no -o ConnectTimeout=10 "${user}@${host}" 2>/dev/null; then
        success "SSH key copied to ${user}@${host}"
        return 0
    fi
    
    # Try alternate IP
    if [[ -n "$alt_ip" ]]; then
        warn "Hostname failed, trying IP address ($alt_ip)..."
        if ssh-copy-id -i "${SSH_KEY}.pub" -o StrictHostKeyChecking=no -o ConnectTimeout=10 "${user}@${alt_ip}" 2>/dev/null; then
            success "SSH key copied to ${user}@${alt_ip}"
            return 0
        fi
    fi
    
    error "Failed to copy SSH key to $node_name"
    return 1
}

# -----------------------------------------------------------------------------
# Test Node
# -----------------------------------------------------------------------------

test_node() {
    local node_name=$1
    local user=$2
    local host=$3
    local alt_ip=$4
    local service=$5
    local kanji=$6
    
    echo -n "  ${CYAN}${kanji}${NC} ${node_name:u}: "
    
    if test_ssh_connection "$user" "$host" "$alt_ip"; then
        echo "${GREEN}✓ Connected${NC}"
        
        # Determine which host worked
        local target="${user}@${host}"
        if ! ssh -o BatchMode=yes -o ConnectTimeout=3 -o StrictHostKeyChecking=no "$target" "echo ok" &>/dev/null; then
            target="${user}@${alt_ip}"
        fi
        
        # Get additional info
        local hostname=$(ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$target" "hostname" 2>/dev/null || echo "?")
        local uptime=$(ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$target" "uptime -p" 2>/dev/null || echo "?")
        local service_status=$(ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=no "$target" "systemctl is-active $service 2>/dev/null" 2>/dev/null || echo "unknown")
        
        echo "       Host: ${hostname}"
        echo "       Uptime: ${uptime}"
        echo "       Service ($service): ${service_status}"
        return 0
    else
        echo "${RED}✗ Failed${NC}"
        return 1
    fi
}

# -----------------------------------------------------------------------------
# Test All Nodes
# -----------------------------------------------------------------------------

test_all_nodes() {
    echo "\n${YELLOW}[3/3]${NC} Testing SSH connectivity..."
    echo ""
    
    local all_ok=true
    
    # Test KOKORO
    if ! test_node "kokoro" "$KOKORO_USER" "$KOKORO_HOST" "$KOKORO_ALT_IP" "$KOKORO_SERVICE" "$KOKORO_KANJI"; then
        all_ok=false
    fi
    echo ""
    
    # Test ME
    if ! test_node "me" "$ME_USER" "$ME_HOST" "$ME_ALT_IP" "$ME_SERVICE" "$ME_KANJI"; then
        all_ok=false
    fi
    echo ""
    
    # Test TE
    if ! test_node "te" "$TE_USER" "$TE_HOST" "$TE_ALT_IP" "$TE_SERVICE" "$TE_KANJI"; then
        all_ok=false
    fi
    echo ""
    
    if $all_ok; then
        echo "${GREEN}All nodes connected successfully!${NC}"
    else
        echo "${YELLOW}Some nodes failed to connect.${NC}"
        echo "Run this script again to retry SSH key copy."
    fi
}

# -----------------------------------------------------------------------------
# Main Setup
# -----------------------------------------------------------------------------

setup_kokoro() {
    ensure_ssh_key
    ensure_ssh_config
    echo "\n${YELLOW}[2/3]${NC} Copying SSH key to KOKORO..."
    copy_ssh_key_to_node "kokoro" "$KOKORO_USER" "$KOKORO_HOST" "$KOKORO_ALT_IP" "$KOKORO_KANJI"
    echo "\n${YELLOW}[3/3]${NC} Verifying connection..."
    test_node "kokoro" "$KOKORO_USER" "$KOKORO_HOST" "$KOKORO_ALT_IP" "$KOKORO_SERVICE" "$KOKORO_KANJI"
}

setup_me() {
    ensure_ssh_key
    ensure_ssh_config
    echo "\n${YELLOW}[2/3]${NC} Copying SSH key to ME..."
    copy_ssh_key_to_node "me" "$ME_USER" "$ME_HOST" "$ME_ALT_IP" "$ME_KANJI"
    echo "\n${YELLOW}[3/3]${NC} Verifying connection..."
    test_node "me" "$ME_USER" "$ME_HOST" "$ME_ALT_IP" "$ME_SERVICE" "$ME_KANJI"
}

setup_te() {
    ensure_ssh_key
    ensure_ssh_config
    echo "\n${YELLOW}[2/3]${NC} Copying SSH key to TE..."
    copy_ssh_key_to_node "te" "$TE_USER" "$TE_HOST" "$TE_ALT_IP" "$TE_KANJI"
    echo "\n${YELLOW}[3/3]${NC} Verifying connection..."
    test_node "te" "$TE_USER" "$TE_HOST" "$TE_ALT_IP" "$TE_SERVICE" "$TE_KANJI"
}

setup_all_nodes() {
    ensure_ssh_key
    ensure_ssh_config
    
    echo "\n${YELLOW}[2/3]${NC} Copying SSH keys to nodes..."
    
    copy_ssh_key_to_node "kokoro" "$KOKORO_USER" "$KOKORO_HOST" "$KOKORO_ALT_IP" "$KOKORO_KANJI"
    copy_ssh_key_to_node "me" "$ME_USER" "$ME_HOST" "$ME_ALT_IP" "$ME_KANJI"
    copy_ssh_key_to_node "te" "$TE_USER" "$TE_HOST" "$TE_ALT_IP" "$TE_KANJI"
    
    test_all_nodes
}

# -----------------------------------------------------------------------------
# Entry Point
# -----------------------------------------------------------------------------

print_header

case "${1:-all}" in
    kokoro|KOKORO)
        setup_kokoro
        ;;
    me|ME)
        setup_me
        ;;
    te|TE)
        setup_te
        ;;
    test)
        test_all_nodes
        ;;
    all|"")
        setup_all_nodes
        ;;
    -h|--help|help)
        echo "Usage: $0 [all|kokoro|me|te|test]"
        echo ""
        echo "Commands:"
        echo "  all    - Setup SSH for all nodes (default)"
        echo "  kokoro - Setup SSH for KOKORO node only"
        echo "  me     - Setup SSH for ME node only"
        echo "  te     - Setup SSH for TE node only"
        echo "  test   - Test SSH connectivity to all nodes"
        exit 0
        ;;
    *)
        echo "Usage: $0 [all|kokoro|me|te|test]"
        echo ""
        echo "Commands:"
        echo "  all    - Setup SSH for all nodes (default)"
        echo "  kokoro - Setup SSH for KOKORO node only"
        echo "  me     - Setup SSH for ME node only"
        echo "  te     - Setup SSH for TE node only"
        echo "  test   - Test SSH connectivity to all nodes"
        exit 1
        ;;
esac

echo ""
echo "${CYAN}═══════════════════════════════════════════════════════════════${NC}"
echo ""
