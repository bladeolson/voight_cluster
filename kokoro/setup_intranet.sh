#!/bin/bash
# setup_intranet.sh
# Installs Caddy and configures it as a reverse proxy for Kokoro

# Don't exit immediately on error for the update check
set +e

echo "=== VOIGHT CLUSTER Intranet Setup ==="
echo "Setting up https://kokoro.local/ ..."

# Fix known broken NVIDIA repo if present (blocks apt update)
if [ -f /etc/apt/sources.list.d/nvidia-container-toolkit.list ]; then
    if grep -q "^deb" /etc/apt/sources.list.d/nvidia-container-toolkit.list; then
        echo "Disabling broken NVIDIA repository config to allow updates..."
        sudo sed -i 's/^deb/#deb/' /etc/apt/sources.list.d/nvidia-container-toolkit.list
    fi
fi

# Re-enable exit on error
set -e

# 1. Install Caddy (if not present)
if ! command -v caddy &> /dev/null; then
    echo "Installing Caddy..."
    sudo apt install -y debian-keyring debian-archive-keyring apt-transport-https curl
    
    # Add Caddy repo key
    curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/gpg.key' | sudo gpg --dearmor --yes --status-fd 1 -o /usr/share/keyrings/caddy-stable-archive-keyring.gpg || true
    
    # Add Caddy repo
    curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/debian.deb.txt' | sudo tee /etc/apt/sources.list.d/caddy-stable.list
    
    echo "Updating package lists..."
    sudo apt update
    
    echo "Installing Caddy package..."
    sudo apt install -y caddy
else
    echo "Caddy is already installed."
fi

# 2. Configure Caddy
echo "Configuring Caddy..."
sudo cp /home/bladeolson/voight_cluster/kokoro/Caddyfile /etc/caddy/Caddyfile

# 3. Restart Caddy
echo "Restarting Caddy service..."
sudo systemctl restart caddy

# 4. Restart Kokoro Node (to update internal URL refs)
echo "Restarting Kokoro Node..."
sudo systemctl restart zen-kokoro-node

echo ""
echo "=== Setup Complete ==="
echo "Dashboard available at: https://kokoro.local/"
echo "Note: You may need to accept the self-signed certificate warning on your first visit,"
echo "      or install the Caddy root certificate from this machine to your browser."
