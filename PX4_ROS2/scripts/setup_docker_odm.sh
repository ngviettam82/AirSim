#!/bin/bash
# Install Docker Engine (not Desktop) + pull the ODM image.
# See docs/ops-playbook.md S3 for Engine-vs-Desktop rationale.
# Usage: sudo -v && bash scripts/setup_docker_odm.sh
set -euo pipefail

echo "=== 1/5 prerequisites ==="
sudo apt-get update
sudo apt-get install -y ca-certificates curl

echo "=== 2/5 Docker apt repository ==="
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt-get update

echo "=== 3/5 install engine ==="
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
                        docker-buildx-plugin docker-compose-plugin

echo "=== 4/5 enable service + non-root access ==="
# systemd is already PID 1 here (see ops-playbook S3).
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
sudo systemctl is-active docker

echo "=== 5/5 pull ODM image (~3-4 GB) ==="
sudo docker pull opendronemap/odm
sudo docker image ls opendronemap/odm

echo
echo "DONE. Group membership needs a WSL restart to take effect:"
echo "  (from Windows)  wsl --shutdown"
echo "After restart, verify without sudo:  docker run --rm hello-world"
