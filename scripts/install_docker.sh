#!/usr/bin/env bash
# Docker Engine + Compose plugin を JetPack 6.x (Ubuntu 22.04 / arm64) に導入する。
# nvidia-container-toolkit は既に apt 導入済みである前提
# (`dpkg -l | grep nvidia-container-toolkit` で確認)。
#
# 冪等。すでに導入済みなら apt は no-op、usermod も既に所属していれば変化なし。
#
# 実行: sudo bash scripts/install_docker.sh
#       (sudo は内部の各コマンドではなくスクリプト自体に付けても良い)

set -euo pipefail

# ----- 0. sudo 確認 -------------------------------------------------------
if [[ $EUID -ne 0 ]]; then
    echo "[install_docker] このスクリプトは sudo (root) で実行してください。" >&2
    echo "[install_docker]   sudo bash scripts/install_docker.sh" >&2
    exit 1
fi

# usermod 対象は sudo を呼び出した一般ユーザー。
TARGET_USER="${SUDO_USER:-$USER}"

# ----- 1. Docker apt リポジトリ ------------------------------------------
install -m 0755 -d /etc/apt/keyrings
if [[ ! -f /etc/apt/keyrings/docker.gpg ]]; then
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    chmod a+r /etc/apt/keyrings/docker.gpg
fi

UBUNTU_CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"
cat >/etc/apt/sources.list.d/docker.list <<EOF
deb [arch=arm64 signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu ${UBUNTU_CODENAME} stable
EOF

# ----- 2. Docker Engine + Compose plugin ---------------------------------
apt-get update
apt-get install -y \
    docker-ce docker-ce-cli containerd.io \
    docker-buildx-plugin docker-compose-plugin

# ----- 3. NVIDIA runtime を Docker に登録 ---------------------------------
# /etc/docker/daemon.json に "nvidia" runtime エントリを追記する。
# nvidia-container-toolkit が apt 導入済みであることが前提。
if command -v nvidia-ctk >/dev/null 2>&1; then
    nvidia-ctk runtime configure --runtime=docker
else
    echo "[install_docker] WARNING: nvidia-ctk が見つかりません。" >&2
    echo "[install_docker] 先に 'sudo apt install -y nvidia-container-toolkit' を実行してください。" >&2
fi

# ----- 4. docker グループへユーザー追加 -----------------------------------
if id -nG "$TARGET_USER" | grep -qw docker; then
    echo "[install_docker] $TARGET_USER は既に docker グループに所属しています。"
else
    usermod -aG docker "$TARGET_USER"
    echo "[install_docker] $TARGET_USER を docker グループに追加しました。"
    echo "[install_docker] 反映には一度ログアウト/ログインが必要です。"
fi

# ----- 5. 再起動 + 確認 ---------------------------------------------------
systemctl enable --now docker
systemctl restart docker

cat <<EOF

[install_docker] 完了。次の手順:
  1) いったんログアウト/ログイン (docker グループの反映)
  2) 動作確認:
       docker run --rm hello-world
       docker info | grep -i runtime          # nvidia が出ること
       docker run --rm --runtime=nvidia nvcr.io/nvidia/l4t-base:r36.2.0 \
           bash -c 'ls /usr/local/cuda && echo OK'
  3) その後、リポジトリ直下で:
       docker compose build
       docker compose up
EOF
