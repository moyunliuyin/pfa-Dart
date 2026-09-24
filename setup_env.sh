#!/usr/bin/env bash
# DartSystem 环境部署脚本（Linux）

set -euo pipefail

cd "$(dirname "$0")"

PYTHON_BIN="${PYTHON_BIN:-python3}"

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "错误: 未找到 $PYTHON_BIN，请先安装 Python 3.10+" >&2
  exit 1
fi

echo "[1/4] 创建虚拟环境 .venv（若不存在）"
if [[ ! -d .venv ]]; then
  "$PYTHON_BIN" -m venv .venv
fi

echo "[2/4] 激活虚拟环境"
# shellcheck disable=SC1091
source .venv/bin/activate

echo "[3/4] 安装 Python 依赖"
python -m pip install --upgrade pip
pip install -r requirements.txt

echo "[4/4] 检查并安装系统依赖"
MISSING_PKGS=()

SYSTEM_PKGS=(
  libxcb-cursor0
  libxkbcommon-x11-0
  libgl1
  libglib2.0-0
  xvfb
  x11-utils
  x11-xserver-utils
  ffmpeg
  vlc
  gstreamer1.0-libav
  gstreamer1.0-plugins-good
  gstreamer1.0-plugins-bad
  gstreamer1.0-plugins-ugly
  libavcodec-extra
)

for pkg in "${SYSTEM_PKGS[@]}"; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    echo "- $pkg: 已安装"
  else
    echo "- $pkg: 未安装"
    MISSING_PKGS+=("$pkg")
  fi
done

if [[ ${#MISSING_PKGS[@]} -gt 0 ]]; then
  if command -v apt-get >/dev/null 2>&1; then
    echo ""
    echo "正在安装缺失系统依赖: ${MISSING_PKGS[*]}"
    sudo apt-get update
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${MISSING_PKGS[@]}"
  else
    echo "错误: 检测到缺失系统依赖，但当前系统没有 apt-get。" >&2
    echo "请手动安装: ${MISSING_PKGS[*]}" >&2
    exit 1
  fi
fi

for group in video render; do
  if getent group "$group" >/dev/null 2>&1; then
    if [[ ${SUDO_USER:-} ]]; then
      sudo usermod -aG "$group" "$SUDO_USER"
      echo "- 已将 $SUDO_USER 加入 $group 组"
    elif [[ $EUID -ne 0 ]]; then
      sudo usermod -aG "$group" "$USER"
      echo "- 已将 $USER 加入 $group 组"
    fi
    if id dartsystem >/dev/null 2>&1; then
      sudo usermod -aG "$group" dartsystem
      echo "- 已将 dartsystem 加入 $group 组"
    fi
  fi
done

if [[ -d /opt/MVS/lib/64 ]]; then
  echo "- MVS 运行库: 已检测到 /opt/MVS/lib/64"
else
  echo "- MVS 运行库: 未检测到 /opt/MVS/lib/64（海康相机模式需要先安装 MVS SDK）"
fi

echo ""
echo "环境部署完成。"
echo "启动命令: ./run.sh"
