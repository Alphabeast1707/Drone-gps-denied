#!/bin/bash
# ═══════════════════════════════════════════════
# Raspberry Pi 4 — GPS-Denied Drone Vision Setup
# ═══════════════════════════════════════════════
# Run this on your Raspberry Pi:
#   chmod +x setup_pi.sh
#   ./setup_pi.sh

set -e

echo "══════════════════════════════════════════"
echo " GPS-Denied Drone Vision — Pi Setup"
echo "══════════════════════════════════════════"

# ── 1. System update ──
echo ""
echo "[1/5] Updating system packages..."
sudo apt update && sudo apt upgrade -y

# ── 2. Install build tools + OpenCV ──
echo ""
echo "[2/5] Installing build tools, OpenCV, and dependencies..."
sudo apt install -y \
    git cmake build-essential \
    python3-opencv python3-pip \
    libatlas-base-dev \
    libcamera-dev \
    python3-libcamera \
    python3-picamera2

# ── 3. Install Python packages ──
echo ""
echo "[3/5] Installing Python packages..."
pip3 install --upgrade pip
pip3 install numpy opencv-python pupil-apriltags

# ── 4. Build AprilTag C library (optional, for best performance) ──
echo ""
echo "[4/5] Building AprilTag C library..."
if [ ! -d "$HOME/apriltag" ]; then
    cd "$HOME"
    git clone https://github.com/AprilRobotics/apriltag.git
    cd apriltag
    mkdir -p build && cd build
    cmake ..
    make -j4
    sudo make install
    sudo ldconfig
    echo "  ✅ AprilTag C library installed."
else
    echo "  ⏭  AprilTag already cloned, skipping."
fi

# ── 5. Test camera ──
echo ""
echo "[5/5] Testing camera..."
echo "  Running libcamera-hello for 3 seconds..."
timeout 3 libcamera-hello --nopreview 2>/dev/null && echo "  ✅ Camera works!" || echo "  ⚠  Camera test failed. Enable it with: sudo raspi-config → Interface → Camera"

echo ""
echo "══════════════════════════════════════════"
echo " ✅  Setup complete!"
echo ""
echo " Next steps:"
echo "   1. Print an AprilTag (tag36h11 family)"
echo "   2. Run:  python3 detect_tag.py"
echo "   3. Then: python3 pose_estimation.py"
echo "   4. Then: python3 landing_controller.py"
echo "══════════════════════════════════════════"
