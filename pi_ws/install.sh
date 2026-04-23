#!/bin/bash
set -e

echo "== Fix Debian Buster archive repos =="

SRC="/etc/apt/sources.list"
BACKUP="/etc/apt/sources.list.bak.$(date +%s)"

# Backup
echo "[+] Backup sources.list → $BACKUP"
sudo cp "$SRC" "$BACKUP"

# Xóa các dòng buster cũ từ deb.debian.org
echo "[+] Removing old deb.debian.org buster entries..."
sudo sed -i '/deb .*debian\.org.*buster/d' "$SRC"
sudo sed -i '/deb .*security\.debian\.org.*buster/d' "$SRC"

# Thêm nếu chưa tồn tại
add_line_if_missing() {
    LINE="$1"
    if ! grep -Fxq "$LINE" "$SRC"; then
        echo "[+] Adding: $LINE"
        echo "$LINE" | sudo tee -a "$SRC" > /dev/null
    else
        echo "[=] Already exists: $LINE"
    fi
}

add_line_if_missing "deb http://archive.debian.org/debian buster main contrib non-free"
add_line_if_missing "deb http://archive.debian.org/debian-security buster/updates main"

# Update
echo "[+] Running apt update (ignore expiry)..."
sudo apt update -o Acquire::Check-Valid-Until=false

echo "== Done! =="
sudo apt-get update
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install  -y ros-noetic-tf-conversions
sudo apt-get install -y ros-noetic-teleop-twist-keyboard
sudo apt-get install -y ros-noetic-diagnostic-updater
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-map-server
sudo apt-get install -y ros-noetic-slam-gmapping
sudo apt-get install -y ros-noetic-dwa-local-planner
sudo apt-get install -y ros-noetic-smach
