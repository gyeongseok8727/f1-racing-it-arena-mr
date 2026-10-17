#!/usr/bin/env bash
# One-time Jetson setup (Ubuntu 22.04 / JetPack 6). Run from the repo root on the Jetson:
#   bash scripts/setup_jetson.sh
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"

if [ ! -d /opt/ros/humble ]; then
  echo "== installing ROS 2 Humble (ros-base)"
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt-get update
  sudo apt-get install -y ros-humble-ros-base python3-colcon-common-extensions
fi

echo "== ROS packages"
sudo apt-get install -y ros-humble-ackermann-msgs ros-humble-rplidar-ros ros-humble-tf2-ros \
  python3-serial python3-yaml python3-pip

echo "== racing python package (editable, uses system numpy/OpenCV from JetPack)"
python3 -m pip install --user -e "$REPO" --no-deps
python3 -c "import cv2, numpy; print('cv2', cv2.__version__, 'numpy', numpy.__version__)"
python3 -c "import cv2; cv2.aruco.ArucoDetector" 2>/dev/null \
  || echo "!! OpenCV lacks cv2.aruco.ArucoDetector (needs >= 4.7): python3 -m pip install --user opencv-contrib-python-headless"

echo "== serial permissions"
sudo usermod -aG dialout "$USER"

echo "== build"
source /opt/ros/humble/setup.bash
cd "$REPO/ros2_ws"
colcon build --symlink-install
echo
echo "done. Log out/in once (dialout group), then:"
echo "  source /opt/ros/humble/setup.bash && source $REPO/ros2_ws/install/setup.bash"
echo "  ros2 launch mr_racing race.launch.py"
