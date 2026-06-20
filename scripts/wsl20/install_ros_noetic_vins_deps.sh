#!/usr/bin/env bash
set -euo pipefail

# Install ROS Noetic and VINS-Fusion dependencies in Ubuntu 20.04 WSL.
# This script requires sudo. Run interactively inside Ubuntu-20.04:
#   cd /mnt/f/research/research/engineering/vins-lightglue-vio
#   bash scripts/wsl20/install_ros_noetic_vins_deps.sh

if [ "$(lsb_release -sc)" != "focal" ]; then
  echo "Expected Ubuntu 20.04 focal, got $(lsb_release -sc)" >&2
  exit 2
fi

sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release ca-certificates

if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]; then
  echo "deb http://packages.ros.org/ros/ubuntu focal main" | \
    sudo tee /etc/apt/sources.list.d/ros-latest.list >/dev/null
fi

sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
  --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt-get install -y \
  ros-noetic-desktop-full \
  python3-catkin-tools \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  python3-vcstool \
  build-essential \
  cmake \
  git \
  wget \
  unzip \
  libeigen3-dev \
  libopencv-dev \
  libceres-dev \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-tf \
  ros-noetic-message-filters \
  ros-noetic-pcl-ros \
  ros-noetic-rviz

if ! grep -q "/opt/ros/noetic/setup.bash" "$HOME/.bashrc"; then
  echo "source /opt/ros/noetic/setup.bash" >> "$HOME/.bashrc"
fi

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update

echo "ROS Noetic/VINS dependencies installed. Open a new shell or run:"
echo "source /opt/ros/noetic/setup.bash"
