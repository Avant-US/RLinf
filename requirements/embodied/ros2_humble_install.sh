#!/bin/bash

# Configure ROS2 Humble apt source for Ubuntu 22.04 (Jammy) using USTC mirror,
# falling back to the official ROS2 repository if the mirror is unreachable.

# Check if apt is available
if ! command -v apt-get &> /dev/null; then
    echo "apt-get could not be found. This script is intended for Debian-based systems."
    exit 1
fi

# Check for sudo privileges
if ! sudo -n true 2>/dev/null; then
    # Check if already running as root
    if [ "$EUID" -eq 0 ]; then
        apt-get update -y
        apt-get install -y --no-install-recommends sudo
    else
        echo "This script requires sudo privileges. Please run as a user with sudo access."
        exit 1
    fi
fi

sudo apt-get update -y
sudo apt-get install -y --no-install-recommends \
    wget \
    curl \
    lsb-release \
    gnupg \
    locales

# Detect Ubuntu codename (e.g., jammy)
ubuntu_codename=""
if command -v lsb_release >/dev/null 2>&1; then
    ubuntu_codename=$(lsb_release -cs || true)
elif [ -f /etc/os-release ]; then
    ubuntu_codename=$(grep '^UBUNTU_CODENAME=' /etc/os-release | cut -d= -f2)
fi

if [ -z "$ubuntu_codename" ]; then
    echo "Failed to detect Ubuntu codename. Cannot configure ROS2 apt source automatically." >&2
    exit 1
fi

# ROS2 Humble is only released for Ubuntu 22.04 (Jammy)
if [ "$ubuntu_codename" != "jammy" ]; then
    echo "ROS2 Humble requires Ubuntu 22.04 (Jammy). Detected codename: '$ubuntu_codename'." >&2
    echo "Please use Ubuntu 22.04 to install ROS2 Humble." >&2
    exit 1
fi

# Idempotency: skip if already installed
if dpkg -l ros-humble-ros-base >/dev/null 2>&1; then
    echo "ros-humble-ros-base is already installed, skipping ROS2 Humble installation."
    exit 0
fi

# Ensure UTF-8 locale (required by ROS2)
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8

#@# REVERT TO ZH_CN
# sudo locale-gen zh_CN.UTF-8
# export LANG="zh_CN.UTF-8"
# export LC_ALL="zh_CN.UTF-8"
# export LANGUAGE="zh_CN:zh:en_US:en"
# export LC_ADDRESS="zh_CN.UTF-8"
# export LC_IDENTIFICATION="zh_CN.UTF-8"
# export LC_MEASUREMENT="zh_CN.UTF-8"
# export LC_MONETARY="zh_CN.UTF-8"
# export LC_NAME="zh_CN.UTF-8"
# export LC_NUMERIC="zh_CN.UTF-8"
# export LC_PAPER="zh_CN.UTF-8"
# export LC_TELEPHONE="zh_CN.UTF-8"
# export LC_TIME="zh_CN.UTF-8"

# Add ROS GPG key (same key ID used by both ROS1 and ROS2)
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Choose apt source: prefer USTC mirror, fall back to official
ros2_mirror="http://mirrors.ustc.edu.cn/ros2/ubuntu"
test_url="${ros2_mirror}/dists/${ubuntu_codename}/"

if ! curl -fsSL --head "$test_url" >/dev/null 2>&1; then
    echo "USTC ROS2 mirror does not appear to be reachable (tested: $test_url)." >&2
    echo "Falling back to official ROS2 repository." >&2
    ros2_mirror="http://packages.ros.org/ros2/ubuntu"
fi

source_line="deb ${ros2_mirror} ${ubuntu_codename} main"

# Check if the source already exists anywhere under /etc/apt
if sudo grep -Rqs -- "$source_line" /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null; then
    echo "ROS2 source already present in /etc/apt, skipping addition: $source_line"
else
    echo "$source_line" | sudo tee /etc/apt/sources.list.d/ros2-latest.list >/dev/null
    echo "Added ROS2 source: $source_line"
fi

sudo apt-get update -y
sudo apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    python3-colcon-common-extensions || {
    echo "Failed to install ROS2 Humble packages. Please check your apt sources or install manually." >&2
    exit 1
}

echo "ROS2 Humble installed successfully."
echo "To use ROS2: source /opt/ros/humble/setup.bash"
