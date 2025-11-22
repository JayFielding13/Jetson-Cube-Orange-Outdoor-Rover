#!/bin/bash
set -e

echo "=========================================="
echo "ROS 2 Humble + Gazebo Installation Script"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Step 1: Setting up ROS 2 repository${NC}"
# Ensure software-properties-common is installed
sudo apt update
sudo apt install -y software-properties-common

# Add ROS 2 GPG key
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "${GREEN}✓ ROS 2 repository added${NC}"
echo ""

echo -e "${YELLOW}Step 2: Updating package lists${NC}"
sudo apt update
echo -e "${GREEN}✓ Package lists updated${NC}"
echo ""

echo -e "${YELLOW}Step 3: Installing ROS 2 Humble Desktop (includes RViz2)${NC}"
echo "This will download ~1.5GB and take 5-15 minutes..."
sudo apt install -y ros-humble-desktop
echo -e "${GREEN}✓ ROS 2 Humble Desktop installed${NC}"
echo ""

echo -e "${YELLOW}Step 4: Installing development tools${NC}"
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-rosdep
echo -e "${GREEN}✓ Development tools installed${NC}"
echo ""

echo -e "${YELLOW}Step 5: Initializing rosdep${NC}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
echo -e "${GREEN}✓ rosdep initialized${NC}"
echo ""

echo -e "${YELLOW}Step 6: Installing Gazebo Garden and ROS 2 integration${NC}"
sudo apt install -y ros-humble-ros-gz
echo -e "${GREEN}✓ Gazebo and ROS 2 integration installed${NC}"
echo ""

echo -e "${YELLOW}Step 7: Setting up environment${NC}"
# Add to bashrc if not already there
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 Humble setup" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo -e "${GREEN}✓ Added ROS 2 setup to ~/.bashrc${NC}"
else
    echo -e "${GREEN}✓ ROS 2 already in ~/.bashrc${NC}"
fi
echo ""

echo -e "${YELLOW}Step 8: Installing additional useful packages${NC}"
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins
echo -e "${GREEN}✓ Additional packages installed${NC}"
echo ""

echo "=========================================="
echo -e "${GREEN}Installation Complete!${NC}"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Source the environment: source /opt/ros/humble/setup.bash"
echo "2. Or open a new terminal (it will auto-source)"
echo "3. Test RViz2: rviz2"
echo "4. Test Gazebo: gz sim"
echo ""
echo "Total disk space used: ~4-5 GB"
echo ""
