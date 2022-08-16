#!/bin/bash

WORKSPACE=$1
ROS_DISTRO=$2


if [ -z $1 ]
  then
    echo "No desired workspace name supplied, please re-run the script"
    echo "Your command should look like ./setup_armada_pc.sh <workspace_name> <ros_distribution>"
    echo "For example; ./setup_armada_pc.sh catkin_ws melodic"
    exit
fi

if [ -z $2 ]
  then
    echo "No desired ros distribution supplied, please re-run the script"
    echo "Your command should look like ./setup_armada_pc.sh <workspace_name> <ros_distribution>"
    echo "For example; ./setup_armada_pc.sh catkin_ws melodic"
    exit
fi

# https://en.wikipedia.org/wiki/ANSI_escape_code
# https://stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

build() {
    # install any dependencies then build
    cd ~/$WORKSPACE
    source devel/setup.bash
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    catkin build
    source devel/setup.bash
}

printmsg() {
    echo -e "${GREEN}$1${NC}"
}

printwrn() {
    echo -e "${RED}$1${NC}"
}

# setup resources
printmsg "Setting up ros resource sources list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

printmsg "Setting up catkin tools resource sources list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

sudo apt update

printmsg "Install ros melodic, python, rosdep, and catkin tools"
sudo apt install ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools -y

printmsg "Add a line in your .bashrc file to always source your ros workspace upon opening"
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
echo "clear" >> ~/.bashrc
source ~/.bashrc

printmsg "Initialize rosdep"
sudo rosdep init
rosdep update

printmsg "Create and init a catkin workspace according to the value you entered when running the script"
mkdir -p ~/$WORKSPACE/src
cd ~/$WORKSPACE
source /opt/ros/melodic/setup.bash
catkin init
catkin build
source /devel/setup.bash

printmsg "This script will guide you to the page with cuda toolkit installation instructions, follow the installation instructions (network installer) for your operating system"
firefox https://developer.nvidia.com/cuda-downloads

printmsg "This script will guide you through the process of installing the Qt Creator IDE with a ROS plugin"
printmsg "This step is optional, you can use a different IDE if desired"
firefox https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html#installation

