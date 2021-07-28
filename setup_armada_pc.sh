#!/bin/bash

WORKSPACE=$1
VER=$2

if [ -z $1 ]
  then
    echo "No desired workspace name supplied, please re-run the script"
    echo "Your command should look like ./setup_armada_pc.sh <workspace_name>"
    echo "For example; ./setup_armada_pc.sh catkin_ws full"
    exit
fi

if [ -z $2 ]
  then
    echo "Please indicate if you want the <full> or <basic> installation"
    echo "Your command should look like ./setup_armada_pc.sh <workspace_name>"
    echo "For example; ./setup_armada_pc.sh catkin_ws full"
    echo "or: ./setup_armada_pc.sh catkin_ws basic"
    echo "basic installation will install ROS, catkin tools and cuda toolkit (most recent for 18.04)"
    echo "full installation will install the basic options and the armada_workstation, pick_and_place, robot, realsense and GPD packages, libraries, and files"
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

printmsg "Setting up ros resource sources list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

printmsg "Setting up catkin tools resource sources list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

printmsg "Setting up cuda toolkit resource sources list"
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /"

printmsg "Update your system to include all of the newly added source lists"
sudo apt update

printmsg "Install ros melodic, python rosdep and catkin tools, and cuda"
sudo apt install ros-melodic-desktop-full -y
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo apt install python-catkin-tools -y
sudo apt install cuda -y

printmsg "Add a line in your .bashrc file to always source your ros workspace upon opening"
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

printmsg "Initialize rosdep"
sudo rosdep init
rosdep update

printmsg "Create and init a catkin workspace according to the value you entered when running the script"
mkdir -p ~/$WORKSPACE/src
cd ~/$WORKSPACE
source /opt/ros/melodic/setup.bash
catkin init
source /devel/setup.bash
catkin build

if [$2 == "full"];
  then
    printmsg "The script will now continue installing files required for full armada_workstation functionality"
    cd ~/installation_scripts
    ./setup_armada_manipulation_resources.sh $WORKSPACE

    printmsg "This script will guide you through the process of installing the Qt Creator IDE with a ROS plugin"
    printmsg"Firefox will now open a link to the QT creator ROS plugin Bionic (18.04) online installer download page."
    printmsg "Click yes on the prompt to download the file called <qtcreator-ros-bionic-latest-online-installer.run>"
    firefox https://qtcreator-ros.datasys.swri.edu/downloads/installers/bionic/qtcreator-ros-bionic-latest-online-installer.run
    printmsg "Follow ALL of the instructions under Qt Installer Procedure. Step 1 and 2 are above the image and cannot be skipped, don't skip any steps."
    firefox https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html#qt-installer-procedure
    printmsg "Follow the instructions at the bottom of the page under the section titled 'Testing Plugin' to connect your ros workspace and then build (hammer in bottom left)"
fi

