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

# ros sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# catkin tools sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

# cuda toolkit sources
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin
sudo mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /"

# update from all sources
sudo apt update

# these could be in one line but separated for visibility for me
sudo apt install ros-melodic-desktop-full -y
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo apt install python-catkin-tools -y
sudo apt install cuda -y

# set up environment for ros
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# initialize rosdep
sudo rosdep init
rosdep update

# create and init catkin workspace
mkdir -p ~/$WORKSPACE/src
cd ~/$WORKSPACE
catkin init
catkin build

if [$2 == "full"];
  then
    cd ~/installation_scripts
    # Clone and build all of the packages used with the armada_workstation (skipping the robots we're not using for now)"
    ./setup_armada_manipulation_resources $WORKSPACE

    # Install ROS Qt Creator Plug-in and IDE
    echo "Firefox will now open a link to the QT creator ROS plugin Bionic (18.04) online installer download page."
    echo "Click yes on the prompt to download the file called <qtcreator-ros-bionic-latest-online-installer.run>"
    firefox https://qtcreator-ros.datasys.swri.edu/downloads/installers/bionic/qtcreator-ros-bionic-latest-online-installer.run
    echo "Follow ALL of the instructions under Qt Installer Procedure. Step 1 and 2 are above the image and cannot be skipped, don't skip any steps."
    firefox https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html#qt-installer-procedure
    echo "Follow the instructions at the bottom of the page under the section titled 'Testing Plugin' to connect your ros workspace and then build (hammer in bottom left)"
fi

