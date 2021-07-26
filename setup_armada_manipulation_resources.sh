#!/bin/bash

WORKSPACE=$1

if [ -z $1 ]
  then
    echo "No desired workspace name supplied, please re-run the script"
    echo "Your command should look like ./setup_armada_pc.sh <workspace_name>"
    echo "For example; ./setup_armada_pc.sh catkin_ws full"
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

# install moveit (these packages have useful reference code and deps we can use for other packages)
sudo apt install ros-$ROS_DISTRO-moveit -y
cd ~/$WORKSPACE/src
git clone -b $ROS_DISTRO-devel https://github.com/ros-planning/moveit_tutorials.git
git clone -b $ROS_DISTRO-devel https://github.com/ros-planning/panda_moveit_config.git

# install gpd library
cd ~/$WORKSPACE/src
git clone https://github.com/atenpas/gpd
sed -i -e 's/PCL 1.9 REQUIRED/PCL REQUIRED/g' ~/$WORKSPACE/src/gpd/CMakeLists.txt
cd gpd
mkdir build && cd build
cmake ..
make -j
sudo make install

# clone and install gpd_ros
cd ~/$WORKSPACE/src
git clone -b master https://github.com/atenpas/gpd_ros
sed -i -e 's/PCL 1.9 REQUIRED/PCL REQUIRED/g' ~/$WORKSPACE/src/gpd_ros/CMakeLists.txt

# install any dependencies then build
build

# clone uml_robotics/uml_hri_nerve_armada_workstation package
cd ~/$WORKSPACE/src
git clone -b master https://github.com/uml-robotics/uml_hri_nerve_armada_workstation.git
# clone uml_robotics/uml_hri_nerve_pick_and_place package
git clone -b master https://github.com/uml-robotics/uml_hri_nerve_pick_and_place.git

# install any dependencies then build
build

# clone uml_robotics/universal_robot package
cd ~/$WORKSPACE/src
git clone -b dev/bflynn https://github.com/uml-robotics/universal_robot.git
#clone uml_robotics/kinova-ros package\
git clone -b dev/bflynn https://github.com/uml-robotics/kinova-ros.git

# install any dependencies then build
build

# clone and install ros_kortex package and libraries, install deps and build
sudo apt install python3 python3-pip -y
sudo python3 -m pip install conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
cd ~/$WORKSPACE/src
git clone -b dev/workstation_sim https://github.com/uml-robotics/ros_kortex.git
build

## clone librealsense package
cd ~/$WORKSPACE/src
git clone -b master https://github.com/IntelRealSense/librealsense.git

## install necessary libraries
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev -y

## add keyserver to list of repositories
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

## install additional libraries
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y

## update, upgrade, install deps and build
sudo apt-get update && sudo apt-get upgrade -y
build

## run realsense camera scripts to add cameras to udev rules and patch software
## your cameras WILL NOT WORK without allowing this step to happen
cd ~/$WORKSPACE/src/librealsense
./scripts/setup_udev_rules.sh
printwrn "This script will require you to enter your password at some point"
./scripts/patch-realsense-ubuntu-lts.sh

## clone realsense package (not realsense-ros package) then clone ddynamic_reconfigure package into realsense package
cd ~/$WORKSPACE/src/
git clone -b development https://github.com/doronhi/realsense.git
cd realsense
git clone -b kinetic-devel https://github.com/pal-robotics/ddynamic_reconfigure.git

## install rgbd_launch/rs_camera package for realsense cameras, install deps and build
sudo apt-get install ros-$ROS_DISTRO-rgbd-launch -y
build

# clone ros_kortex_vision package and install libraries, update deps and build
sudo apt install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base -y
cd ~/$WORKSPACE/src/
git clone -b master https://github.com/Kinovarobotics/ros_kortex_vision.git
build

mkdir ~/.gazebo/models
cd ~/.gazebo/models
git clone -b https://github.com/osrf/gazebo_models.git
