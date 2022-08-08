#!/bin/bash

WORKSPACE=$1
ROS_DISTRO=$2

if [ -z $1 ]
  then
    echo "No desired workspace name supplied, please re-run the script"
    echo "Your command should look like ./setup_armada_pc.sh <workspace_name> <ros_distro>"
    echo "For example; ./setup_armada_manipulation_resources.sh catkin_ws melodic"
    exit
fi

if [ -z $2 ]
  then
    echo "No ROS distribution specified, please re-run the script"
    echo "Your command should look like ./setup_armada_pc.sh <workspace_name> <ros_distro>"
    echo "For example; ./setup_armada_manipulation_resources.sh catkin_ws melodic"
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

printmsg "installing MoveIt"
sudo apt install ros-$ROS_DISTRO-moveit -y

printmsg "Installing GPD as a library"
cd ~/$WORKSPACE
git clone https://github.com/atenpas/gpd
sed -i -e 's/PCL 1.9 REQUIRED/PCL REQUIRED/g' ~/$WORKSPACE/gpd/CMakeLists.txt
cd gpd
mkdir build && cd build
cmake ..
make -j
sudo make install

printmsg "Cloning and installing the gpd_ros package"
cd ~/$WORKSPACE/src
git clone -b master https://github.com/atenpas/gpd_ros
sed -i -e 's/PCL 1.9 REQUIRED/PCL REQUIRED/g' ~/$WORKSPACE/src/gpd_ros/CMakeLists.txt
catkin build gpd_ros

printmsg "Cloning the uml_robotics/universal_robot package"
cd ~/$WORKSPACE/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git fmauch_universal_robot
sudo apt update -qq
rosdep update
cd ~/$WORKSPACE
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin build Universal_Robots_ROS_Driver
catkin build fmauch_universal_robot

printmsg "Cloning the librealsense package"
cd ~/$WORKSPACE/src
git clone -b master https://github.com/IntelRealSense/librealsense.git

printmsg "Installing additional libraries for the realsense cameras"
sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev -y

printmsg "Adding keyservers to the list of source repositories"
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://librealsense.intel.com/Debian/apt-repo focal main" -u

printmsg "Installing additional librealsense libraries"
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y

printmsg "Updating to include new source repositories"
sudo apt update && sudo apt-get upgrade -y
catkin build librealsense2

printmsg "Running librealsense script to edit your machine's udev rules to allow communication with the realsense device"
cd ~/$WORKSPACE/src/librealsense
./scripts/setup_udev_rules.sh
printmsg "Patching the realsense drivers" 
printwrn "WARNING: This script will require you to enter your password at some point but it also takes a long time to load - keep checking back for progress"
./scripts/patch-realsense-ubuntu-lts.sh

printmsg "Cloning realsense package (not realsense-ros package)"
cd ~/$WORKSPACE/src/
git clone -b development https://github.com/doronhi/realsense.git
cd realsense
printmsg "Cloning the ddynamic_reconfigure into the realsense package, this is necessary for the package to function"
git clone -b kinetic-devel https://github.com/pal-robotics/ddynamic_reconfigure.git

printmsg "Installing the rgbd_launch/rs_camera package specifically for realsense camera functionality"
sudo apt install ros-$ROS_DISTRO-rgbd-launch -y

printmsg "Cloning gazebo model resources into the hidden .gazebo folder in your home directory for simualtion usage"
mkdir -p ~/.gazebo
cd ~/.gazebo
git clone -b master https://github.com/osrf/gazebo_models.git models

printmsg "Cloning Gazebo mimic joint functionality plugin package"
cd ~/$WORKSPACE/src
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git

printmsg "Installing FlexBe"
sudo apt install ros-$ROS_DISTRO-flexbe-behavior-engine
cd ~/$WORKSPACE/src
git clone https://github.com/FlexBE/flexbe_app.git
git clone https://github.com/FlexBE/generic_flexbe_states.git

sudo apt install ros-melodic-gripper-action-controller

printmsg "Do a final build of all the packages"
cd ~/$WORKSPACE
catkin build
cd -


# old stuff for using ros_kortex that might become relevant again
#printmsg "Installing python3 and conan resources and set conan profile variables"
#sudo apt install python3 python3-pip -y
#sudo python3 -m pip install conan
#conan config set general.revisions_enabled=1
#conan profile new default --detect > /dev/null
#conan profile update settings.compiler.libcxx=libstdc++11 default
#printmsg "Cloning the ros_kortex_vision package and installing additional gstreamer libraries"
#sudo apt install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base -y
#cd ~/$WORKSPACE/src/
#git clone -b master https://github.com/Kinovarobotics/ros_kortex_vision.git
#build
