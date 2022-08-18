#!/bin/bash

WORKSPACE=$1

if [ -z $1 ]
  then
    echo "No desired workspace name supplied, please re-run the script"
    echo "Your command should look like ./setup_armada_pc.sh <workspace_name>>"
    echo "For example; ./setup_armada_workspace.sh catkin_ws"
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
cd ~/$WORKSPACE/src
git clone https://github.com/atenpas/gpd
sed -i -e 's|PCL 1.9 REQUIRED|PCL REQUIRED|g' ~/$WORKSPACE/src/gpd/CMakeLists.txt
sed -i -e 's|weights_file = /home/andreas/projects/gpd/lenet/15channels/params/|weights_file = /home/$USER/$WORKSPACE/gpd/models/lenet/15channels/params/|g' ~/$WORKSPACE/src/gpd/cfg/ros_eigen_params.cfg
cd gpd
mkdir build && cd build
cmake ..
make -j
sudo make install
catkin build

printmsg "Cloning and installing the gpd_ros package"
cd ~/$WORKSPACE/src
git clone -b master https://github.com/atenpas/gpd_ros
sed -i -e 's/PCL 1.9 REQUIRED/PCL REQUIRED/g' ~/$WORKSPACE/src/gpd_ros/CMakeLists.txt
catkin build gpd_ros

printmsg "Cloning UniversalRobots packages"
cd ~/$WORKSPACE/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git fmauch_universal_robot
sudo apt update -qq
rosdep update
cd ~/$WORKSPACE
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin build Universal_Robots_ROS_Driver
catkin build fmauch_universal_robot

printmsg "Installing Intel Realsense packages"
cd ~/$WORKSPACE/src
sudo apt install ros-$ROS_DISTRO-realsense2-camera -y
sudo apt install ros-$ROS_DISTRO-realsense2-description -y

printmsg "Installing FlexBe"
sudo apt install ros-$ROS_DISTRO-flexbe-behavior-engine
cd ~/$WORKSPACE/src
git clone https://github.com/FlexBE/flexbe_app.git
git clone https://github.com/FlexBE/generic_flexbe_states.git

printmsg "Cloning gazebo model resources into the hidden .gazebo folder in your home directory for simualtion usage"
cd ~/.gazebo
git clone -b master https://github.com/osrf/gazebo_models.git models

printmsg "Cloning Gazebo mimic joint functionality plugin package"
cd ~/$WORKSPACE/src
git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git

sudo apt install ros-melodic-gripper-action-controller

printmsg "Do a final build of all the packages"
cd ~/$WORKSPACE
catkin build
cd -
