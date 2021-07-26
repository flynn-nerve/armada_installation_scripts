#!/bin/bash

WORKSPACE=$1

if [ -z $1 ]
  then
    echo "No workspace supplied, please re-run the script"
    echo "Your command should look like ./install_realsense.sh <your_workspace>"
    echo "For example; ./install_realsense.sh catkin_ws"
    exit
fi

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

## update and upgrade
sudo apt-get update && sudo apt-get upgrade -y

## check for and install missing dependencies and build workspace
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/$WORKSPACE
catkin build
source devel/setup.bash

## run realsense camera scripts to add cameras to udev rules and patch software
## your cameras WILL NOT WORK without allowing this step to happen
cd ~/$WORKSPACE/src/librealsense
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts.sh

## clone realsense package (not realsense-ros package)
## clone ddynamic_reconfigure package into realsense package
cd ~/$WORKSPACE/src/
git clone -b development https://github.com/doronhi/realsense.git
cd realsense
git clone -b $ROS_DISTRO-devel https://github.com/pal-robotics/ddynamic_reconfigure.git

## install rgbd_launch/rs_camera package for realsense cameras
cd ~/$WORKSPACE/
sudo apt-get install ros-$ROS_DISTRO-rgbd-launch -y

## check for and install missing dependencies again, build
cd ~/$WORKSPACE
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
catkin build
source devel/setup.bash
