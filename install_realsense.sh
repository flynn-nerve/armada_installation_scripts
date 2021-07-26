#!/bin/bash

WORKSPACE=$1

if [ -z $1 ]
  then
    echo "No workspace supplied, please re-run the script"
    echo "Your command should look like ./install_realsense.sh <your_workspace>"
    echo "For example; ./install_realsense.sh catkin_ws"
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

printmsg "Cloning the librealsense package"
cd ~/$WORKSPACE/src
git clone -b master https://github.com/IntelRealSense/librealsense.git

printmsg "Installing additional libraries for the realsense cameras"
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev -y

printmsg "Adding keyservers to the list of source repositories"
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

printmsg "Installing additional librealsense libraries"
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y

printmsg "Updating to include new source repositories"
sudo apt-get update && sudo apt-get upgrade -y
build

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
sudo apt-get install ros-$ROS_DISTRO-rgbd-launch -y
build
