#!/bin/bash
# Based on https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_melodic.sh


echo ""
echo "[Note] Target OS version  >>> Ubuntu 18.04.x (Bionic Beaver) or Linux Mint 19.x"
echo "[Note] Target ROS version >>> ROS Melodic Morenia"
echo "[Note] Catkin workspace   >>> $HOME/low_cost_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target OS, ROS version and name of catkin workspace]"
name_os_version=${name_os_version:="bionic"}
name_ros_version=${name_ros_version:="melodic"}
name_ros_workspace=${name_ros_workspace:="low_cost_ws"}

echo "[Update the package lists and upgrade them]"
sudo apt update -y
sudo apt upgrade -y

echo "[Install PyRobot library]"
sudo apt-get install curl
curl 'https://raw.githubusercontent.com/TempleRAIL/pyrobot/master/robots/LoCoBot/install/locobot_install_all.sh' > locobot_install_all.sh
chmod +x locobot_install_all.sh
./locobot_install_all.sh -t sim_only -p 2 -l interbotix
rm locobot_install_all.sh

echo "[Install the ROS packages]"
sudo apt install -y \
    ros-$name_ros_version-gazebo-ros \
    ros-$name_ros_version-eigen-conversions \
    ros-$name_ros_version-object-recognition-msgs \
    ros-$name_ros_version-roslint \
    ros-$name_ros_version-vision-msgs \
    ros-$name_ros_version-ros-numpy \
    ros-$name_ros_version-dynamixel-workbench-msgs

pip install scikit-image

echo "[Download Gazebo grasp fix plugin]"
if [ ! -d "$HOME/$name_ros_workspace/src" ] 
then
    mkdir -p $HOME/$name_ros_workspace/src
fi
cd $HOME/$name_ros_workspace/src
if [ ! -d "general-message-pkgs" ]
then
    git clone https://github.com/JenniferBuehler/general-message-pkgs.git
fi
if [ ! -d "gazebo-pkgs" ]
then
    git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
fi

echo "[Complete!!!]"
exit 0
