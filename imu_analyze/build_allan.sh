#!/bin/bash
cd /home/moi/1_ws/

source /opt/ros/noteic/setup.bash
source ~/1_ws/devel/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="code_utils"
catkin_make -DCATKIN_WHITELIST_PACKAGES="imu_utils"

#注释
#安装  sudo apt-get install libdw-dev
