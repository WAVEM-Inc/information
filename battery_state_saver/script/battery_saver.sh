#!bin/bash
# ros2 version
# /opt/ros/humble/setup.bash
#
source /home/nuc-bt/kec_ws/src/total.bash
nohup ros2 run battery_state_saver battery_state_saver &
