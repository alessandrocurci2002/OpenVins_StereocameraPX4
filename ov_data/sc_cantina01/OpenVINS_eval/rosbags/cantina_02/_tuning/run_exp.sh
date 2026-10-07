#!/bin/bash
# usage: run_exp.sh <config_name> <ros_domain_id>   (eseguito dentro il container)
source /opt/ros/*/setup.bash
source /root/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=$2
D=/root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval/rosbags/cantina_02/_tuning/$1
BAG=/root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval/rosbags/cantina_02/tagslam_cantina02
ros2 run ov_msckf run_subscribe_msckf --ros-args -p config_path:=$D/estimator_config.yaml \
  -p save_total_state:=true -p filepath_est:=$D/est.txt -p filepath_std:=$D/std.txt -p filepath_gt:=$D/gt.txt > $D/log.txt 2>&1 &
sleep 5
ros2 bag play $BAG > $D/play.log 2>&1
sleep 3
pkill -INT -f "config_path:=$D/estimator_config.yaml"
sleep 2
pkill -KILL -f "config_path:=$D/estimator_config.yaml"
echo done $1
