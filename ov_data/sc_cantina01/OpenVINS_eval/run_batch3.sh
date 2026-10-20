#!/bin/bash
# OpenVINS (config_v9) sui bag del batch03. Da lanciare DENTRO il container ROS2 (provola1).
# Per ogni test copia la config_v9 (rosbags/cantina_03/Openvins_config_v9) in
# rosbags/batch3_<test>/Openvins_config_v9/, riproduce il bag a velocita' reale e salva
# il total state (ov_estimate.txt) e le deviazioni standard (ov_estimate_std.txt).
# Le run sono in sequenza: in parallelo la contesa di CPU puo' far scartare frame.
#
# Uso: run_batch3.sh [2loops hover 6dof]
EVAL=/root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval
CFG=$EVAL/rosbags/cantina_03/Openvins_config_v9
source /opt/ros/humble/setup.bash
source /root/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1
unset DISPLAY

for TEST in ${@:-2loops hover 6dof}; do
  BAG=$(ls -d $EVAL/rosbags/batch3_$TEST/rec_*/ | head -1)
  RUN=$EVAL/rosbags/batch3_$TEST/Openvins_config_v9
  rm -rf "$RUN"; mkdir -p "$RUN"
  cp "$CFG"/estimator_config.yaml "$CFG"/kalibr_imu_chain.yaml "$CFG"/kalibr_imucam_chain.yaml "$RUN"/
  echo "[$(date +%T)] $TEST: $BAG"
  ros2 run ov_msckf run_subscribe_msckf --ros-args -p config_path:="$RUN"/estimator_config.yaml \
    -p save_total_state:=true -p filepath_est:="$RUN"/ov_estimate.txt -p filepath_std:="$RUN"/ov_estimate_std.txt \
    -p filepath_gt:="$RUN"/ov_groundtruth.txt > "$RUN"/openvins.log 2>&1 &
  sleep 4
  # solo i topic che OpenVINS usa: i px4_msgs non sono installati nel container
  ros2 bag play "$BAG" --disable-keyboard-controls \
    --topics /oak/imu/data /oak/left/image_rect /oak/right/image_rect > "$RUN"/bag.log 2>&1
  sleep 3
  pkill -INT -f "filepath_est:=$RUN/ov_estimate.txt"
  for _ in $(seq 1 30); do pgrep -f "filepath_est:=$RUN/ov_estimate.txt" > /dev/null || break; sleep 1; done
  pkill -KILL -f "filepath_est:=$RUN/ov_estimate.txt" 2>/dev/null
  echo "[$(date +%T)] $TEST: righe $(grep -vc '^#' "$RUN"/ov_estimate.txt)" | tee "$RUN"/run.done
done
