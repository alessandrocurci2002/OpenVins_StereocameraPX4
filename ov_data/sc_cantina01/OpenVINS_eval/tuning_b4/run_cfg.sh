#!/bin/bash
# Una run di OpenVINS con la cartella di config tuning_b4/runs/NOME (creata da make_cfg.py).
# Da lanciare DENTRO il container ROS2.  Uso: run_cfg.sh NOME DOMAIN_ID BAG_relativo_a_rosbags [RATE]
NAME=$1; DOMAIN=$2; BAGREL=$3; RATE=${4:-1.0}
EVAL=/root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval
RUN=$EVAL/tuning_b4/runs/$NAME
source /opt/ros/humble/setup.bash; source /root/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1; unset DISPLAY
rm -f "$RUN"/ov_estimate*.txt "$RUN"/run.done
ros2 run ov_msckf run_subscribe_msckf --ros-args -p config_path:="$RUN"/estimator_config.yaml -p save_total_state:=true \
  -p filepath_est:="$RUN"/ov_estimate.txt -p filepath_std:="$RUN"/ov_estimate_std.txt -p filepath_gt:="$RUN"/ov_groundtruth.txt > "$RUN"/openvins.log 2>&1 &
sleep 4
ros2 bag play "$EVAL/rosbags/$BAGREL" --disable-keyboard-controls --rate "$RATE" \
  --topics /oak/imu/data /oak/left/image_rect /oak/right/image_rect > "$RUN"/bag.log 2>&1
sleep 3
pkill -INT -f "[f]ilepath_est:=$RUN/ov_estimate.txt"
for _ in $(seq 1 20); do pgrep -f "[f]ilepath_est:=$RUN/ov_estimate.txt" > /dev/null || break; sleep 1; done
pkill -KILL -f "[f]ilepath_est:=$RUN/ov_estimate.txt" 2>/dev/null
echo "[$NAME] righe: $(grep -vc '^#' "$RUN"/ov_estimate.txt)" > "$RUN"/run.done
chmod -R a+rwX "$RUN"
