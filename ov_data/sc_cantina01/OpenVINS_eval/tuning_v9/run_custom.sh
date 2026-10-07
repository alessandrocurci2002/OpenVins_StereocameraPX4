#!/bin/bash
# Run di OpenVINS con una cartella di configurazione gia' pronta (tuning_v9/runs/NOME).
# Da lanciare DENTRO il container.  Uso: run_custom.sh NOME DOMAIN_ID BAG_relativo_a_rosbags [START_OFFSET_s]
NAME=$1; DOMAIN=$2; BAGREL=$3; OFFSET=${4:-0}
EVAL=/root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval
RUN=$EVAL/tuning_v9/runs/$NAME
source /opt/ros/humble/setup.bash; source /root/colcon_ws/install/setup.bash
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1; unset DISPLAY
ros2 run ov_msckf run_subscribe_msckf --ros-args -p config_path:="$RUN"/estimator_config.yaml -p save_total_state:=true \
  -p filepath_est:="$RUN"/ov_estimate.txt -p filepath_std:="$RUN"/ov_estimate_std.txt -p filepath_gt:="$RUN"/ov_groundtruth.txt > "$RUN"/openvins.log 2>&1 &
sleep 4
ros2 bag play "$EVAL/rosbags/$BAGREL" --disable-keyboard-controls --start-offset "$OFFSET" > "$RUN"/bag.log 2>&1
sleep 3
pkill -INT -f "filepath_est:=$RUN/ov_estimate.txt"
for _ in $(seq 1 20); do pgrep -f "filepath_est:=$RUN/ov_estimate.txt" > /dev/null || break; sleep 1; done
pkill -KILL -f "filepath_est:=$RUN/ov_estimate.txt" 2>/dev/null
echo "[$NAME] righe: $(grep -vc '^#' "$RUN"/ov_estimate.txt)" > "$RUN"/run.done
