#!/bin/bash
# Una run di OpenVINS su cantina_03 con i rumori IMU dati, partendo da Openvins_config_v9.
# Da lanciare DENTRO il container. Ogni run usa un ROS_DOMAIN_ID diverso, cosi' piu'
# run possono girare in parallelo senza vedere i topic delle altre.
#
# Uso: run_one.sh NOME ACC_ND ACC_RW GYR_ND GYR_RW DOMAIN_ID [BAG relativo a rosbags/, default cantina_03]
NAME=$1; AN=$2; AR=$3; GN=$4; GR=$5; DOMAIN=$6

EVAL=/root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval
BASE=$EVAL/rosbags/cantina_03/Openvins_config_v9
BAG=$EVAL/rosbags/${7:-cantina_03/tagslam_cantina03}
RUN=$EVAL/tuning_v9/runs/$NAME

source /opt/ros/humble/setup.bash
source /root/colcon_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=1
unset DISPLAY

rm -rf "$RUN"; mkdir -p "$RUN"
cp "$BASE"/estimator_config.yaml "$BASE"/kalibr_imu_chain.yaml "$BASE"/kalibr_imucam_chain.yaml "$RUN"/
sed -i -E \
  -e "s/^(  accelerometer_noise_density:).*/\1 $AN/" \
  -e "s/^(  accelerometer_random_walk:).*/\1 $AR/" \
  -e "s/^(  gyroscope_noise_density:).*/\1 $GN/" \
  -e "s/^(  gyroscope_random_walk:).*/\1 $GR/" \
  "$RUN"/kalibr_imu_chain.yaml
echo "$AN $AR $GN $GR" > "$RUN"/noise.txt

ros2 run ov_msckf run_subscribe_msckf --ros-args \
  -p config_path:="$RUN"/estimator_config.yaml \
  -p save_total_state:=true \
  -p filepath_est:="$RUN"/ov_estimate.txt \
  -p filepath_std:="$RUN"/ov_estimate_std.txt \
  -p filepath_gt:="$RUN"/ov_groundtruth.txt \
  > "$RUN"/openvins.log 2>&1 &
OV_PID=$!

sleep 4
ros2 bag play "$BAG" --disable-keyboard-controls > "$RUN"/bag.log 2>&1
sleep 3

# SIGINT al processo vero (ros2 run fa da wrapper), poi attesa della chiusura dei file
pkill -INT -f "filepath_est:=$RUN/ov_estimate.txt" 2>/dev/null
for _ in $(seq 1 30); do
  pgrep -f "filepath_est:=$RUN/ov_estimate.txt" > /dev/null || break
  sleep 1
done
pkill -KILL -f "filepath_est:=$RUN/ov_estimate.txt" 2>/dev/null
wait $OV_PID 2>/dev/null

echo "[$NAME] righe stimate: $(grep -vc '^#' "$RUN"/ov_estimate.txt 2>/dev/null)"
