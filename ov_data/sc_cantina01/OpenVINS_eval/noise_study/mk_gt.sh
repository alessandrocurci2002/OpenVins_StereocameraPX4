#!/bin/bash
# Candidati di rumore IMU (noise_study) sui bag con ground truth TagSLAM. DENTRO il container, da tuning_b4.
# batch4 (motori accesi, a mano): feature 300/80/75 come config_b4; cantina_03 (motori spenti): feature della v9.
F300="num_pts=300 max_msckf_in_update=80 max_slam=75"
declare -A N
N[P0]="accelerometer_noise_density=0.09 accelerometer_random_walk=0.002 gyroscope_noise_density=0.0015 gyroscope_random_walk=0.00002"
N[P2]="accelerometer_noise_density=0.35 accelerometer_random_walk=0.015 gyroscope_noise_density=0.003 gyroscope_random_walk=0.0002"
N[P3]="accelerometer_noise_density=0.6 accelerometer_random_walk=0.045 gyroscope_noise_density=0.0045 gyroscope_random_walk=0.0007"
N[P2lo]="accelerometer_noise_density=0.35 accelerometer_random_walk=0.002 gyroscope_noise_density=0.003 gyroscope_random_walk=0.00002"
N[P2hi]="accelerometer_noise_density=0.35 accelerometer_random_walk=0.045 gyroscope_noise_density=0.003 gyroscope_random_walk=0.0007"
: > lists/noise_gt.txt
for p in P0 P2 P3 P2lo P2hi; do
  python3 make_cfg.py b4n_$p ${N[$p]} $F300; echo "b4n_$p batch_4_2loops_motor_on/rec_20261009_091521 1.0" >> lists/noise_gt.txt
  python3 make_cfg.py e6n_$p ${N[$p]} $F300; echo batch_4_2loops_6dof_motor_on > runs/e6n_$p/dataset.txt
  echo "e6n_$p batch_4_2loops_6dof_motor_on_sshdownload 1.0" >> lists/noise_gt.txt
done
for p in P2 P3; do
  python3 make_cfg.py c3n_$p ${N[$p]}; echo dataset4 > runs/c3n_$p/dataset.txt
  echo "c3n_$p cantina_03/tagslam_cantina03 1.0" >> lists/noise_gt.txt
done
chmod -R a+rwX runs lists
