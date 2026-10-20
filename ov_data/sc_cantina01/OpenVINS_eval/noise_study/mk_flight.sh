#!/bin/bash
# Candidati di rumore IMU (noise_study): config cartelle per il volo. DENTRO il container, da tuning_b4.
F200="num_pts=200 max_msckf_in_update=40 max_slam=50"
F300="num_pts=300 max_msckf_in_update=80 max_slam=75"
declare -A N
N[P1]="accelerometer_noise_density=0.026 accelerometer_random_walk=0.002 gyroscope_noise_density=0.0036 gyroscope_random_walk=0.0002"
N[P2]="accelerometer_noise_density=0.35 accelerometer_random_walk=0.015 gyroscope_noise_density=0.003 gyroscope_random_walk=0.0002"
N[P3]="accelerometer_noise_density=0.6 accelerometer_random_walk=0.045 gyroscope_noise_density=0.0045 gyroscope_random_walk=0.0007"
N[P2lo]="accelerometer_noise_density=0.35 accelerometer_random_walk=0.002 gyroscope_noise_density=0.003 gyroscope_random_walk=0.00002"
N[P2hi]="accelerometer_noise_density=0.35 accelerometer_random_walk=0.045 gyroscope_noise_density=0.003 gyroscope_random_walk=0.0007"
: > lists/noise_flight.txt
python3 make_cfg.py fl_P1_f300 init_window_time=2.0 ${N[P1]} $F300; echo "fl_P1_f300 prova_drone_20261009_172706_recovered 1.0" >> lists/noise_flight.txt
for p in P2 P3 P2lo P2hi; do
  python3 make_cfg.py fl_${p}_f200 init_window_time=2.0 ${N[$p]} $F200
  python3 make_cfg.py fl_${p}_f300 init_window_time=2.0 ${N[$p]} $F300
  echo "fl_${p}_f200 prova_drone_20261009_172706_recovered 1.0" >> lists/noise_flight.txt
  echo "fl_${p}_f300 prova_drone_20261009_172706_recovered 1.0" >> lists/noise_flight.txt
done
chmod -R a+rwX runs lists
