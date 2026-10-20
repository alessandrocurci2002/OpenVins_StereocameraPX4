#!/bin/bash
# Margini attorno a P2: soglia di divergenza su N_a, ruolo del giroscopio, feature minime; P2 con feature del RPi sul batch4.
# DENTRO il container, da tuning_b4.
F100="num_pts=100 max_msckf_in_update=20 max_slam=25"
F200="num_pts=200 max_msckf_in_update=40 max_slam=50"
RW="accelerometer_random_walk=0.015 gyroscope_random_walk=0.0002"
B=prova_drone_20261009_172706_recovered
: > lists/noise_extra.txt
python3 make_cfg.py fl_Na018_f200 init_window_time=2.0 accelerometer_noise_density=0.18 gyroscope_noise_density=0.003 $RW $F200
python3 make_cfg.py fl_Na025_f200 init_window_time=2.0 accelerometer_noise_density=0.25 gyroscope_noise_density=0.003 $RW $F200
python3 make_cfg.py fl_P2g15_f200 init_window_time=2.0 accelerometer_noise_density=0.35 gyroscope_noise_density=0.0015 $RW $F200
python3 make_cfg.py fl_P2_f100 init_window_time=2.0 accelerometer_noise_density=0.35 gyroscope_noise_density=0.003 $RW $F100
for n in fl_Na018_f200 fl_Na025_f200 fl_P2g15_f200 fl_P2_f100; do echo "$n $B 1.0" >> lists/noise_extra.txt; done
python3 make_cfg.py b4n_P2_f200 accelerometer_noise_density=0.35 gyroscope_noise_density=0.003 $RW $F200
python3 make_cfg.py e6n_P2_f200 accelerometer_noise_density=0.35 gyroscope_noise_density=0.003 $RW $F200
echo batch_4_2loops_6dof_motor_on > runs/e6n_P2_f200/dataset.txt
echo "b4n_P2_f200 batch_4_2loops_motor_on/rec_20261009_091521 1.0" >> lists/noise_extra.txt
echo "e6n_P2_f200 batch_4_2loops_6dof_motor_on_sshdownload 1.0" >> lists/noise_extra.txt
chmod -R a+rwX runs lists
