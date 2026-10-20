#!/usr/bin/env python3
"""
Converte l'output di EKF2 (/fmu/out/vehicle_odometry, px4_msgs/VehicleOdometry) di un bag
ROS2 nel formato di traiettoria di ov_eval, nel frame cam0 come la GT di TagSLAM
(/tagslam/odom/body_rig) e OpenVINS esportato con state_to_pose.py --frame cam0:
    # timestamp(s) tx ty tz qx qy qz qw

VehicleOdometry (pose_frame = NED): position = p_B in NED, q = Hamilton [w, x, y, z] da body FRD a NED.
  - mondo: NED ruotato di 180 deg attorno a x (N, -E, up: z verso l'alto come OpenVINS);
    ov_eval allinea comunque i mondi (se3), il cambio serve solo a leggere i file;
  - body: cam0 = B * T_B_C, con R_B_C = R_B_I * R_I_C
      R_B_I da px4_oak_calib.yaml (calib_px4_oak.py, giroscopi PX4 vs OAK),
      R_I_C = R_C_I^T da T_cam_imu di cam0 in kalibr_imucam_chain.yaml (config_v9);
    il braccio di leva p_B_C (centro del Pixhawk -> cam0, in FRD) non e' misurato: default 0,
    passarlo con --lever-arm x y z [m] quando e' noto;
  - tempo: timestamp_sample (istante del campione stimato, clock del RPi grazie alla timesync
    uXRCE-DDS) meno l'offset PX4-OAK di px4_oak_calib.yaml.

Uso (nel container ROS2):
    python3 ekf2_to_pose.py rosbags/batch3_6dof/rec_20261007_153538 --out algorithms/ekf2/batch3_6dof/run_1.txt
"""
import argparse
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

import px4_bag

HERE = Path(__file__).resolve().parent
R_UP_NED = np.diag([1.0, -1.0, -1.0])   # NED -> (N, -E, up)


def load_calib(calib_path, imucam_path):
    c = yaml.safe_load(open(calib_path))
    R_B_I = np.array(c["R_B_I"])
    txt = open(imucam_path).read().replace("%YAML:1.0", "")
    T_C_I = np.array(yaml.safe_load(txt)["cam0"]["T_cam_imu"])
    return R_B_I @ T_C_I[:3, :3].T, float(c["time_offset_px4_minus_oak_s"])


def convert(bag, out, calib, imucam, lever_arm):
    R_B_C, dt_px4 = load_calib(calib, imucam)
    p_B_C = np.array(lever_arm, dtype=float)
    msgs = px4_bag.read_topic(bag, "/fmu/out/vehicle_odometry")
    frames = {m.pose_frame for _, m in msgs}
    if frames != {1}:
        raise ValueError(f"pose_frame attesi NED (1), trovati {frames}")
    resets = sorted({m.reset_counter for _, m in msgs})
    t = np.array([m.timestamp_sample for _, m in msgs]) * 1e-6 - dt_px4
    p_ned = np.array([m.position for _, m in msgs], dtype=float)
    q = np.array([m.q for _, m in msgs], dtype=float)             # w x y z
    R_ned_B = R.from_quat(q[:, [1, 2, 3, 0]])
    R_W_B = R.from_matrix(R_UP_NED) * R_ned_B
    p_W_B = p_ned @ R_UP_NED.T
    R_W_C = R_W_B * R.from_matrix(R_B_C)
    p_W_C = p_W_B + R_W_B.apply(p_B_C)
    qc = R_W_C.as_quat()                                          # x y z w (Hamilton, cam0 -> mondo)
    order = np.argsort(t)
    keep = np.concatenate([[True], np.diff(t[order]) > 0])        # niente timestamp duplicati
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f:
        f.write("# timestamp(s) tx ty tz qx qy qz qw\n")
        for i in order[keep]:
            f.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (t[i], *p_W_C[i], *qc[i]))
    print(f"[OK] {keep.sum()} pose EKF2 -> {out} ({t.min():.3f} -> {t.max():.3f} s, reset_counter {resets})")
    print(f"     R_B_C (body FRD -> cam0) come RPY ZYX: {np.round(R.from_matrix(R_B_C).as_euler('ZYX', degrees=True)[::-1], 2)} deg,"
          f" braccio p_B_C = {p_B_C} m, offset PX4-OAK {dt_px4 * 1000:+.1f} ms")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="cartella del bag ROS2 (o il .db3)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--calib", default=str(HERE / "px4_oak_calib.yaml"))
    ap.add_argument("--imucam", default=str(HERE / "rosbags/cantina_03/Openvins_config_v9/kalibr_imucam_chain.yaml"))
    ap.add_argument("--lever-arm", nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=("X", "Y", "Z"),
                    help="posizione di cam0 nel body FRD del Pixhawk [m]")
    a = ap.parse_args()
    convert(a.bag, a.out, a.calib, a.imucam, a.lever_arm)
