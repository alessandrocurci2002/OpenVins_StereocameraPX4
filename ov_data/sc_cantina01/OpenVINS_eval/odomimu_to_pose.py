#!/usr/bin/env python3
"""
Converte /odomimu (nav_msgs/Odometry pubblicato dall'OpenVINS che gira in tempo reale sul
RPi5, quello che il bridge manda a PX4) nel formato di ov_eval, nel frame cam0 come la GT:
    # timestamp(s) tx ty tz qx qy qz qw

/odomimu e' la posa dell'IMU nel mondo di OpenVINS (q = Hamilton IMU -> mondo, p_IinG).
cam0: R_W_C = R_W_I R_I_C, p_W_C = p_W_I + R_W_I p_CinI, con T_cam_imu di cam0 della config
indicata (la config online ha calib_cam_extrinsics: true, ma gli estrinseci stimati online
non sono nel messaggio: si usano quelli di calibrazione, differenza attesa < 1 mm / 0.1 deg).

Uso (nel container ROS2):
    python3 odomimu_to_pose.py rosbags/batch3_6dof/rec_20261007_153538 --out algorithms/openvins_rpi_online/batch3_6dof/run_1.txt
"""
import argparse
import sqlite3
from pathlib import Path

import numpy as np
import yaml
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation as R

import px4_bag

HERE = Path(__file__).resolve().parent


def convert(bag, out, imucam):
    T_C_I = np.array(yaml.safe_load(open(imucam).read().replace("%YAML:1.0", ""))["cam0"]["T_cam_imu"])
    R_I_C = T_C_I[:3, :3].T
    p_CinI = -R_I_C @ T_C_I[:3, 3]
    ts = get_typestore(Stores.ROS2_HUMBLE)
    db = sqlite3.connect(str(px4_bag.db3_of(bag)))
    tid, typ = db.execute("select id, type from topics where name = '/odomimu'").fetchone()
    rows, frames = [], set()
    for (raw,) in db.execute("select data from messages where topic_id = ? order by timestamp", (tid,)):
        m = ts.deserialize_cdr(raw, typ)
        frames.add((m.header.frame_id, m.child_frame_id))
        p, q = m.pose.pose.position, m.pose.pose.orientation
        rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    a = np.array(sorted(rows))
    a = a[np.concatenate([[True], np.diff(a[:, 0]) > 0])]
    R_W_I = R.from_quat(a[:, 4:8])
    p_W_C = a[:, 1:4] + R_W_I.apply(p_CinI)
    q_W_C = (R_W_I * R.from_matrix(R_I_C)).as_quat()
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f:
        f.write("# timestamp(s) tx ty tz qx qy qz qw\n")
        for t, p, q in zip(a[:, 0], p_W_C, q_W_C):
            f.write("%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (t, *p, *q))
    print(f"[OK] {len(a)} pose /odomimu -> {out} ({a[0, 0]:.3f} -> {a[-1, 0]:.3f} s, frame {frames})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", required=True)
    ap.add_argument("--imucam", default=str(HERE / "rosbags/cantina_03/Openvins_config_v9/kalibr_imucam_chain.yaml"))
    a = ap.parse_args()
    convert(a.bag, a.out, a.imucam)
