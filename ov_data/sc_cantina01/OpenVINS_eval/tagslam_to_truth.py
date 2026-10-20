#!/usr/bin/env python3
"""
GT di ov_eval da TagSLAM (/tagslam/odom/body_rig di out.bag = posa di cam0 nel mondo del tag 0),
livellata rispetto alla gravita':
    # timestamp(s) tx ty tz qx qy qz qw

Il mondo di TagSLAM e' il frame del tag 0, appeso a muro_1: x a destra lungo il muro, y in alto,
z verso la stanza. Con y in alto il grafico "x-y" di plot_trajectories sarebbe una vista frontale
e la "z" la profondita'. Qui il mondo viene ruotato (rotazione rigida: ATE/RPE con allineamento se3
non cambiano) in un frame G con
    z_G = verticale verso l'alto, x_G = x del tag 0 proiettato in orizzontale, y_G = z_G x x_G
(y_G orizzontale, entra nel muro). La verticale nel frame del tag 0 si stima da uno stimatore
riferito alla gravita' (z del suo mondo = alto) confrontando le orientazioni di cam0:
    u_tag = R_tag_C(t) R_up_C(t)^T e_z   (media su tutti gli istanti comuni)
Uso (nel container ROS2):
    python3 tagslam_to_truth.py rosbags/batch3_hover/tagslam/out.bag --up-from algorithms/config_v9/batch3_hover/run_1.txt \
        --out truths/batch3_hover.txt [--t-start T --t-end T] [--up-vector ux uy uz]
"""
import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation as R


def load_tagslam(bag):
    with AnyReader([Path(bag)]) as rd:
        con = [c for c in rd.connections if c.topic == "/tagslam/odom/body_rig"]
        rows = []
        for c, _, raw in rd.messages(connections=con):
            m = rd.deserialize(raw, c.msgtype)
            p, q = m.pose.pose.position, m.pose.pose.orientation
            rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    a = np.array(sorted(rows))
    return a[np.concatenate([[True], np.diff(a[:, 0]) > 0])]


def up_in_tag_frame(gt, est, max_dt=0.01):
    """Verticale (versore) nel mondo del tag 0 da uno stimatore con mondo z-up; anche la dispersione in gradi."""
    te = est[:, 0]
    idx = np.clip(np.searchsorted(te, gt[:, 0]), 1, len(te) - 1)
    idx = np.where(np.abs(te[idx - 1] - gt[:, 0]) < np.abs(te[idx] - gt[:, 0]), idx - 1, idx)
    ok = np.abs(te[idx] - gt[:, 0]) < max_dt
    R_tag_C = R.from_quat(gt[ok, 4:8])
    R_up_C = R.from_quat(est[idx[ok], 4:8])
    u = (R_tag_C * R_up_C.inv()).apply([0.0, 0.0, 1.0])
    m = u.mean(axis=0)
    m /= np.linalg.norm(m)
    spread = np.degrees(np.arccos(np.clip(u @ m, -1, 1)))
    return m, np.median(spread), int(ok.sum())


def leveling_rotation(u):
    x = np.array([1.0, 0.0, 0.0]) - u[0] * u
    x /= np.linalg.norm(x)
    y = np.cross(u, x)
    return np.vstack([x, y, u])   # righe = assi di G nel mondo del tag: v_G = R_G_tag v_tag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", required=True)
    ap.add_argument("--up-from", help="traiettoria ov_eval (cam0) di uno stimatore con mondo z-up")
    ap.add_argument("--up-vector", nargs=3, type=float, help="verticale nel mondo del tag 0 gia' nota")
    ap.add_argument("--t-start", type=float, default=-np.inf)
    ap.add_argument("--t-end", type=float, default=np.inf)
    a = ap.parse_args()
    gt = load_tagslam(a.bag)
    if a.up_vector:
        u = np.array(a.up_vector) / np.linalg.norm(a.up_vector)
        print(f"verticale data: {np.round(u, 5)}")
    else:
        u, spread, n = up_in_tag_frame(gt, np.loadtxt(a.up_from))
        print(f"verticale nel mondo del tag 0 da {a.up_from}: {np.round(u, 5)} (dispersione mediana {spread:.2f} deg su {n} istanti)")
    print(f"  inclinazione dell'asse y del tag 0 rispetto alla verticale: {np.degrees(np.arccos(u[1])):.2f} deg")
    R_G_tag = leveling_rotation(u)
    sel = (gt[:, 0] >= a.t_start) & (gt[:, 0] <= a.t_end)
    g = gt[sel]
    p = g[:, 1:4] @ R_G_tag.T
    q = (R.from_matrix(R_G_tag) * R.from_quat(g[:, 4:8])).as_quat()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "w") as f:
        f.write("# timestamp(s) tx ty tz qx qy qz qw\n")
        for t, pp, qq in zip(g[:, 0], p, q):
            f.write("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n" % (t, *pp, *qq))
    path = np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))
    print(f"[OK] {len(g)} pose -> {a.out} ({g[0, 0]:.3f} -> {g[-1, 0]:.3f} s, {g[-1, 0] - g[0, 0]:.1f} s, percorso {path:.2f} m)")


if __name__ == "__main__":
    main()
