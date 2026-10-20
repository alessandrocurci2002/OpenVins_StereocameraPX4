#!/usr/bin/env python3
"""
Stima dai sensori inerziali la relazione tra il Pixhawk (EKF2) e l'IMU dell'OAK-D Pro, cioe' il
parametro imu_to_body_rotation_deg di px4_vio_bridge/config/vio_bridge.yaml.

  B = body FRD del Pixhawk (sensor_combined, gia' ruotato con SENS_BOARD_ROT e calibrato da PX4)
  I = frame IMU di OpenVINS, quello delle pose che il bridge riceve su /odomimu. Con il modello
      "kalibr" coincide con il frame dell'accelerometro: il giroscopio grezzo dell'OAK va prima
      portato in I come fa OpenVINS (Propagator.cpp):
          a_I = R_ACCtoIMU * Ta^-1 * (a_m - b_a)
          w_I = R_IMUtoGYRO^T * Tw^-1 * (w_m - b_g - Tg * a_I)
      con Tw, Ta, R_IMUtoGYRO, R_IMUtoACC, Tg di kalibr_imu_chain.yaml (config_v9).
  R_B_I: v_B = R_B_I v_I, la stessa matrice che il bridge costruisce con
         R = Rz(yaw) Ry(pitch) Rx(roll) da imu_to_body_rotation_deg = [roll, pitch, yaw].

Passi:
  1. offset temporale t_px4 - t_oak: massimo della correlazione di |w| (non dipende dal frame);
  2. bias dei giroscopi: media nelle fasi da fermo (|w| piccolo per almeno 1 s su entrambi);
     bias dell'accelerometro OAK: quello stimato da OpenVINS (ov_estimate.txt, colonne 14-16) nelle
     stesse fasi; quello del Pixhawk non e' noto ed entra nell'incertezza del vincolo di gravita';
  3. R_B_I con il problema di Wahba (scipy align_vectors) su
       - vettori w sincronizzati (rotazione), peso 1 / sigma_w^2
       - forza specifica media di ogni fase da fermo (gravita'), peso 1 / sigma_g^2
     Un insieme di vettori tutti paralleli a un asse u lascia libera la rotazione attorno a u. La
     gravita' e' verticale, quindi fissa solo l'inclinazione (2 gradi di liberta'); la rotazione
     attorno alla verticale si ricava solo da rotazioni attorno ad assi orizzontali (roll/pitch del
     corpo), che nell'hover sono piccole. Si riportano la stima con soli giroscopi, quella combinata,
     la loro incertezza (bootstrap su blocchi di 2 s) e gli autovalori dell'informazione per asse.

Uso (nel container ROS2):
    python3 calib_px4_oak.py rosbags/batch3_hover/rec_20261007_153315 [altri bag] [--out px4_oak_calib.yaml]
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
BRIDGE_RPY_DEG = [82.0, -0.9, 90.0]      # valore attuale in vio_bridge.yaml (branch dev/companion_computer_RPi5)
CHAIN = HERE / "rosbags/cantina_03/Openvins_config_v9/kalibr_imu_chain.yaml"
SIGMA_G = 0.05                           # m/s^2: bias residuo (non stimato) dell'accelerometro del Pixhawk


def bridge_matrix(rpy_deg):
    """Esattamente frame_transforms.rotation_matrix_from_euler_deg del bridge."""
    r, p, y = np.radians(rpy_deg)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def bridge_rpy(Rm):
    """Inversa di bridge_matrix: [roll, pitch, yaw] in gradi."""
    return R.from_matrix(Rm).as_euler("ZYX", degrees=True)[::-1]


def load_chain(path):
    c = yaml.safe_load(open(path).read().replace("%YAML:1.0", ""))["imu0"]
    g = lambda k: np.array(c.get(k, np.eye(3)), dtype=float)
    return dict(Tw=g("Tw"), Ta=g("Ta"), R_IMUtoGYRO=g("R_IMUtoGYRO"), R_IMUtoACC=g("R_IMUtoACC"),
                Tg=np.array(c.get("Tg", np.zeros((3, 3))), dtype=float))


def load_oak(bag_dir):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    db = sqlite3.connect(str(px4_bag.db3_of(bag_dir)))
    tid, typ = db.execute("select id, type from topics where name = '/oak/imu/data'").fetchone()
    t, w, a = [], [], []
    for (raw,) in db.execute("select data from messages where topic_id = ? order by timestamp", (tid,)):
        m = ts.deserialize_cdr(raw, typ)
        t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        w.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        a.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
    return np.array(t), np.array(w), np.array(a)


def load_px4(bag_dir):
    msgs = px4_bag.read_topic(bag_dir, "/fmu/out/sensor_combined")
    t = np.array([m.timestamp for _, m in msgs]) * 1e-6
    ta = t + np.array([m.accelerometer_timestamp_relative for _, m in msgs]) * 1e-6
    return t, np.array([m.gyro_rad for _, m in msgs]), ta, np.array([m.accelerometer_m_s2 for _, m in msgs])


def static_mask(t, w, thr=0.03, min_len=1.0):
    """Campioni in fasi con |w| < thr rad/s per almeno min_len secondi."""
    quiet = np.linalg.norm(w, axis=1) < thr
    mask = np.zeros_like(quiet)
    i = 0
    while i < len(t):
        if not quiet[i]:
            i += 1
            continue
        j = i
        while j < len(t) and quiet[j]:
            j += 1
        if t[j - 1] - t[i] >= min_len:
            mask[i:j] = True
        i = j
    return mask


def interp3(tq, t, v):
    return np.stack([np.interp(tq, t, v[:, k]) for k in range(3)], axis=1)


def time_offset(to, wo, tp, wp, max_lag=0.5, fs=1000.0):
    t0, t1 = max(to[0], tp[0]) + max_lag, min(to[-1], tp[-1]) - max_lag
    grid = np.arange(t0, t1, 1 / fs)
    no = np.interp(grid, to, np.linalg.norm(wo, axis=1))
    npn = np.linalg.norm(wp, axis=1)
    lags = np.arange(-max_lag, max_lag, 1 / fs)
    cc = np.array([np.corrcoef(no, np.interp(grid + lag, tp, npn))[0, 1] for lag in lags])
    k = int(np.argmax(cc))
    return lags[k], cc[k]


def segments(mask, t):
    """Intervalli (i0, i1) contigui in cui mask e' vero."""
    out, i = [], 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j < len(mask) and mask[j]:
                j += 1
            out.append((i, j))
            i = j
        else:
            i += 1
    return out


def solve(wI, wB, gI, gB, sigma_w, sigma_g=SIGMA_G):
    a = np.vstack([wB, gB]) if len(gB) else wB
    b = np.vstack([wI, gI]) if len(gI) else wI
    wts = np.concatenate([np.full(len(wB), 1 / sigma_w ** 2), np.full(len(gB), 1 / sigma_g ** 2)])
    rot, _ = R.align_vectors(a, b, weights=wts)
    return rot


def estimate(bag_dir, chain_path=CHAIN, ov_estimate=None, min_rate=0.05, n_boot=300, seed=0):
    ch = load_chain(chain_path)
    to, wm, am = load_oak(bag_dir)
    tp, wp, tpa, ap = load_px4(bag_dir)
    lag, cc = time_offset(to, wm, tp, wp)

    # accelerometro OAK nel frame I, con il bias stimato da OpenVINS dove disponibile
    ba = np.zeros(3)
    ov_estimate = ov_estimate or next(Path(bag_dir).parent.glob("Openvins_config_v9/ov_estimate.txt"), None)
    oak_static = static_mask(to, wm)
    if ov_estimate is not None and Path(ov_estimate).exists():
        ov = np.loadtxt(ov_estimate)
        st_ov = np.interp(ov[:, 0], to, oak_static.astype(float)) > 0.5
        ba = ov[st_ov, 14:17].mean(axis=0) if st_ov.any() else ov[-200:, 14:17].mean(axis=0)
    aI = (ch["R_IMUtoACC"].T @ np.linalg.inv(ch["Ta"]) @ (am - ba).T).T

    # bias dei giroscopi dalle fasi da fermo (su entrambi i sensori, sul tempo OAK)
    wp_on_oak = interp3(to + lag, tp, wp)
    static = oak_static & static_mask(to, wp_on_oak)
    bg_oak = wm[static].mean(axis=0)
    bg_px4 = wp_on_oak[static].mean(axis=0)
    Dw = np.linalg.inv(ch["Tw"])
    wI = (ch["R_IMUtoGYRO"].T @ Dw @ (wm - bg_oak - (ch["Tg"] @ aI.T).T).T).T
    wB = wp_on_oak - bg_px4

    # vincoli: rotazione (|w| > min_rate) e gravita' (media di ogni fase da fermo)
    rot_sel = (np.linalg.norm(wI, axis=1) > min_rate) & (to > tp[0] - lag + 0.5) & (to < tp[-1] - lag - 0.5)
    ap_on_oak = interp3(to + lag, tpa, ap)
    stat_segs = segments(static, to)
    gI = np.array([aI[i:j].mean(0) for i, j in stat_segs]) if stat_segs else np.zeros((0, 3))
    gB = np.array([ap_on_oak[i:j].mean(0) for i, j in stat_segs]) if stat_segs else np.zeros((0, 3))

    rot_w, _ = R.align_vectors(wB[rot_sel], wI[rot_sel])
    resid = wB[rot_sel] - rot_w.apply(wI[rot_sel])
    sigma_w = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1)) / 3))
    rot_c = solve(wI[rot_sel], wB[rot_sel], gI, gB, sigma_w)

    # bootstrap su blocchi di 2 s (campioni consecutivi sono correlati)
    rng = np.random.default_rng(seed)
    blk = np.floor((to - to[0]) / 2.0).astype(int)
    blocks = np.unique(blk[rot_sel])
    boot_w, boot_c = [], []
    for _ in range(n_boot):
        pick = rng.choice(blocks, size=len(blocks), replace=True)
        idx = np.concatenate([np.where(rot_sel & (blk == b))[0] for b in pick])
        rw, _ = R.align_vectors(wB[idx], wI[idx])
        boot_w.append(bridge_rpy(rw.as_matrix()))
        gsel = rng.integers(0, len(gB), len(gB)) if len(gB) else []
        boot_c.append(bridge_rpy(solve(wI[idx], wB[idx], gI[gsel], gB[gsel], sigma_w).as_matrix()))
    # errore angolare residuo della gravita' con la stima dai soli giroscopi
    g_err = [np.degrees(np.arccos(np.clip(np.dot(rot_w.apply(gi), gb) / np.linalg.norm(gi) / np.linalg.norm(gb), -1, 1)))
             for gi, gb in zip(gI, gB)]
    # quanto ogni asse e' eccitato: autovalori della matrice di informazione dei soli giroscopi
    M = wI[rot_sel].T @ wI[rot_sel] / rot_sel.sum()
    ev = np.sort(np.linalg.eigvalsh(np.trace(M) * np.eye(3) - M))
    return dict(lag=lag, cc=cc, n_rot=int(rot_sel.sum()), n_static_segments=len(stat_segs),
                static_s=float(sum(to[j - 1] - to[i] for i, j in stat_segs)), sigma_w=sigma_w,
                rpy_gyro=bridge_rpy(rot_w.as_matrix()), std_gyro=np.std(boot_w, axis=0),
                rpy_comb=bridge_rpy(rot_c.as_matrix()), std_comb=np.std(boot_c, axis=0),
                R_B_I=rot_c.as_matrix(), gravity_err_gyro_only_deg=g_err, info_eig=ev,
                bias_oak_gyro=bg_oak, bias_px4_gyro=bg_px4, bias_oak_acc=ba,
                diff_bridge_deg=float(np.degrees((R.from_matrix(bridge_matrix(BRIDGE_RPY_DEG)).inv() * rot_c).magnitude())))


def fmt(v, d=2):
    return "[" + ", ".join(f"{x:.{d}f}" for x in v) + "]"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--chain", default=str(CHAIN), help="kalibr_imu_chain.yaml con le intrinseche dell'IMU OAK")
    ap.add_argument("--min-rate", type=float, default=0.05, help="|w| minima [rad/s] per i campioni di rotazione")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    res = {}
    for b in a.bags:
        r = estimate(b, a.chain, min_rate=a.min_rate)
        res[b] = r
        print(f"{b}\n  offset t_px4 - t_oak = {r['lag'] * 1000:+.1f} ms (corr {r['cc']:.3f})"
              f"\n  campioni di rotazione {r['n_rot']} (|w| > {a.min_rate} rad/s), residuo sigma_w {r['sigma_w'] * 1000:.1f} mrad/s;"
              f" fasi da fermo {r['n_static_segments']} ({r['static_s']:.0f} s)"
              f"\n  eccitazione (autovalori informazione, rad^2/s^2): {fmt(r['info_eig'], 4)}"
              f"\n  solo giroscopi   [roll, pitch, yaw] = {fmt(r['rpy_gyro'])} deg  +- {fmt(r['std_gyro'])} (1 sigma bootstrap)"
              f"\n  giroscopi+gravita' [roll, pitch, yaw] = {fmt(r['rpy_comb'])} deg  +- {fmt(r['std_comb'])}"
              f"\n  gravita' vista con la stima dei soli giroscopi: errore {fmt(r['gravity_err_gyro_only_deg'])} deg per fase da fermo"
              f"\n  differenza dalla rotazione attuale del bridge {BRIDGE_RPY_DEG}: {r['diff_bridge_deg']:.2f} deg")
    if a.out:
        best = max(res.values(), key=lambda r: r["n_rot"])
        name = [k for k, v in res.items() if v is best][0]
        with open(a.out, "w") as f:
            f.write("# Stimato da calib_px4_oak.py (giroscopi PX4 sensor_combined vs OAK /oak/imu/data nel frame IMU di\n")
            f.write("# OpenVINS, piu' la gravita' nelle fasi da fermo)\n")
            f.write(f"# bag: {name}\n")
            f.write(f"time_offset_px4_minus_oak_s: {best['lag']:.4f}\n")
            f.write("R_B_I:   # v_body_FRD = R_B_I v_imu_openvins\n")
            for row in best["R_B_I"]:
                f.write(f"  - [{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}]\n")
            f.write(f"imu_to_body_rotation_deg: {fmt(best['rpy_comb'], 3)}   # [roll, pitch, yaw], convenzione del bridge\n")
            f.write(f"imu_to_body_rotation_deg_std: {fmt(best['std_comb'], 3)}\n")
        print("scritto", a.out)
