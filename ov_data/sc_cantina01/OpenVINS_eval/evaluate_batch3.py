#!/usr/bin/env python3
"""
Valutazione del batch03 (cantina, 07/10/2026): TagSLAM (GT) contro OpenVINS offline (config_v9),
OpenVINS online sul RPi5 (/odomimu) ed EKF2 del Pixhawk. Da lanciare DENTRO il container ROS2:
    cd /root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval && python3 evaluate_batch3.py

Ingressi (gia' prodotti da run_batch3.sh e dagli script di conversione):
    rosbags/batch3_<test>/rec_*/                 bag ROS2 (hard link ai bag di TagSLAM/.../tagslam_cantina_batch03)
    rosbags/batch3_<test>/Openvins_config_v9/    ov_estimate.txt, ov_estimate_std.txt
    rosbags/batch3_<test>/tagslam/out.bag        /tagslam/odom/body_rig (TagSLAM, ottimizzazione batch)

Il registratore sul RPi5 ha perso TUTTI i topic per 7.3 s (2loops, da 90.7 s) e 18.5 s (6dof, da
84.3 s): senza IMU ne' immagini OpenVINS offline diverge dopo il buco. I dataset di ov_eval sono quindi
finestre continue (tempi dall'inizio del bag):
    batch3_hover         0 - fine     tutti gli algoritmi
    batch3_6dof_seg1     0 - 84.2 s   tutti (prima del buco)
    batch3_2loops_seg0   0 - 52.0 s   tutti (prima dello stallo di 1.2 s di /odomimu a 52 s, dopo il quale l'online diverge)
    batch3_2loops_seg1   0 - 90.6 s   OpenVINS offline, EKF2 (prima del buco)
    batch3_2loops_full   tutto        EKF2 (gira sul Pixhawk: il buco e' solo nella registrazione)
    batch3_6dof_full     tutto        EKF2
Allineamenti (entrambi calcolati da ov_eval):
    se3ori  principale: rotazione dalla media delle orientazioni, traslazione dai baricentri (aggiunto in
            ov_eval/src/alignment). L'se3 classico (Umeyama sulle sole posizioni) e' degenere in hover
            (traiettoria quasi verticale: la rotazione attorno alla verticale e' indeterminata) e nel 6dof
            viene trascinato dalle posizioni di EKF2: ne escono errori di orientazione di 14-53 deg fittizi.
    se3     classico, per confronto con i risultati precedenti (dataset2-5).
Uscite: truths/<ds>.txt, algorithms/<algo>/<ds>/run_1.txt, results/batch3/<ds>/<align>/..., results/batch3/summary.csv
"""
import csv
import json
import os
import re
import subprocess
from pathlib import Path

import numpy as np
import yaml

from ekf2_to_pose import convert as ekf2_convert
from odomimu_to_pose import convert as odomimu_convert
from state_to_pose import convert as state_convert
from tagslam_to_truth import leveling_rotation, load_tagslam, up_in_tag_frame
from scipy.spatial.transform import Rotation as R

EVAL = Path(__file__).resolve().parent
OV_BIN = Path("/root/colcon_ws/install/ov_eval/lib/ov_eval")
TESTS = {"2loops": "rec_20261007_152554", "hover": "rec_20261007_153315", "6dof": "rec_20261007_153538"}
# (test, fine finestra in s dall'inizio del bag, algoritmi)
DATASETS = {
    "batch3_hover": ("hover", None, ["config_v9", "openvins_rpi_online", "ekf2"]),
    "batch3_6dof_seg1": ("6dof", 84.2, ["config_v9", "openvins_rpi_online", "ekf2"]),
    "batch3_2loops_seg0": ("2loops", 52.0, ["config_v9", "openvins_rpi_online", "ekf2"]),
    "batch3_2loops_seg1": ("2loops", 90.6, ["config_v9", "ekf2"]),
    "batch3_2loops_full": ("2loops", None, ["ekf2"]),
    "batch3_6dof_full": ("6dof", None, ["ekf2"]),
}
ALIGNS = ["se3ori", "se3"]
CONTINUOUS_END = {"2loops": 90.6, "hover": None, "6dof": 84.2}   # per la stima della verticale
LABELS = {
    "tagslam_gt": ["TagSLAM (GT)", "#0b0b0b"],
    "config_v9": ["OpenVINS offline config_v9", "#2a78d6"],
    "openvins_rpi_online": ["OpenVINS online RPi5 (/odomimu)", "#eb6834"],
    "ekf2": ["EKF2 PX4 (flow + range + mag)", "#1baf7a"],
}


def bag_start(test):
    meta = yaml.safe_load(open(next((EVAL / f"rosbags/batch3_{test}").glob("rec_*/metadata.yaml"))))
    return meta["rosbag2_bagfile_information"]["starting_time"]["nanoseconds_since_epoch"] * 1e-9


def write_rows(path, rows, header="# timestamp(s) tx ty tz qx qy qz qw"):
    path.parent.mkdir(parents=True, exist_ok=True)
    ncol = rows.shape[1]
    fmt = "%.6f" + " %.6f" * 7 + (" %.10f" * (ncol - 8))
    np.savetxt(path, rows, fmt=fmt, header=header[2:], comments="# ")


def header_of(path):
    with open(path) as f:
        return f.readline().rstrip("\n")


def convert_all():
    """Stime complete (anche dopo i buchi) in rosbags/batch3_<test>/converted/<algo>.txt, frame cam0."""
    for test, rec in TESTS.items():
        d = EVAL / f"rosbags/batch3_{test}"
        out = d / "converted"
        out.mkdir(exist_ok=True)
        ov = d / "Openvins_config_v9"
        state_convert(str(ov / "ov_estimate.txt"), str(out / "config_v9.txt"), str(ov / "ov_estimate_std.txt"), "cam0")
        ekf2_convert(str(d / rec), str(out / "ekf2.txt"), str(EVAL / "px4_oak_calib.yaml"),
                     str(EVAL / "rosbags/cantina_03/Openvins_config_v9/kalibr_imucam_chain.yaml"), [0.0, 0.0, 0.0])
        odomimu_convert(str(d / rec), str(out / "openvins_rpi_online.txt"),
                        str(EVAL / "rosbags/cantina_03/Openvins_config_v9/kalibr_imucam_chain.yaml"))


def vertical():
    """Verticale nel mondo del tag 0: media delle stime da OpenVINS offline ed EKF2 sui tratti continui."""
    us, rows = [], []
    for test in TESTS:
        t0, end = bag_start(test), CONTINUOUS_END[test]
        gt = load_tagslam(EVAL / f"rosbags/batch3_{test}/tagslam/out.bag")
        if end is not None:
            gt = gt[gt[:, 0] <= t0 + end]
        for algo in ("config_v9", "ekf2"):
            u, spread, n = up_in_tag_frame(gt, np.loadtxt(EVAL / f"rosbags/batch3_{test}/converted/{algo}.txt"))
            us.append(u)
            rows.append((test, algo, u, spread, n))
    m = np.mean(us, axis=0)
    m /= np.linalg.norm(m)
    dev = [np.degrees(np.arccos(np.clip(u @ m, -1, 1))) for u in us]
    return m, rows, max(dev)


def build_dataset(ds, R_G_tag):
    test, end, algos = DATASETS[ds]
    t0 = bag_start(test)
    t_end = t0 + end if end is not None else np.inf
    gt = load_tagslam(EVAL / f"rosbags/batch3_{test}/tagslam/out.bag")
    gt = gt[gt[:, 0] <= t_end]
    p = gt[:, 1:4] @ R_G_tag.T
    q = (R.from_matrix(R_G_tag) * R.from_quat(gt[:, 4:8])).as_quat()
    write_rows(EVAL / f"truths/{ds}.txt", np.column_stack([gt[:, 0], p, q]))
    for algo in algos:
        src = EVAL / f"rosbags/batch3_{test}/converted/{algo}.txt"
        a = np.loadtxt(src)
        a = a[(a[:, 0] >= gt[0, 0] - 0.05) & (a[:, 0] <= t_end)]
        write_rows(EVAL / f"algorithms/{algo}/{ds}/run_1.txt", a, header_of(src))
    path = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
    return dict(t0=t0, t_start=gt[0, 0] - t0, t_end=gt[-1, 0] - t0, n_gt=len(gt), path=path, algos=algos)


def nice_segments(path):
    """5 sotto-segmenti RPE: 10-50% del percorso della GT, arrotondati a 2 cifre significative."""
    out = []
    for f in (0.1, 0.2, 0.3, 0.4, 0.5):
        x = path * f
        e = np.floor(np.log10(x)) - 1
        out.append(float(np.round(x / 10 ** e) * 10 ** e))
    return sorted(set(out))


def ov_env(plot_dir, prefix, title, algo="", align="se3"):
    env = dict(os.environ)
    env.update({
        "PYTHONPATH": f"{EVAL}:" + env.get("PYTHONPATH", ""),
        "MPLBACKEND": "module://ov_eval_save_backend",
        "OVEVAL_PLOT_DIR": str(plot_dir), "OVEVAL_PLOT_PREFIX": prefix,
        "OVEVAL_PLOT_TITLE": title, "OVEVAL_ALGO": algo, "OVEVAL_BODY": "cam0", "OVEVAL_ALIGN": align,
        "OVEVAL_LABELS": json.dumps(LABELS),
    })
    env.pop("DISPLAY", None)
    return env


def run(cmd, env, log):
    with open(log, "w") as f:
        r = subprocess.run(cmd, env=env, cwd=EVAL, stdout=f, stderr=subprocess.STDOUT)
    if r.returncode != 0:
        print(f"  [!] {cmd[0].name if hasattr(cmd[0], 'name') else cmd[0]} uscito con {r.returncode}, vedi {log}")
    return Path(log).read_text()


def parse_singlerun(txt):
    txt = re.sub(r"\x1b\[[0-9;]*m", "", txt)
    out = {}
    m = re.search(r"Absolute Trajectory Error.*?rmse_ori = ([\d.]+) \| rmse_pos = ([\d.]+)", txt, re.S)
    if m:
        out["ate_ori_deg"], out["ate_pos_m"] = float(m.group(1)), float(m.group(2))
    for seg, ori, pos, n in re.findall(r"seg ([\d.]+) - median_ori = ([\d.]+) \| median_pos = ([\d.]+) \((\d+) samples\)", txt):
        out[f"rpe_{seg}m_pos_med_m"] = float(pos)
        out[f"rpe_{seg}m_ori_med_deg"] = float(ori)
    m = re.search(r"Normalized Estimation Error Squared.*?mean_ori = ([\d.]+) \| mean_pos = ([\d.]+)", txt, re.S)
    if m:
        out["nees_ori"], out["nees_pos"] = float(m.group(1)), float(m.group(2))
    return out


def evaluate(ds, info, align):
    res_dir = EVAL / f"results/batch3/{ds}/{align}"
    res_dir.mkdir(parents=True, exist_ok=True)
    segs = nice_segments(info["path"])
    os.environ["OVEVAL_SEGMENTS"] = ",".join(f"{s:g}" for s in segs)
    window = f"finestra {max(info['t_start'], 0):.1f}-{info['t_end']:.1f} s, percorso GT {info['path']:.1f} m"
    rows = []
    for algo in info["algos"]:
        title = f"{ds}: {LABELS[algo][0]} vs TagSLAM - allineamento {align}, {window}"
        txt = run([str(OV_BIN / "error_singlerun"), align, f"truths/{ds}.txt", f"algorithms/{algo}/{ds}/run_1.txt"],
                  ov_env(res_dir / algo, "error_singlerun", title, algo, align), res_dir / f"{algo}_error_singlerun.txt")
        r = {"dataset": ds, "algo": algo, "align": align, "window_s": f"{max(info['t_start'], 0):.1f}-{info['t_end']:.1f}",
             "gt_path_m": round(info["path"], 2), "segments_m": os.environ["OVEVAL_SEGMENTS"]}
        r.update(parse_singlerun(txt))
        with open(EVAL / f"algorithms/{algo}/{ds}/run_1.txt") as f:
            f.readline()
            if len(f.readline().split()) < 20:     # senza covarianza ov_eval stampa NEES = 0: non e' un valore
                r.pop("nees_pos", None)
                r.pop("nees_ori", None)
        rows.append(r)

    # traiettorie: file con nomi = chiavi di LABELS, cosi' la legenda di plot_trajectories e' leggibile
    tmp = Path(f"/tmp/ovplot/{ds}")
    tmp.mkdir(parents=True, exist_ok=True)
    for f in tmp.glob("*.txt"):
        f.unlink()
    (tmp / "tagslam_gt.txt").symlink_to(EVAL / f"truths/{ds}.txt")
    ests = []
    for algo in info["algos"]:
        (tmp / f"{algo}.txt").symlink_to(EVAL / f"algorithms/{algo}/{ds}/run_1.txt")
        ests.append(str(tmp / f"{algo}.txt"))
    run([str(OV_BIN / "plot_trajectories"), align, str(tmp / "tagslam_gt.txt"), *ests],
        ov_env(res_dir / "trajectories", "plot_trajectories", f"{ds}: traiettorie allineate ({align}) a TagSLAM, {window}", "", align),
        res_dir / "plot_trajectories.txt")

    # confronto: cartelle con il solo dataset ds, per avere sotto-segmenti RPE adatti alla sua lunghezza
    cmp_dir = Path(f"/tmp/ovcmp/{ds}")
    subprocess.run(["rm", "-rf", str(cmp_dir)])
    (cmp_dir / "truths").mkdir(parents=True)
    (cmp_dir / "truths" / f"{ds}.txt").symlink_to(EVAL / f"truths/{ds}.txt")
    for algo in info["algos"]:
        (cmp_dir / "algorithms" / algo / ds).mkdir(parents=True)
        (cmp_dir / "algorithms" / algo / ds / "run_1.txt").symlink_to(EVAL / f"algorithms/{algo}/{ds}/run_1.txt")
    run([str(OV_BIN / "error_comparison"), align, str(cmp_dir / "truths"), str(cmp_dir / "algorithms")],
        ov_env(res_dir / "comparison", "error_comparison", f"{ds}: RPE a confronto, allineamento {align}, {window}", "", align),
        res_dir / "error_comparison.txt")
    return rows


def assoc(gt, est, max_dt=0.02):
    idx = np.clip(np.searchsorted(est[:, 0], gt[:, 0]), 1, len(est) - 1)
    idx = np.where(np.abs(est[idx - 1, 0] - gt[:, 0]) < np.abs(est[idx, 0] - gt[:, 0]), idx - 1, idx)
    ok = np.abs(est[idx, 0] - gt[:, 0]) < max_dt
    return gt[ok], est[idx[ok]]


def umeyama_scale(src, dst):
    """Scala della similitudine (Umeyama) che porta src su dst."""
    ms, md = src.mean(0), dst.mean(0)
    xs, xd = src - ms, dst - md
    U, D, Vt = np.linalg.svd(xd.T @ xs / len(src))
    S = np.eye(src.shape[1])
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1
    return float(np.trace(np.diag(D) @ S) / (xs ** 2).sum(1).mean())


def extra_metrics():
    """Metriche fuori da ov_eval, sui file gia' scritti in truths/ e algorithms/."""
    out = {"gt": {}, "hover_static": {}, "ekf2_scale": {}, "loop_closure": {}}
    for ds, (test, _, algos) in DATASETS.items():
        gt = np.loadtxt(EVAL / f"truths/{ds}.txt")
        p = gt[:, 1:4]
        out["gt"][ds] = dict(duration_s=float(gt[-1, 0] - gt[0, 0]),
                             path_m=float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1))),
                             start_end_dist_m=float(np.linalg.norm(p[-1] - p[0])), z_range_m=float(np.ptp(p[:, 2])))
        if "ekf2" in algos:
            g, e = assoc(gt, np.loadtxt(EVAL / f"algorithms/ekf2/{ds}/run_1.txt"))
            out["ekf2_scale"][ds] = dict(scale_3d=umeyama_scale(e[:, 1:4], g[:, 1:4]),
                                         scale_xy=umeyama_scale(e[:, 1:3], g[:, 1:3]))
    # chiusura dei 2 giri: partenza e arrivo nello stesso punto (GT e EKF2 sulla registrazione completa)
    gt = np.loadtxt(EVAL / "truths/batch3_2loops_full.txt")
    e = np.loadtxt(EVAL / "algorithms/ekf2/batch3_2loops_full/run_1.txt")
    e = e[(e[:, 0] >= gt[0, 0]) & (e[:, 0] <= gt[-1, 0])]
    for name, a in (("tagslam_gt", gt), ("ekf2", e)):
        p0 = a[a[:, 0] <= a[0, 0] + 1.0, 1:4].mean(0)
        p1 = a[a[:, 0] >= a[-1, 0] - 1.0, 1:4].mean(0)
        out["loop_closure"][name] = dict(dist_m=float(np.linalg.norm(p1 - p0)), dxy_m=float(np.linalg.norm((p1 - p0)[:2])),
                                         dz_m=float((p1 - p0)[2]))
    # hover: drift e jitter da fermo dopo il sollevamento, nel frame di ciascuno stimatore (z verso l'alto per tutti).
    # Da 20 s: il sollevamento finisce a ~12 s, ma la quota di EKF2 si assesta solo verso i 20 s (transitorio del rangefinder)
    t0 = bag_start("hover")
    for algo in ["tagslam_gt", *DATASETS["batch3_hover"][2]]:
        f = EVAL / ("truths/batch3_hover.txt" if algo == "tagslam_gt" else f"algorithms/{algo}/batch3_hover/run_1.txt")
        a = np.loadtxt(f)
        a = a[a[:, 0] >= t0 + 20.0]
        p = a[:, 1:4]
        p0, p1 = p[a[:, 0] <= a[0, 0] + 1.0].mean(0), p[a[:, 0] >= a[-1, 0] - 1.0].mean(0)
        out["hover_static"][algo] = dict(window_s=f"20.0-{a[-1, 0] - t0:.1f}", drift_m=float(np.linalg.norm(p1 - p0)),
                                         drift_z_m=float((p1 - p0)[2]), std_xyz_mm=[float(x) for x in p.std(0) * 1000])
    return out


def main():
    convert_all()
    u, rows, max_dev = vertical()
    R_G_tag = leveling_rotation(u)
    lev = {"up_in_tag0_frame": u.tolist(), "tilt_of_tag0_y_deg": float(np.degrees(np.arccos(u[1]))),
           "max_dev_between_estimates_deg": float(max_dev),
           "estimates": [dict(test=t, algo=a, up=uu.tolist(), spread_deg=float(s), n=n) for t, a, uu, s, n in rows],
           "R_G_tag": R_G_tag.tolist()}
    (EVAL / "results/batch3").mkdir(parents=True, exist_ok=True)
    (EVAL / "results/batch3/gt_leveling.json").write_text(json.dumps(lev, indent=2))
    print(f"verticale nel mondo del tag 0: {np.round(u, 5)}, inclinazione {lev['tilt_of_tag0_y_deg']:.2f} deg, "
          f"dispersione max {max_dev:.2f} deg")

    allrows, infos = [], {}
    for ds in DATASETS:
        info = build_dataset(ds, R_G_tag)
        infos[ds] = {k: v for k, v in info.items() if k != "algos"}
        print(f"=== {ds}: {info['t_start']:.1f}-{info['t_end']:.1f} s, {info['n_gt']} pose GT, percorso {info['path']:.2f} m")
        for align in ALIGNS:
            allrows += evaluate(ds, info, align)
    base = ["dataset", "algo", "align", "window_s", "gt_path_m", "ate_pos_m", "ate_ori_deg", "nees_pos", "nees_ori", "segments_m"]
    with open(EVAL / "results/batch3/summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(base)
        for r in allrows:
            w.writerow([r.get(k, "") if k != "segments_m" else r[k].replace(",", ";") for k in base])
    with open(EVAL / "results/batch3/rpe.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["dataset", "algo", "align", "segment_m", "rpe_pos_median_m", "rpe_ori_median_deg"])
        for r in allrows:
            for seg in r["segments_m"].split(","):
                key = f"rpe_{seg}m"
                if f"{key}_pos_med_m" in r:
                    w.writerow([r["dataset"], r["algo"], r["align"], seg, r[f"{key}_pos_med_m"], r[f"{key}_ori_med_deg"]])
    (EVAL / "results/batch3/datasets.json").write_text(json.dumps({"datasets": infos, "extra": extra_metrics()}, indent=2))
    for r in allrows:
        print(f"  {r['dataset']:20s} {r['align']:6s} {r['algo']:20s} ATE pos {r.get('ate_pos_m', float('nan')):.3f} m, "
              f"ori {r.get('ate_ori_deg', float('nan')):.2f} deg, NEES pos {r.get('nees_pos', float('nan'))}")


if __name__ == "__main__":
    main()
