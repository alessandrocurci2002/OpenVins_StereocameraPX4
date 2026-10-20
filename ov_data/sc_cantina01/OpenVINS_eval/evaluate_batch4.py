#!/usr/bin/env python3
"""
Valutazione del batch04 (cantina, 09/10/2026, motori del drone accesi): TagSLAM (GT) contro
OpenVINS offline tarato per le vibrazioni (config_b4) ed EKF2 del Pixhawk. Da lanciare DENTRO il
container ROS2:
    cd /root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval && python3 evaluate_batch4.py

Ingressi:
    truths/<ds>.txt                                   TagSLAM /tagslam/odom/body_rig (extract_gt.py), posa di cam0
    tuning_b4/runs/<RUN_OV>_*/ov_estimate*.txt         OpenVINS config_b4 (run di tuning scelte, una per bag)
    rosbags/<bag>/                                    bag ROS2 con /fmu/out/vehicle_odometry (EKF2)
Uscite:
    algorithms/{config_b4,ekf2,config_v9}/<ds>/run_1.txt
    results/batch4/<ds>/<align>/...   log e grafici di ov_eval (error_singlerun, plot_trajectories, error_comparison)
    results/batch4/summary.csv, results/batch4/drift.csv
Allineamenti: se3 (Umeyama sulle posizioni, come dataset2-5) e se3ori (rotazione dalle orientazioni,
traslazione dai baricentri, aggiunto in ov_eval nel batch3).
Drift di ritorno: il drone parte e torna nello stesso punto (GT: 0.8 cm e 0.7 cm). Per ogni stimatore e'
la distanza tra la posizione media nella fase a riposo iniziale e quella nella fase a riposo finale, nel
suo frame. OpenVINS non pubblica nella fase iniziale (ZUPT): li' vale l'origine dell'init, fatta da fermo.
"""
import csv
import json
import os
import re
import subprocess
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from ekf2_to_pose import convert as ekf2_convert
from state_to_pose import convert as state_convert
from tagslam_to_truth import leveling_rotation, up_in_tag_frame

EVAL = Path(__file__).resolve().parent
OV_BIN = Path("/root/colcon_ws/install/ov_eval/lib/ov_eval")
IMUCAM = EVAL / "rosbags/cantina_03/Openvins_config_v9/kalibr_imucam_chain.yaml"
# dataset -> bag, run OpenVINS tarata, run config_v9 (diverge), fasi a riposo [s dall'inizio GT]
DATASETS = {
    "batch_4_2loops_motor_on": dict(bag="batch_4_2loops_motor_on/rec_20261009_091521", ov="d_an09_feat", v9="l_v9_rt",
                                    rest0=(0.0, 24.9), rest1=(151.1, 176.0)),
    "batch_4_2loops_6dof_motor_on": dict(bag="batch_4_2loops_6dof_motor_on_sshdownload", ov="e6_d_an09_feat", v9="e6_l_v9",
                                         rest0=(0.0, 22.5), rest1=(193.2, 231.4)),
}
ALIGNS = ["se3ori", "se3"]
LABELS = {
    "tagslam_gt": ["TagSLAM (GT)", "#0b0b0b"],
    "config_b4": ["OpenVINS offline config_b4", "#2a78d6"],
    "ekf2": ["EKF2 PX4 (flow + range + mag)", "#1baf7a"],
    "config_v9": ["OpenVINS offline config_v9", "#eb6834"],
}


def header_of(path):
    with open(path) as f:
        return f.readline().rstrip("\n")


def convert_all():
    for ds, d in DATASETS.items():
        run = EVAL / "tuning_b4/runs" / d["ov"]
        state_convert(str(run / "ov_estimate.txt"), str(EVAL / f"algorithms/config_b4/{ds}/run_1.txt"),
                      str(run / "ov_estimate_std.txt"), "cam0")
        v9 = EVAL / "tuning_b4/runs" / d["v9"]
        state_convert(str(v9 / "ov_estimate.txt"), str(EVAL / f"algorithms/config_v9/{ds}/run_1.txt"),
                      str(v9 / "ov_estimate_std.txt"), "cam0")
        ekf2_convert(str(EVAL / "rosbags" / d["bag"]), str(EVAL / f"algorithms/ekf2/{ds}/run_1.txt"),
                     str(EVAL / "px4_oak_calib.yaml"), str(IMUCAM), [0.0, 0.0, 0.0])


def nice_segments(path):
    out = []
    for f in (0.1, 0.2, 0.3, 0.4, 0.5):
        x = path * f
        e = np.floor(np.log10(x)) - 1
        out.append(float(np.round(x / 10 ** e) * 10 ** e))
    return sorted(set(out))


def ov_env(plot_dir, prefix, title, algo="", align="se3"):
    env = dict(os.environ)
    env.update({"PYTHONPATH": f"{EVAL}:" + env.get("PYTHONPATH", ""), "MPLBACKEND": "module://ov_eval_save_backend",
                "OVEVAL_PLOT_DIR": str(plot_dir), "OVEVAL_PLOT_PREFIX": prefix, "OVEVAL_PLOT_TITLE": title,
                "OVEVAL_ALGO": algo, "OVEVAL_BODY": "cam0", "OVEVAL_ALIGN": align, "OVEVAL_LABELS": json.dumps(LABELS)})
    env.pop("DISPLAY", None)
    return env


def run(cmd, env, log):
    with open(log, "w") as f:
        subprocess.run(cmd, env=env, cwd=EVAL, stdout=f, stderr=subprocess.STDOUT)
    return Path(log).read_text()


def parse_singlerun(txt):
    txt = re.sub(r"\x1b\[[0-9;]*m", "", txt)
    out = {}
    m = re.search(r"Absolute Trajectory Error.*?rmse_ori = ([\d.]+) \| rmse_pos = ([\d.]+)", txt, re.S)
    if m:
        out["ate_ori_deg"], out["ate_pos_m"] = float(m.group(1)), float(m.group(2))
    for seg, ori, pos, n in re.findall(r"seg ([\d.]+) - median_ori = ([\d.]+) \| median_pos = ([\d.]+) \((\d+) samples\)", txt):
        out[f"rpe_{seg}m_pos_med_m"], out[f"rpe_{seg}m_ori_med_deg"] = float(pos), float(ori)
    m = re.search(r"Normalized Estimation Error Squared.*?mean_ori = ([\d.]+) \| mean_pos = ([\d.]+)", txt, re.S)
    if m:
        out["nees_ori"], out["nees_pos"] = float(m.group(1)), float(m.group(2))
    return out


def leveled_gt(ds):
    """GT ruotata con z verticale (verticale stimata dall'orientazione di OpenVINS config_b4): solo per i grafici."""
    gt = np.loadtxt(EVAL / f"truths/{ds}.txt")
    u, spread, n = up_in_tag_frame(gt, np.loadtxt(EVAL / f"algorithms/config_b4/{ds}/run_1.txt"))
    Rg = leveling_rotation(u)
    p = gt[:, 1:4] @ Rg.T
    q = (R.from_matrix(Rg) * R.from_quat(gt[:, 4:8])).as_quat()
    return np.column_stack([gt[:, 0], p, q]), float(spread)


def evaluate(ds, algos, align):
    res = EVAL / f"results/batch4/{ds}/{align}"
    res.mkdir(parents=True, exist_ok=True)
    gt = np.loadtxt(EVAL / f"truths/{ds}.txt")
    path = float(np.sum(np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1)))
    os.environ["OVEVAL_SEGMENTS"] = ",".join(f"{s:g}" for s in nice_segments(path))
    window = f"{gt[-1, 0] - gt[0, 0]:.0f} s, percorso GT {path:.1f} m"
    rows = []
    for algo in algos:
        txt = run([str(OV_BIN / "error_singlerun"), align, f"truths/{ds}.txt", f"algorithms/{algo}/{ds}/run_1.txt"],
                  ov_env(res / algo, "error_singlerun", f"{ds}: {LABELS[algo][0]} vs TagSLAM - allineamento {align}, {window}",
                         algo, align), res / f"{algo}_error_singlerun.txt")
        r = {"dataset": ds, "algo": algo, "align": align, "gt_path_m": round(path, 2), "segments_m": os.environ["OVEVAL_SEGMENTS"]}
        r.update(parse_singlerun(txt))
        with open(EVAL / f"algorithms/{algo}/{ds}/run_1.txt") as f:
            f.readline()
            if len(f.readline().split()) < 20:
                r.pop("nees_pos", None)
                r.pop("nees_ori", None)
        rows.append(r)
    # traiettorie con la GT livellata (vista dall'alto leggibile); le metriche se3/se3ori non cambiano
    tmp = Path(f"/tmp/ovplot_b4/{ds}")
    subprocess.run(["rm", "-rf", str(tmp)])
    tmp.mkdir(parents=True)
    lev, _ = leveled_gt(ds)
    np.savetxt(tmp / "tagslam_gt.txt", lev, fmt="%.6f", header="timestamp(s) tx ty tz qx qy qz qw")
    ests = []
    for algo in algos:
        (tmp / f"{algo}.txt").symlink_to(EVAL / f"algorithms/{algo}/{ds}/run_1.txt")
        ests.append(str(tmp / f"{algo}.txt"))
    run([str(OV_BIN / "plot_trajectories"), align, str(tmp / "tagslam_gt.txt"), *ests],
        ov_env(res / "trajectories", "plot_trajectories", f"{ds}: traiettorie allineate ({align}) a TagSLAM, {window}", "", align),
        res / "plot_trajectories.txt")
    cmp = Path(f"/tmp/ovcmp_b4/{ds}")
    subprocess.run(["rm", "-rf", str(cmp)])
    (cmp / "truths").mkdir(parents=True)
    (cmp / "truths" / f"{ds}.txt").symlink_to(EVAL / f"truths/{ds}.txt")
    for algo in algos:
        (cmp / "algorithms" / algo / ds).mkdir(parents=True)
        (cmp / "algorithms" / algo / ds / "run_1.txt").symlink_to(EVAL / f"algorithms/{algo}/{ds}/run_1.txt")
    run([str(OV_BIN / "error_comparison"), align, str(cmp / "truths"), str(cmp / "algorithms")],
        ov_env(res / "comparison", "error_comparison", f"{ds}: RPE a confronto, allineamento {align}, {window}", "", align),
        res / "error_comparison.txt")
    return rows


def drift(ds):
    d = DATASETS[ds]
    gt = np.loadtxt(EVAL / f"truths/{ds}.txt")
    t0 = gt[0, 0]
    out = []
    for name, f in (("tagslam_gt", EVAL / f"truths/{ds}.txt"), ("config_b4", EVAL / f"algorithms/config_b4/{ds}/run_1.txt"),
                    ("ekf2", EVAL / f"algorithms/ekf2/{ds}/run_1.txt")):
        a = np.loadtxt(f)
        t = a[:, 0] - t0
        m0 = (t >= d["rest0"][0] + 0.5) & (t <= d["rest0"][1] - 0.5)
        m1 = (t >= d["rest1"][0] + 1.0) & (t <= d["rest1"][1] - 0.5)
        if name == "config_b4":   # nessuna uscita a riposo: origine dell'init, portata su cam0 come il resto
            ov = np.loadtxt(EVAL / f"tuning_b4/runs/{d['ov']}/ov_estimate.txt", usecols=range(17))
            to = ov[:, 0] - t0
            p0 = np.zeros(3)
            p1 = ov[(to >= d["rest1"][0] + 1.0) & (to <= d["rest1"][1] - 0.5), 5:8].mean(0)
            note = "IMU, origine init"
        else:
            p0, p1 = a[m0, 1:4].mean(0), a[m1, 1:4].mean(0)
            note = "cam0"
        dp = p1 - p0
        out.append(dict(dataset=ds, algo=name, drift_m=float(np.linalg.norm(dp)), drift_xy_m=float(np.hypot(*dp[:2])),
                        drift_z_m=float(dp[2]), note=note))
    return out


def main():
    convert_all()
    allrows, drows = [], []
    for ds in DATASETS:
        for align in ALIGNS:
            allrows += evaluate(ds, ["config_b4", "ekf2"], align)
        drows += drift(ds)
    base = ["dataset", "algo", "align", "gt_path_m", "ate_pos_m", "ate_ori_deg", "nees_pos", "nees_ori", "segments_m"]
    (EVAL / "results/batch4").mkdir(parents=True, exist_ok=True)
    with open(EVAL / "results/batch4/summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(base)
        for r in allrows:
            w.writerow([r.get(k, "") if k != "segments_m" else r[k].replace(",", ";") for k in base])
    with open(EVAL / "results/batch4/rpe.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["dataset", "algo", "align", "segment_m", "rpe_pos_median_m", "rpe_ori_median_deg"])
        for r in allrows:
            for seg in r["segments_m"].split(","):
                if f"rpe_{seg}m_pos_med_m" in r:
                    w.writerow([r["dataset"], r["algo"], r["align"], seg, r[f"rpe_{seg}m_pos_med_m"], r[f"rpe_{seg}m_ori_med_deg"]])
    with open(EVAL / "results/batch4/drift.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(drows[0]))
        w.writeheader()
        w.writerows(drows)
    for r in allrows:
        print(f"  {r['dataset']:30s} {r['align']:6s} {r['algo']:10s} ATE {r.get('ate_pos_m', float('nan')) * 100:6.1f} cm "
              f"{r.get('ate_ori_deg', float('nan')):5.2f} deg  NEES {r.get('nees_pos', '')}")
    for r in drows:
        print(f"  drift {r['dataset']:30s} {r['algo']:10s} {r['drift_m'] * 100:6.1f} cm (xy {r['drift_xy_m'] * 100:5.1f}, z {r['drift_z_m'] * 100:+5.1f})")


if __name__ == "__main__":
    main()
