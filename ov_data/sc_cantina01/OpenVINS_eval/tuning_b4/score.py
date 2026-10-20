#!/usr/bin/env python3
"""
Punteggio delle run di tuning sul batch4 (motori accesi). Da lanciare DENTRO il container ROS2:
    cd .../OpenVINS_eval/tuning_b4 && python3 score.py [nomi run ...]

Per ogni run (runs/<nome>/ov_estimate.txt):
  - completezza e divergenza: righe, |v| massima, primo istante con |v| > 1 m/s;
  - drift di ritorno: OpenVINS si inizializza da fermo (ZUPT) nella fase a riposo iniziale, quindi
    l'origine (p = 0) e' il punto di partenza; il punto d'arrivo e' la media della posizione nella fase
    a riposo finale. Fasi dal GT TagSLAM (velocita' < 2 cm/s), che le da' a 0.8 cm l'una dall'altra;
  - ATE e NEES con ov_eval error_singlerun (se3) contro truths/<dataset>.txt, posa di cam0.
Riepilogo in tuning_b4/summary.csv.
"""
import math
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
EVAL = HERE.parent
sys.path.insert(0, str(EVAL))
from state_to_pose import convert  # noqa: E402

# dataset -> (cartella bag relativa a rosbags, fine fase a riposo iniziale, inizio fase a riposo finale) [s dall'inizio GT]
DATASETS = {"batch_4_2loops_motor_on": ("batch_4_2loops_motor_on/rec_20261009_091521", 24.9, 151.1),
            "batch_4_2loops_6dof_motor_on": ("batch_4_2loops_6dof_motor_on_sshdownload", 22.5, 193.2),
            # cantina_03 a motori spenti (GT TagSLAM con dimensione tag misurata); init quando la camera viene sollevata
            "dataset4": ("cantina_03/tagslam_cantina03", 6.8, 199.5)}
OV_BIN = "/root/colcon_ws/install/ov_eval/lib/ov_eval/error_singlerun"


def dataset_of(run):
    """Nome del dataset (= file in truths/): da runs/<nome>/dataset.txt, default il 2loops."""
    f = run / "dataset.txt"
    return f.read_text().strip() if f.exists() else "batch_4_2loops_motor_on"


def score(name):
    run = HERE / "runs" / name
    ds = dataset_of(run)
    _, t_rest0, t_rest1 = DATASETS[ds]
    gt = np.loadtxt(EVAL / "truths" / f"{ds}.txt")
    T0, Tend = gt[0, 0], gt[-1, 0]
    est = run / "ov_estimate.txt"
    r = {"name": name, "dataset": ds, "overrides": (run / "overrides.txt").read_text().strip() if (run / "overrides.txt").exists() else ""}
    if not est.exists() or est.stat().st_size < 1000:
        r.update(rows=0, status="nessun output")
        return r
    E = np.loadtxt(est, usecols=range(17))
    t = E[:, 0] - T0
    v = np.linalg.norm(E[:, 8:11], axis=1)
    r["rows"] = len(E)
    r["t_last"] = round(t[-1], 1)
    r["v_max"] = round(float(v.max()), 2)
    div = t[v > 1.0]
    r["diverge_s"] = round(float(div[0]), 1) if len(div) else None
    gaps = np.diff(t)
    r["gaps_gt_0.2s"] = int((gaps > 0.2).sum())
    end = (t >= t_rest1 + 1.0) & (t <= Tend - T0 - 0.5)
    if end.any() and r["diverge_s"] is None:
        pe = E[end, 5:8].mean(axis=0)
        r["drift_cm"] = round(float(np.linalg.norm(pe)) * 100, 1)
        r["drift_h_cm"] = round(float(np.hypot(pe[0], pe[1])) * 100, 1)
        r["drift_v_cm"] = round(float(pe[2]) * 100, 1)
    complete = t[-1] > Tend - T0 - 2.0
    r["status"] = "divergente" if r["diverge_s"] is not None else ("ok" if complete else "incompleta")
    if r["status"] == "ok":
        out = run / "eval" / "run_1.txt"
        convert(str(est), str(out), str(run / "ov_estimate_std.txt"), "cam0")
        env = dict(os.environ, MPLBACKEND="Agg")
        env.pop("DISPLAY", None)
        log = subprocess.run(["bash", "-c", f"source /opt/ros/humble/setup.bash; source /root/colcon_ws/install/setup.bash; "
                                            f"{OV_BIN} se3 {EVAL}/truths/{ds}.txt {out}"],
                             capture_output=True, text=True, env=env).stdout
        (run / "eval" / "error_singlerun_se3.txt").write_text(log)
        m = re.search(r"rmse_ori = ([\d.]+) \| rmse_pos = ([\d.]+)", log)
        n = re.search(r"Normalized Estimation Error Squared.*?mean_ori = ([\d.]+) \| mean_pos = ([\d.]+)", log, re.S)
        if m:
            r["ate_pos_cm"] = round(float(m.group(2)) * 100, 1)
            r["ate_ori_deg"] = float(m.group(1))
        if n:
            r["nees_pos"] = float(n.group(2))
    return r


def main():
    names = sys.argv[1:] or sorted(p.name for p in (HERE / "runs").iterdir() if (p / "run.done").exists())
    cols = ["name", "status", "rows", "t_last", "v_max", "diverge_s", "gaps_gt_0.2s", "drift_cm", "drift_h_cm", "drift_v_cm",
            "ate_pos_cm", "ate_ori_deg", "nees_pos", "overrides"]
    summary = HERE / "summary.csv"
    old = {}
    if summary.exists():
        for line in summary.read_text().splitlines()[1:]:
            old[line.split(";")[0]] = line
    for n in names:
        r = score(n)
        old[n] = ";".join(str(r.get(c, "")) for c in cols)
        print("  ".join(f"{c}={r.get(c)}" for c in cols if r.get(c) not in (None, "")))
    summary.write_text(";".join(cols) + "\n" + "\n".join(old[k] for k in sorted(old)) + "\n")


if __name__ == "__main__":
    main()
