#!/usr/bin/env python3
"""
Valuta le run del tuning dei rumori IMU (tuning_v9/runs/<nome>) contro la GT di TagSLAM.

Per ogni run:
  1. converte ov_estimate.txt in formato ov_eval con state_to_pose.py (frame cam0, come
     la GT /tagslam/odom/body_rig) in tuning_v9/algorithms/<nome>/dataset3/run_1.txt
  2. lancia ov_eval error_singlerun (allineamento se3: la mappa di TagSLAM non e' allineata
     alla gravita') nel container, senza finestre di matplotlib
  3. calcola il drift finale: distanza tra l'origine dell'init (camera a riposo prima del
     sollevamento) e la posizione media a riposo dopo che la camera e' stata riappoggiata
     (TagSLAM conferma che i due punti coincidono entro 0.5 cm)

Score = ATE_pos / ATE_pos(base) + drift / drift(base): pesa allo stesso modo l'errore lungo
tutta la traiettoria e il drift finale.

Uso (dall'host): python3 evaluate.py [nomi run ...]   (default: tutte le run complete)
"""

import math
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EVAL = HERE.parent
sys.path.insert(0, str(EVAL))
from state_to_pose import convert  # noqa: E402

CONTAINER = "provola1"
EVAL_IN_CONTAINER = "/root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval"
T_LIFT = 1791191068.15889          # prima riga di OpenVINS (sollevamento)
T_REST_END = T_LIFT + 191.5        # camera di nuovo ferma (da IMU)
MIN_ROWS = 5900


def endpoint_drift(est_path):
    rows = [[float(v) for v in line.split()[:8]] for line in open(est_path) if not line.startswith("#")]
    rest = [r[5:8] for r in rows if r[0] >= T_REST_END]
    end = [sum(p[k] for p in rest) / len(rest) for k in range(3)]
    return math.dist(end, [0, 0, 0]), math.hypot(end[0], end[1]), end[2]


def run_ov_eval(names):
    cmds = " ; ".join(
        f"/root/colcon_ws/install/ov_eval/lib/ov_eval/error_singlerun se3 truths/dataset3.txt "
        f"tuning_v9/algorithms/{n}/dataset3/run_1.txt > tuning_v9/runs/{n}/eval_se3.log 2>&1"
        for n in names)
    subprocess.run(["docker", "exec", CONTAINER, "bash", "-c",
                    "source /opt/ros/humble/setup.bash; source /root/colcon_ws/install/setup.bash; "
                    f"unset DISPLAY; export MPLBACKEND=Agg; cd {EVAL_IN_CONTAINER}; {cmds}"], check=True)


def parse_eval(log_path):
    txt = Path(log_path).read_text()
    blocks = txt.split("======================================")
    out = {}
    ate = re.search(r"Absolute Trajectory Error.*?rmse_ori = ([\d.]+) \| rmse_pos = ([\d.]+)", txt, re.S)
    out["ate_ori"], out["ate_pos"] = float(ate.group(1)), float(ate.group(2))
    for seg, ori, pos in re.findall(r"seg (\d+) - median_ori = ([\d.]+) \| median_pos = ([\d.]+)", txt):
        out[f"rpe{seg}_pos"] = float(pos)
    nees = re.search(r"Normalized Estimation Error Squared.*?mean_ori = ([\d.]+) \| mean_pos = ([\d.]+)", txt, re.S)
    if nees:
        out["nees_ori"], out["nees_pos"] = float(nees.group(1)), float(nees.group(2))
    return out


def main():
    runs_dir = HERE / "runs"
    names = sys.argv[1:] or sorted(p.name for p in runs_dir.iterdir() if (p / "ov_estimate.txt").exists())
    ok = []
    for n in names:
        est = runs_dir / n / "ov_estimate.txt"
        nrows = sum(1 for line in open(est) if not line.startswith("#"))
        if nrows < MIN_ROWS:
            print(f"[skip] {n}: {nrows} righe (run incompleta?)")
            continue
        convert(str(est), str(HERE / "algorithms" / n / "dataset3" / "run_1.txt"),
                str(runs_dir / n / "ov_estimate_std.txt"), "cam0")
        ok.append(n)
    run_ov_eval(ok)

    res = []
    for n in ok:
        r = {"name": n, "noise": (runs_dir / n / "noise.txt").read_text().split()}
        r.update(parse_eval(runs_dir / n / "eval_se3.log"))
        r["drift"], r["drift_h"], r["drift_v"] = endpoint_drift(runs_dir / n / "ov_estimate.txt")
        res.append(r)

    base = next((r for r in res if r["name"] == "base"), None)
    if base is None:
        base_line = (HERE / "summary.csv")
        ref = None
        if base_line.exists():
            for line in base_line.read_text().splitlines()[1:]:
                f = line.split(",")
                if f[0] == "base":
                    ref = (float(f[5]), float(f[7]))
        ref = ref or (res[0]["ate_pos"], res[0]["drift"])
    else:
        ref = (base["ate_pos"], base["drift"])
    for r in res:
        r["score"] = r["ate_pos"] / ref[0] + r["drift"] / ref[1]

    header = "name,acc_nd,acc_rw,gyr_nd,gyr_rw,ate_pos_m,ate_ori_deg,drift_m,drift_h_m,drift_v_m,rpe8_pos_m,rpe40_pos_m,nees_pos,nees_ori,score"
    summary = HERE / "summary.csv"
    old = {}
    if summary.exists():
        for line in summary.read_text().splitlines()[1:]:
            old[line.split(",")[0]] = line
    for r in res:
        old[r["name"]] = ",".join([r["name"], *r["noise"]] + [
            f"{r['ate_pos']:.4f}", f"{r['ate_ori']:.3f}", f"{r['drift']:.4f}", f"{r['drift_h']:.4f}",
            f"{r['drift_v']:+.4f}", f"{r.get('rpe8_pos', float('nan')):.4f}", f"{r.get('rpe40_pos', float('nan')):.4f}",
            f"{r.get('nees_pos', float('nan')):.2f}", f"{r.get('nees_ori', float('nan')):.2f}", f"{r['score']:.3f}"])
    summary.write_text(header + "\n" + "\n".join(old.values()) + "\n")

    print(f"{'run':12s} {'acc_nd':>7s} {'acc_rw':>7s} {'gyr_nd':>7s} {'gyr_rw':>7s} | {'ATE cm':>6s} {'ATEori':>6s} "
          f"{'drift':>6s} {'oriz':>5s} {'vert':>6s} | {'NEESp':>6s} | score")
    for r in sorted(res, key=lambda r: r["score"]):
        print(f"{r['name']:12s} {r['noise'][0]:>7s} {r['noise'][1]:>7s} {r['noise'][2]:>7s} {r['noise'][3]:>7s} | "
              f"{r['ate_pos']*100:6.1f} {r['ate_ori']:6.2f} {r['drift']*100:6.1f} {r['drift_h']*100:5.1f} "
              f"{r['drift_v']*100:+6.1f} | {r.get('nees_pos', float('nan')):6.1f} | {r['score']:.3f}")


if __name__ == "__main__":
    main()
