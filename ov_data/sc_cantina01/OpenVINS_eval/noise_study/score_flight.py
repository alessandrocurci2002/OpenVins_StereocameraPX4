#!/usr/bin/env python3
"""
Valuta le repliche di OpenVINS sul volo (tuning_b4/runs/<nome>, bag prova_drone_20261009_172706_recovered)
confrontandole con EKF2 (/fmu/out/vehicle_local_position_v1 nello stesso bag: flusso ottico + range
finder, VIO non fusa). Senza ground truth, EKF2 e' un riferimento debole in orizzontale (scala del flusso)
ma buono in verticale (EKF2_HGT_REF = range finder).

Metriche (dal decollo, letto da vehicle_land_detected):
  - divergenza: primo istante con |v_OV| > 3 m/s; |v| massima;
  - transitorio: |v_OV| massima nei primi 8 s dopo il decollo;
  - verticale: RMSE di v_up rispetto a EKF2, NEES medio e^2/(sigma_OV^2 + evv^2) (atteso ~1),
    errore di quota (OV - EKF2, entrambi relativi al decollo): RMSE e valore finale;
  - orizzontale: yaw allineato ai minimi quadrati sulle velocita', RMSE e scala OV/EKF2 stimata;
  - ruvidita': RMS di v - media mobile 1 s (quello che EKF2 vedrebbe come rumore di misura);
  - bias: spostamento di b_a e b_g stimati tra il decollo e il volo;
  - costo: tempo per frame (righe [TIME] di openvins.log), mediana e p95.
Uso (container, da OpenVINS_eval): python3 noise_study/score_flight.py nome [nome ...]
"""
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import px4_bag  # noqa: E402

EVAL = Path(__file__).resolve().parent.parent
BAG = EVAL / "rosbags/prova_drone_20261009_172706_recovered"
RUNS = EVAL / "tuning_b4/runs"
OUT = EVAL / "results/noise_study"


def load_px4():
    ts = px4_bag.typestore()
    lp = px4_bag.read_topic(BAG, "/fmu/out/vehicle_local_position_v1", ts)
    ld = px4_bag.read_topic(BAG, "/fmu/out/vehicle_land_detected", ts)
    t_to = next(m.timestamp for _, m in ld if not m.landed) * 1e-6
    E = dict(t=np.array([m.timestamp_sample for _, m in lp]) * 1e-6,
             p=np.array([[m.y, m.x, -m.z] for _, m in lp]),           # NED -> ENU
             v=np.array([[m.vy, m.vx, -m.vz] for _, m in lp]),
             evv=np.array([m.evv for _, m in lp]), rng=np.array([m.dist_bottom for _, m in lp]))
    return E, t_to


def interp(tq, t, x):
    return np.stack([np.interp(tq, t, x[:, k]) for k in range(x.shape[1])], axis=1) if x.ndim == 2 else np.interp(tq, t, x)


def rough(t, v, win=1.0):
    out = []
    for k in range(v.shape[1]):
        m = np.array([v[(t >= ti - win / 2) & (t <= ti + win / 2), k].mean() for ti in t])
        out.append(v[:, k] - m)
    return float(np.sqrt((np.array(out) ** 2).sum(0).mean()))


def frame_times(log):
    s = [float(x) for x in re.findall(r"\[TIME\]: ([\d.]+) seconds total", log.read_text(errors="ignore"))] if log.exists() else []
    return (np.median(s) * 1e3, np.percentile(s, 95) * 1e3) if s else (np.nan, np.nan)


def score(name, E, t_to):
    run = RUNS / name
    X = np.loadtxt(run / "ov_estimate.txt", usecols=range(17))
    S = np.loadtxt(run / "ov_estimate_std.txt", usecols=range(16))
    t, p, v, bg, ba = X[:, 0], X[:, 5:8], X[:, 8:11], X[:, 11:14], X[:, 14:17]
    sv = S[:, 7:10]
    chain = (run / "kalibr_imu_chain.yaml").read_text()
    nz = [re.search(rf"^  {k}:\s*([\d.eE+-]+)", chain, re.M).group(1) for k in
          ("accelerometer_noise_density", "accelerometer_random_walk", "gyroscope_noise_density", "gyroscope_random_walk")]
    feat = [re.search(rf"^{k}:\s*(\d+)", (run / "estimator_config.yaml").read_text(), re.M).group(1)
            for k in ("num_pts", "max_msckf_in_update", "max_slam")]
    r = {"name": name, "noise": "/".join(nz), "feat": "/".join(feat)}
    sp = np.linalg.norm(v, axis=1)
    fl = t >= t_to
    div = t[fl & (sp > 3.0)]
    r["diverge_s"] = round(float(div[0] - E["t0"]), 1) if len(div) else None
    r["v_max"] = round(float(sp[fl].max()), 2)
    r["transient_vmax"] = round(float(sp[fl & (t <= t_to + 8)].max()), 2)
    t_end = div[0] if len(div) else min(t[-1], E["t"][-1])
    k = (t >= t_to + 3) & (t <= t_end)
    r["t_eval_s"] = round(float(t_end - t_to - 3), 1)
    Ev, Ep = interp(t[k], E["t"], E["v"]), interp(t[k], E["t"], E["p"])
    evv = np.interp(t[k], E["t"], E["evv"])
    e_vz = v[k, 2] - Ev[:, 2]
    r["vz_rmse"] = round(float(np.sqrt((e_vz ** 2).mean())), 3)
    r["vz_nees"] = round(float((e_vz ** 2 / (sv[k, 2] ** 2 + evv ** 2)).mean()), 2)
    r["sigma_vz_ov"] = round(float(np.median(sv[k, 2])), 3)
    nees = e_vz ** 2 / (sv[k, 2] ** 2 + evv ** 2)
    tw = t[k] - t[k][0]
    r["vz_nees_win"] = "/".join(f"{nees[(tw >= w) & (tw < w + 20)].mean():.2f}" for w in range(0, int(tw[-1]) - 9, 20))
    i0 = int(np.argmin(abs(t - t_to)))
    h_ov = p[k, 2] - p[i0, 2]
    h_ek = Ep[:, 2] - np.interp(t[i0], E["t"], E["p"][:, 2])
    r["h_rmse"] = round(float(np.sqrt(((h_ov - h_ek) ** 2).mean())), 3)
    r["h_end_err"] = round(float(h_ov[-1] - h_ek[-1]), 3)
    # yaw OV -> EKF2 sulle velocita' orizzontali (Procrustes 2D) e scala
    a, b = v[k, :2], Ev[:, :2]
    H = a.T @ b
    psi = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    Rz = np.array([[np.cos(psi), -np.sin(psi)], [np.sin(psi), np.cos(psi)]])
    ar = a @ Rz.T
    r["vh_rmse"] = round(float(np.sqrt(((ar - b) ** 2).sum(1).mean())), 3)
    r["scale_ov_ekf"] = round(float((ar * b).sum() / (b * b).sum()), 2)
    r["rough_ov"] = round(rough(t[k], v[k]), 3)
    r["rough_ekf"] = round(rough(E["t"][(E["t"] >= t_to + 3) & (E["t"] <= t_end)], E["v"][(E["t"] >= t_to + 3) & (E["t"] <= t_end)]), 3)
    pre = t <= t[0] + 1.0                                  # primo secondo di stima (dopo il decollo: OV pubblica da ~2 s dopo)
    if pre.any():
        r["dba"] = round(float(np.linalg.norm(ba[k].mean(0) - ba[pre].mean(0))), 3)
        r["dbg"] = round(float(np.linalg.norm(bg[k].mean(0) - bg[pre].mean(0))), 4)
        r["ba_span"] = round(float(np.linalg.norm(ba[k].max(0) - ba[k].min(0))), 3)
    r["frame_ms_med"], r["frame_ms_p95"] = (round(x, 1) for x in frame_times(run / "openvins.log"))
    return r, dict(t=t - E["t0"], sp=sp, vz=v[:, 2], h=p[:, 2] - p[0, 2], k=k)


def main():
    names = sys.argv[1:]
    E, t_to = load_px4()
    meta = __import__("yaml").safe_load(open(BAG / "metadata.yaml"))["rosbag2_bagfile_information"]
    E["t0"] = meta["starting_time"]["nanoseconds_since_epoch"] * 1e-9
    cols = ["name", "noise", "feat", "diverge_s", "v_max", "transient_vmax", "t_eval_s", "vz_rmse", "vz_nees", "vz_nees_win", "sigma_vz_ov", "h_rmse",
            "h_end_err", "vh_rmse", "scale_ov_ekf", "rough_ov", "rough_ekf", "dba", "ba_span", "dbg", "frame_ms_med", "frame_ms_p95"]
    rows, curves = [], {}
    for n in names:
        r, c = score(n, E, t_to)
        rows.append(r)
        curves[n] = c
        print("  ".join(f"{c_}={r.get(c_)}" for c_ in cols))
    OUT.mkdir(parents=True, exist_ok=True)
    csv = OUT / "flight_scores.csv"
    old = {}
    if csv.exists():
        for line in csv.read_text().splitlines()[1:]:
            old[line.split(";")[0]] = line
    for r in rows:
        old[r["name"]] = ";".join(str(r.get(c, "")) for c in cols)
    csv.write_text(";".join(cols) + "\n" + "\n".join(old[k] for k in sorted(old)) + "\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    te = E["t"] - E["t0"]
    ax[0].plot(te, np.linalg.norm(E["v"], axis=1), "k", lw=2, label="EKF2")
    ax[1].plot(te, E["v"][:, 2], "k", lw=2, label="EKF2")
    t_first = min(c["t"][0] for c in curves.values()) + E["t0"]      # quote allineate al primo istante di stima OV
    i0 = int(np.argmin(abs(E["t"] - t_first)))
    ax[2].plot(te, E["p"][:, 2] - E["p"][i0, 2], "k", lw=2, label="EKF2 (quota da range)")
    for n, c in curves.items():
        ax[0].plot(c["t"], c["sp"], lw=1, label=n)
        ax[1].plot(c["t"], c["vz"], lw=1)
        ax[2].plot(c["t"], c["h"], lw=1)
    ax[0].set_ylim(0, 3.5)
    ax[1].set_ylim(-1.5, 1.5)
    ax[2].set_ylim(-0.5, 3.2)
    for a_, lab in zip(ax, ["|v| [m/s]", "v_up [m/s]", "quota dal decollo [m]"]):
        a_.set_ylabel(lab)
        a_.axvline(t_to - E["t0"], color="gray", ls="--")
        a_.grid(alpha=0.3)
    ax[0].legend(fontsize=7, ncol=3)
    ax[2].set_xlabel("tempo dall'inizio del bag [s]")
    tag = sys.argv[1] if len(names) == 1 else "_".join(sorted({n.split("_")[1] for n in names}))
    fig.tight_layout()
    fig.savefig(OUT / f"flight_{tag}.png", dpi=110)


if __name__ == "__main__":
    main()
