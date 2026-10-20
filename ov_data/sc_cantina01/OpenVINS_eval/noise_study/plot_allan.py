#!/usr/bin/env python3
"""
Allan deviation (media rms sui 3 assi) dei segmenti stimati da imu_noise.py, con le rette del rumore
bianco N/sqrt(tau) dei candidati. Uso: python3 noise_study/plot_allan.py  (da OpenVINS_eval, nel container)
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

OUT = Path(__file__).resolve().parent.parent / "results" / "noise_study"
SEGS = [("flight", "static_disarmed", "volo: fermo, disarmato (OAK grezza)", "tab:green"),
        ("flight", "armed_ground", "volo: armato a terra (residuo vs Pixhawk)", "tab:olive"),
        ("batch4", "armed", "batch4 2loops: a mano, motori accesi (residuo)", "tab:blue"),
        ("flight", "flight", "volo: in aria 14-93 s (residuo vs Pixhawk)", "tab:red")]
CAND = {"P0 attuale": (0.09, 0.0015), "P1 statico x20": (0.026, 0.0036), "P2 dinamico": (0.35, 0.003),
        "P3 inviluppo": (0.6, 0.0045)}


def main():
    fig, ax = plt.subplots(1, 2, figsize=(14, 5.5))
    for f, seg, lab, col in SEGS:
        p = OUT / f"{f}_noise.npz"
        if not p.exists():
            continue
        d = np.load(p)
        if f"{seg}_acc_adev" not in d:
            continue
        taus = d["taus"]
        for a_, k in zip(ax, ("acc", "gyro")):
            ad = d[f"{seg}_{k}_adev"]
            a_.loglog(taus, np.sqrt(np.nanmean(ad ** 2, axis=1)), "o-", ms=3, color=col, label=lab)
    tt = np.logspace(-2, 1.5, 50)
    for (name, (na, ng)), ls in zip(CAND.items(), ["--", ":", "-.", (0, (5, 1))]):
        ax[0].loglog(tt, na / np.sqrt(tt), color="k", ls=ls, lw=1, label=f"{name}: N={na}")
        ax[1].loglog(tt, ng / np.sqrt(tt), color="k", ls=ls, lw=1, label=f"{name}: N={ng}")
    for a_, u in zip(ax, ("m/s^2", "rad/s")):
        a_.axvline(1 / 30, color="gray", lw=0.8)
        a_.text(1 / 30, a_.get_ylim()[0] * 1.5, " 1 frame", color="gray", fontsize=8)
        a_.axvline(11 / 30, color="gray", lw=0.8, ls="--")
        a_.text(11 / 30, a_.get_ylim()[0] * 1.5, " 11 cloni", color="gray", fontsize=8)
        a_.set_xlabel("tau [s]")
        a_.set_ylabel(f"Allan deviation [{u}]")
        a_.grid(which="both", alpha=0.3)
    ax[0].set_title("Accelerometro OAK")
    ax[1].set_title("Giroscopio OAK")
    ax[0].legend(fontsize=7)
    ax[1].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(OUT / "allan_candidates.png", dpi=120)
    print(OUT / "allan_candidates.png")


if __name__ == "__main__":
    main()
