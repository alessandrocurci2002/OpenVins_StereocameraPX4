#!/usr/bin/env python3
"""
Cosa succede quando gli stream riprendono dopo un buco (batch03, bag registrati sul RPi5).

Per ogni bag guarda, attorno a ogni interruzione di /odomimu:
  - OpenVINS online (/odomimu): velocita', deviazioni standard di posizione / orientazione / velocita'
    (la covarianza che OpenVINS pubblica, cioe' quella del filtro propagata con l'IMU);
  - bridge (/fmu/in/vehicle_visual_odometry): se pubblica durante il buco, cosa manda alla ripresa,
    con quali varianze, reset_counter e quality;
  - EKF2: flag di fusione della visione esterna (cs_ev_*), flag di rigetto delle innovazioni,
    /fmu/out/vehicle_local_position (velocita', eph/evh, reset) e /fmu/out/vehicle_odometry.

Tempi = tempo di registrazione dall'inizio del bag. Nei buchi della registrazione non si vede nulla:
cosa e' successo li' sul drone si deduce solo da prima e dopo.

Da lanciare DENTRO il container ROS2:
    cd /root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval && python3 analyze_resume_batch3.py
Uscite: results/batch3/resume_analysis/{report.txt, <test>_resume.png}
"""
import math
import os
from pathlib import Path

import numpy as np

os.environ.setdefault("MPLBACKEND", "Agg")
import matplotlib.pyplot as plt  # noqa: E402

import px4_bag  # noqa: E402

EVAL = Path(__file__).resolve().parent
OUT = EVAL / "results" / "batch3" / "resume_analysis"
TESTS = {"2loops": "rec_20261007_152554", "hover": "rec_20261007_153315", "6dof": "rec_20261007_153538"}
GAP = {"/odomimu": 0.25, "/oak/imu/data": 0.25, "/oak/left/image_rect": 0.25,
       "/fmu/in/vehicle_visual_odometry": 0.25, "/fmu/out/vehicle_odometry": 0.25,
       "/fmu/out/vehicle_local_position_v1": 0.25, "/fmu/out/estimator_status_flags": 3.0}
EV_FLAGS = ["cs_ev_pos", "cs_ev_vel", "cs_ev_yaw", "cs_ev_hgt", "cs_opt_flow", "cs_rng_hgt", "cs_baro_hgt",
            "cs_inertial_dead_reckoning"]
REJ_FLAGS = ["reject_hor_vel", "reject_ver_vel", "reject_hor_pos", "reject_ver_pos", "reject_yaw",
             "reject_optflow_x", "reject_optflow_y"]


def gaps(t, thr):
    d = np.diff(t)
    return [(t[i], t[i + 1]) for i in np.where(d > thr)[0]]


def load(bag, ts):
    db = px4_bag.db3_of(bag)
    import sqlite3
    t0 = sqlite3.connect(str(db)).execute("select min(timestamp) from messages").fetchone()[0]
    rel = lambda rows: np.array([(t - t0) * 1e-9 for t, _ in rows])
    D = {}
    for topic in GAP:
        rows = px4_bag.read_topic(bag, topic, ts)
        D[topic] = (rel(rows), [m for _, m in rows])
    return D


def odom_series(t, msgs):
    v = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for m in msgs])
    p = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in msgs])
    P = np.array([np.array(m.pose.covariance).reshape(6, 6) for m in msgs])
    T = np.array([np.array(m.twist.covariance).reshape(6, 6) for m in msgs])
    sp = np.sqrt(np.clip(np.einsum("nii->ni", P[:, :3, :3]), 0, None))
    so = np.degrees(np.sqrt(np.clip(np.einsum("nii->ni", P[:, 3:, 3:]), 0, None)))
    sv = np.sqrt(np.clip(np.einsum("nii->ni", T[:, :3, :3]), 0, None))
    stamp = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs])
    return dict(t=t, stamp=stamp, p=p, v=np.linalg.norm(v, axis=1), sp=np.linalg.norm(sp, axis=1),
                so=so.max(1), sv=np.linalg.norm(sv, axis=1))


def bridge_series(t, msgs):
    g = lambda f: np.array([list(getattr(m, f)) for m in msgs], dtype=float)
    return dict(t=t, p=g("position"), v=np.linalg.norm(g("velocity"), axis=1),
                sp=np.linalg.norm(np.sqrt(np.clip(g("position_variance"), 0, None)), axis=1),
                so=np.degrees(np.sqrt(np.clip(g("orientation_variance"), 0, None))).max(1),
                sv=np.linalg.norm(np.sqrt(np.clip(g("velocity_variance"), 0, None)), axis=1),
                reset=np.array([m.reset_counter for m in msgs]), quality=np.array([m.quality for m in msgs]),
                stamp=np.array([m.timestamp for m in msgs], dtype=float) * 1e-6)


def broken(t, y, thr=0.25):
    """Inserisce NaN nei buchi, cosi' le linee non li attraversano."""
    t = np.asarray(t, float); y = np.asarray(y, float)
    i = np.where(np.diff(t) > thr)[0] + 1
    return np.insert(t, i, np.nan), np.insert(y, i, np.nan)


def at(s, key, t):
    i = np.searchsorted(s["t"], t)
    i = min(max(i, 0), len(s["t"]) - 1)
    return s[key][i], s["t"][i]


def window_stats(s, a, b, key):
    m = (s["t"] >= a) & (s["t"] <= b)
    return (np.nanmedian(s[key][m]), np.nanmax(s[key][m])) if m.any() else (float("nan"), float("nan"))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    ts = px4_bag.typestore()
    lines = []
    P = lambda s="": (lines.append(s), print(s))
    for test, rec in TESTS.items():
        bag = EVAL / "rosbags" / f"batch3_{test}" / rec
        D = load(bag, ts)
        od = odom_series(*D["/odomimu"])
        br = bridge_series(*D["/fmu/in/vehicle_visual_odometry"])
        lp_t, lp = D["/fmu/out/vehicle_local_position_v1"]
        ekf = dict(t=lp_t, v=np.array([math.hypot(math.hypot(m.vx, m.vy), m.vz) for m in lp]),
                   eph=np.array([m.eph for m in lp]), evh=np.array([m.evh for m in lp]),
                   xy_valid=np.array([m.xy_valid for m in lp]), v_xy_valid=np.array([m.v_xy_valid for m in lp]),
                   dr=np.array([m.dead_reckoning for m in lp]), xy_reset=np.array([m.xy_reset_counter for m in lp]),
                   vxy_reset=np.array([m.vxy_reset_counter for m in lp]), p=np.array([[m.x, m.y, m.z] for m in lp]))
        fl_t, fl = D["/fmu/out/estimator_status_flags"]
        flags = {k: np.array([getattr(m, k) for m in fl]) for k in EV_FLAGS + REJ_FLAGS}

        P(f"===================== {test} ({rec}), durata {max(v[0][-1] for v in D.values() if len(v[0])):.1f} s")
        P("Buchi (> soglia) per topic [s dall'inizio del bag]:")
        for topic, (t, _) in D.items():
            g = gaps(t, GAP[topic])
            P(f"  {topic:38s} {len(t):6d} msg  " + (", ".join(f"{a:.1f}->{b:.1f} ({b - a:.1f})" for a, b in g) or "nessuno"))
        P(f"Bridge: {len(br['t'])} msg contro {len(od['t'])} /odomimu; reset_counter unici {sorted(set(br['reset']))}, "
          f"quality unici {sorted(set(br['quality']))}")
        P("EKF2, flag (valori assunti nel bag): " + ", ".join(f"{k}={sorted(set(flags[k].tolist()))}" for k in EV_FLAGS))
        P("EKF2, rigetti: " + ", ".join(f"{k}={sorted(set(flags[k].tolist()))}" for k in REJ_FLAGS))
        trans = []
        for k in EV_FLAGS + REJ_FLAGS:
            f = flags[k]
            for i in np.where(np.diff(f.astype(int)) != 0)[0]:
                trans.append((fl_t[i + 1], k, bool(f[i + 1])))
        P("EKF2, cambi di flag: " + (", ".join(f"{t:.1f}s {k}->{v}" for t, k, v in sorted(trans)) or "nessuno"))
        P(f"EKF2 local_position: xy_valid {sorted(set(ekf['xy_valid'].tolist()))}, v_xy_valid {sorted(set(ekf['v_xy_valid'].tolist()))}, "
          f"dead_reckoning {sorted(set(ekf['dr'].tolist()))}, xy_reset_counter {ekf['xy_reset'][0]}->{ekf['xy_reset'][-1]}, "
          f"vxy_reset_counter {ekf['vxy_reset'][0]}->{ekf['vxy_reset'][-1]}")
        P(f"  |v| EKF2 mediana {np.median(ekf['v']):.2f} m/s, max {ekf['v'].max():.2f} m/s;  |v| /odomimu mediana {np.median(od['v']):.2f}, max {od['v'].max():.1f} m/s")

        g_od = gaps(od["t"], GAP["/odomimu"])
        for a, b in g_od:
            P(f"--- interruzione /odomimu {a:.1f} -> {b:.1f} s ({b - a:.1f} s)")
            n_br = ((br["t"] > a + 1e-3) & (br["t"] < b - 1e-3)).sum()
            imu_gap = any(x <= a + 0.3 and y >= b - 0.3 for x, y in gaps(D["/oak/imu/data"][0], 0.25))
            P(f"    IMU OAK registrata nel buco: {'no' if imu_gap else 'si'};  messaggi del bridge nel buco: {n_br}")
            for lab, t1, t2 in (("prima  [-2, 0] s", a - 2, a), ("dopo   [0, 2] s", b, b + 2), ("dopo   [2, 10] s", b + 2, b + 10),
                                ("dopo   [10, 30] s", b + 10, b + 30)):
                vm, vM = window_stats(od, t1, t2, "v")
                sp, _ = window_stats(od, t1, t2, "sp")
                so, _ = window_stats(od, t1, t2, "so")
                sv, _ = window_stats(od, t1, t2, "sv")
                bsp, _ = window_stats(br, t1, t2, "sp")
                bsv, _ = window_stats(br, t1, t2, "sv")
                ev, eM = window_stats(ekf, t1, t2, "v")
                eph, _ = window_stats(ekf, t1, t2, "eph")
                evh, _ = window_stats(ekf, t1, t2, "evh")
                P(f"    {lab}: OV |v| {vm:6.2f} (max {vM:6.1f}) m/s, sigma p {sp * 100:6.1f} cm, sigma ori {so:5.2f} deg, sigma v {sv:6.3f} m/s"
                  f" | bridge sigma p {bsp * 100:6.1f} cm, sigma v {bsv:6.3f} | EKF2 |v| {ev:5.2f} (max {eM:5.2f}), eph {eph:5.2f} m, evh {evh:5.2f} m/s")
            if len(od["t"][od["t"] > b]):
                s0, _ = at(od, "stamp", b + 1e-3)
                b0, _ = at(br, "stamp", b + 1e-3)
                P(f"    alla ripresa: header.stamp di /odomimu avanza di {s0 - at(od, 'stamp', a - 1e-3)[0]:.2f} s, quello del bridge di "
                  f"{b0 - at(br, 'stamp', a - 1e-3)[0]:.2f} s (nessun messaggio arretrato inviato dopo il buco se ~ durata del buco)")
        div = od["t"][od["v"] > 2.0]
        P(f"Divergenza OV online (|v| > 2 m/s): " + (f"dal {div[0]:.1f} s ({len(div)} msg)" if len(div) else "mai"))
        if len(div):
            m = od["v"] > 2.0
            nees_like = (od["v"][m] / np.maximum(od["sv"][m], 1e-6))
            P(f"  durante la divergenza: sigma v mediana {np.median(od['sv'][m]):.3f} m/s, |v|/sigma_v mediano {np.median(nees_like):.0f}, "
              f"sigma p mediana {np.median(od['sp'][m]) * 100:.1f} cm")
        P()

        # ---------------- grafico
        fig, ax = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
        for a, b in gaps(D["/oak/imu/data"][0], 0.25):
            for x in ax:
                x.axvspan(a, b, color="#d9d9d9", zorder=0)
        for a, b in g_od:
            for x in ax:
                x.axvspan(a, b, color="#f3c9c4", alpha=0.6, zorder=0)
        ax[0].semilogy(*broken(od["t"], np.maximum(od["v"], 1e-3)), color="#2a78d6", lw=1, label="OpenVINS online |v| (/odomimu)")
        ax[0].semilogy(*broken(ekf["t"], np.maximum(ekf["v"], 1e-3)), color="#0b0b0b", lw=1, label="EKF2 |v| (vehicle_local_position)")
        ax[0].axhline(2.0, color="#b3261e", ls=":", lw=1, label="2 m/s (soglia divergenza)")
        ax[0].set_ylabel("|v| [m/s]")
        ax[1].semilogy(*broken(od["t"], od["sp"] * 100), color="#2a78d6", lw=1, label="OpenVINS sigma posizione")
        ax[1].semilogy(*broken(br["t"], br["sp"] * 100), color="#e8a33d", lw=1, ls="--", label="bridge -> PX4 sqrt(position_variance)")
        ax[1].semilogy(*broken(ekf["t"], ekf["eph"] * 100), color="#0b0b0b", lw=1, label="EKF2 eph")
        ax[1].set_ylabel("deviazione std [cm]")
        ax[2].semilogy(*broken(od["t"], np.maximum(od["sv"], 1e-4)), color="#2a78d6", lw=1, label="OpenVINS sigma velocita'")
        ax[2].semilogy(*broken(br["t"], np.maximum(br["sv"], 1e-4)), color="#e8a33d", lw=1, ls="--", label="bridge -> PX4 sqrt(velocity_variance)")
        ax[2].semilogy(*broken(ekf["t"], np.maximum(ekf["evh"], 1e-4)), color="#0b0b0b", lw=1, label="EKF2 evh")
        ax[2].set_ylabel("deviazione std [m/s]")
        show = ["cs_ev_pos", "cs_ev_vel", "cs_ev_yaw", "cs_opt_flow", "cs_rng_hgt", "cs_baro_hgt", "cs_inertial_dead_reckoning",
                "reject_hor_vel", "reject_hor_pos", "reject_optflow_x"]
        for i, k in enumerate(show):
            f = flags[k].astype(bool)
            for j in np.where(f)[0]:
                t_end = fl_t[j + 1] if j + 1 < len(fl_t) else fl_t[j] + 1.0
                ax[3].broken_barh([(fl_t[j], t_end - fl_t[j])], (i + 0.15, 0.7), color="#2a78d6" if k.startswith("cs_") else "#b3261e")
        ax[3].set_yticks(np.arange(len(show)) + 0.5)
        ax[3].set_yticklabels(show, fontsize=8)
        ax[3].set_ylim(0, len(show))
        ax[3].set_title("EKF2 estimator_status_flags: barra = flag attivo (blu: fusione, rosso: rigetto)", fontsize=9)
        ax[3].set_xlabel("tempo dall'inizio del bag [s]  (grigio: buco di registrazione; rosa: /odomimu fermo)")
        for x in ax[:3]:
            x.legend(loc="upper left", fontsize=8)
            x.grid(True, color="#e4e3df", lw=0.6)
        fig.suptitle(f"batch3_{test}: ripresa degli stream - OpenVINS online, bridge, EKF2", fontsize=12)
        fig.tight_layout()
        fig.savefig(OUT / f"{test}_resume.png", dpi=120)
        plt.close(fig)
    (OUT / "report.txt").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
