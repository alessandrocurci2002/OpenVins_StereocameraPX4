#!/usr/bin/env python3
"""
Rumore dell'IMU dell'OAK-D Pro (/oak/imu/data) misurato nelle condizioni operative, per scegliere
noise_density e random_walk di OpenVINS (kalibr_imu_chain.yaml).

Metodo (IEEE Std 952-1997 / 1554-2005, Allan variance; El-Sheimy et al. 2008):
  - da fermo e disarmato: Allan deviation del segnale grezzo dell'OAK (moto nullo);
  - armato / in volo / a mano con motori accesi: il moto vero non e' nullo e va tolto. Si usa
    come riferimento l'IMU del Pixhawk (/fmu/out/sensor_combined, smorzata, medie su 5 ms a 100 Hz,
    tempi gia' nel dominio ROS), portata nel frame dell'OAK:
        w_ref = R_IB w_B
        a_ref = R_IB (a_B + alpha x r + w x (w x r))        r = EKF2_EV_POS (braccio di leva)
    R_IB e lo sfasamento temporale sono stimati sullo stesso segmento (Wahba sui giroscopi filtrati
    a 5 Hz, correlazione di |w|). Il residuo OAK(media 10 ms) - riferimento contiene il rumore
    dell'OAK piu' quello (piccolo) del Pixhawk e gli errori di scala/allineamento: e' una stima
    per eccesso, cioe' conservativa.
  - dal residuo: Allan deviation sigma(tau), densita' equivalente N_eff(tau) = sigma(tau) sqrt(tau)
    (per rumore bianco e' costante e uguale alla noise density), fit del modello
    sigma^2 = N^2/tau + 0.4339 B^2 + K^2 tau/3 (N white, B bias instability, K random walk),
    densita' spettrale nella banda 0.5-15 Hz (sqrt(PSD_monolatera/2)), deriva media del residuo
    rispetto al segmento da fermo (spostamento del bias da vibrazione, VRE).

Uso (nel container ROS2, da OpenVINS_eval):
    python3 noise_study/imu_noise.py BAG NOME seg_nome:t0:t1 [seg_nome:t0:t1 ...]
  tempi in secondi dall'inizio del bag; un segmento chiamato "static*" usa il segnale grezzo.
Uscite in results/noise_study/<NOME>_*.{txt,npz}.
"""
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import nnls
from scipy.signal import welch
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import px4_bag  # noqa: E402
from calib_px4_oak import load_oak, load_px4  # noqa: E402

EVAL = Path(__file__).resolve().parent.parent
OUT = EVAL / "results" / "noise_study"
R_ARM = np.array([0.085, 0.0, 0.06])     # EKF2_EV_POS_X/Y/Z [m], body FRD (EKF2_IMU_POS = 0)
CALIB = yaml.safe_load(open(EVAL / "px4_oak_calib.yaml"))
TAUS = np.unique(np.round(np.logspace(np.log10(0.01), np.log10(30), 40), 3))


def adev(x, fs, taus):
    """Allan deviation sovrapposta (per colonna) di x campionato a fs."""
    th = np.vstack([np.zeros((1, x.shape[1])), np.cumsum(x, axis=0) / fs])
    out = []
    for tau in taus:
        m = int(round(tau * fs))
        if m < 1 or 2 * m >= len(th) - 1 or len(th) - 2 * m < 9 * m:   # almeno ~9 cluster indipendenti
            out.append(np.full(x.shape[1], np.nan))
            continue
        d = th[2 * m:] - 2 * th[m:-m] + th[:-2 * m]
        out.append(np.sqrt((d ** 2).mean(axis=0) / (2 * (m / fs) ** 2)))
    return np.array(out)


def fit_nbk(taus, s):
    """Fit NNLS di sigma^2(tau) = N^2/tau + 0.4339 B^2 + K^2 tau/3 (pesi relativi). Restituisce N, B, K."""
    ok = np.isfinite(s)
    t, v = taus[ok], s[ok] ** 2
    A = np.stack([1 / t, np.full_like(t, 0.4339), t / 3], axis=1) / v[:, None]
    c, _ = nnls(A, np.ones_like(v))
    return np.sqrt(c)


def win_avg(t, x, tq, w):
    """Media di x (campioni a t) sulle finestre [tq - w, tq] (integrale trapezoidale cumulato)."""
    I = np.vstack([np.zeros((1, x.shape[1])), np.cumsum(0.5 * (x[1:] + x[:-1]) * np.diff(t)[:, None], axis=0)])
    out = np.stack([(np.interp(tq, t, I[:, k]) - np.interp(tq - w, t, I[:, k])) / w for k in range(x.shape[1])], axis=1)
    return out


def interp3(tq, t, v):
    return np.stack([np.interp(tq, t, v[:, k]) for k in range(v.shape[1])], axis=1)


def lowpass(x, fs, fc):
    from scipy.signal import butter, filtfilt
    b, a = butter(2, fc / (fs / 2))
    return filtfilt(b, a, x, axis=0)


def analyse(bag, name, segs):
    to, wo, ao = load_oak(bag)
    tp, wp, tpa, ap = load_px4(bag)
    t_bag0 = min(to[0], tp[0])
    meta = yaml.safe_load(open(Path(bag) / "metadata.yaml"))["rosbag2_bagfile_information"]
    t_bag0 = meta["starting_time"]["nanoseconds_since_epoch"] * 1e-9
    lines, store = [f"bag {bag}", f"OAK {len(to)} campioni ({1 / np.median(np.diff(to)):.0f} Hz), PX4 {len(tp)} ({1 / np.median(np.diff(tp)):.0f} Hz)"], {}
    static_ref = None
    for seg in segs:
        sname, s0, s1 = seg.split(":")
        s0, s1 = t_bag0 + float(s0), t_bag0 + float(s1)
        res = {}
        if sname.startswith("static"):
            k = (to >= s0) & (to <= s1)
            fs = 1 / np.median(np.diff(to))
            xa, xg = ao[k] - ao[k].mean(0), wo[k] - wo[k].mean(0)
            info = "segnale grezzo OAK"
            mean_a, mean_g = None, None
        else:
            fs = 100.0
            grid = np.arange(s0 + 0.05, s1 - 0.05, 1 / fs)
            # sfasamento temporale (|w| filtrato 20 Hz) e rotazione (Wahba, giroscopi filtrati 5 Hz)
            kk = (to >= s0 - 1) & (to <= s1 + 1)
            g_o = np.arange(s0, s1, 0.001)
            no = np.interp(g_o, to, np.linalg.norm(lowpass(wo, 200, 20), axis=1))
            npn = np.linalg.norm(lowpass(wp, 100, 20), axis=1)
            lags = np.arange(-0.03, 0.03, 0.0005)
            cc = [np.corrcoef(no, np.interp(g_o + L, tp, npn))[0, 1] for L in lags]
            lag = lags[int(np.argmax(cc))]
            if max(cc) < 0.9:                              # poco moto: sfasamento non osservabile, si usa la calibrazione
                lag = -CALIB["time_offset_px4_minus_oak_s"]
            wB = interp3(grid + lag, tp, lowpass(wp, 100, 5))
            wI = interp3(grid, to, lowpass(wo, 200, 5))
            rot, _ = R.align_vectors(wB, wI)               # w_B = R_BI w_I (solo diagnostica)
            R_cal = np.array(CALIB["R_B_I"])
            ang = np.degrees(R.from_matrix(rot.as_matrix().T @ R_cal).magnitude())
            R_BI = R_cal                                   # rotazione fissa (batch3): le medie restano confrontabili
            # riferimento nel frame OAK
            w_B = interp3(grid + lag, tp, wp)
            a_B = interp3(grid + lag, tpa, ap)
            wf = lowpass(wp, 100, 20)
            alpha = interp3(grid + lag, tp[1:-1], (wf[2:] - wf[:-2]) / (tp[2:] - tp[:-2])[:, None])
            a_B = a_B + np.cross(alpha, R_ARM) + np.cross(w_B, np.cross(w_B, R_ARM))
            w_ref, a_ref = w_B @ R_BI, a_B @ R_BI           # R_IB v = (v^T R_BI)^T
            w_oak = win_avg(to[kk], wo[kk], grid, 0.010)
            a_oak = win_avg(to[kk], ao[kk], grid, 0.010)
            rg, ra = w_oak - w_ref, a_oak - a_ref
            mean_a, mean_g = ra.mean(0), rg.mean(0)
            from scipy.signal import coherence
            fc_, ca = coherence(ra, a_ref, fs=fs, nperseg=int(4 * fs), axis=0)
            _, cg = coherence(rg, w_ref, fs=fs, nperseg=int(4 * fs), axis=0)
            lo = (fc_ >= 0.5) & (fc_ <= 5)
            coh = (ca[lo].mean(0), cg[lo].mean(0))
            xa, xg = ra - mean_a, rg - mean_g
            info = (f"residuo OAK - Pixhawk: lag {lag * 1e3:+.1f} ms (corr {max(cc):.3f}), R_BI stimata a {ang:.2f} deg "
                    f"da px4_oak_calib.yaml; |w| medio {np.linalg.norm(w_B, axis=1).mean():.2f} rad/s; coerenza residuo-moto 0.5-5 Hz "
                    f"acc {' '.join(f'{v:.2f}' for v in coh[0])}, gyro {' '.join(f'{v:.2f}' for v in coh[1])}")
        ad_a, ad_g = adev(xa, fs, TAUS), adev(xg, fs, TAUS)
        fa, Pa = welch(xa, fs=fs, nperseg=min(len(xa), int(5 * fs)), axis=0)
        fg, Pg = welch(xg, fs=fs, nperseg=min(len(xg), int(5 * fs)), axis=0)
        band = (fa >= 0.5) & (fa <= 15)
        res.update(fs=fs, dur=(s1 - s0))
        lines.append(f"\n== {sname}  [{s0 - t_bag0:.1f}, {s1 - t_bag0:.1f}] s ({s1 - s0:.1f} s), {info}")
        for lab, x, ad, P, f in (("acc [m/s^2]", xa, ad_a, Pa, fa), ("gyro [rad/s]", xg, ad_g, Pg, fg)):
            key = "acc" if lab.startswith("acc") else "gyro"
            sig = x.std(0)
            psd_n = np.sqrt(P[band].mean(0) / 2)
            def at(tau):
                i = int(np.argmin(abs(TAUS - tau)))
                return ad[i] * np.sqrt(TAUS[i])
            nb = np.array([fit_nbk(TAUS, ad[:, k]) for k in range(3)])
            lines.append(f"  {lab}")
            lines.append(f"    sigma per campione ({fs:.0f} Hz)      " + "  ".join(f"{v:.4g}" for v in sig) +
                         f"   -> sigma*sqrt(dt) = {np.linalg.norm(sig) / np.sqrt(3) / np.sqrt(fs):.4g}")
            for tau in (0.033, 0.1, 0.33, 1.0):
                lines.append(f"    N_eff(tau={tau:5.3f} s) = sigma_A*sqrt(tau)  " + "  ".join(f"{v:.4g}" for v in at(tau)) +
                             f"   rms {np.sqrt((at(tau) ** 2).mean()):.4g}")
            lines.append(f"    densita' PSD 0.5-15 Hz             " + "  ".join(f"{v:.4g}" for v in psd_n) +
                         f"   rms {np.sqrt((psd_n ** 2).mean()):.4g}")
            lines.append(f"    fit Allan N                        " + "  ".join(f"{v:.4g}" for v in nb[:, 0]))
            lines.append(f"    fit Allan B (instab. bias)         " + "  ".join(f"{v:.4g}" for v in nb[:, 1]))
            lines.append(f"    fit Allan K (random walk)          " + "  ".join(f"{v:.4g}" for v in nb[:, 2]) +
                         f"   (tau max {TAUS[np.isfinite(ad[:, 0])].max():.1f} s)")
            pk = f[np.argsort(P.mean(1))[-3:]][::-1]
            lines.append(f"    picchi spettro residuo [Hz]       {', '.join(f'{v:.1f}' for v in pk)}")
            res[key] = dict(adev=ad, sig=sig, psd_n=psd_n, fit=nb, f=f, P=P)
        if mean_a is not None:
            res["mean_acc"], res["mean_gyro"] = mean_a, mean_g
            if static_ref is not None:
                lines.append(f"  spostamento medio residuo vs {static_ref[0]}: acc " +
                             "  ".join(f"{v:+.3f}" for v in mean_a - static_ref[1]) + " m/s^2, gyro " +
                             "  ".join(f"{v:+.4f}" for v in mean_g - static_ref[2]) + " rad/s")
            else:
                static_ref = (sname, mean_a, mean_g)
        store[sname] = res
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}_noise.txt").write_text("\n".join(lines) + "\n")
    np.savez(OUT / f"{name}_noise.npz", taus=TAUS, **{f"{s}_{k}_{q}": v[k][q] for s, v in store.items() for k in ("acc", "gyro")
                                                      for q in ("adev", "f", "P")})
    print("\n".join(lines))


if __name__ == "__main__":
    analyse(sys.argv[1], sys.argv[2], sys.argv[3:])
