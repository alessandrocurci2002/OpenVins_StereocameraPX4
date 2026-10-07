#!/usr/bin/env python3
"""
Converte un file di "total state" di OpenVINS (quello prodotto da
save_total_state:=true, header "timestamp(s) q p v bg ba cam_imu_dt num_cam ...")
nel formato di traiettoria richiesto da ov_eval:
    # timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33

Il total state file NON e' utilizzabile direttamente da error_singlerun /
error_dataset / error_comparison / plot_trajectories: il loro Loader legge le
colonne 1-7 come "tx ty tz qx qy qz qw", mentre nel total state file le stesse
colonne sono in realta' "qx qy qz qw px py pz" (quaternione poi posizione).
Le colonne 8+ (bias, cam_imu_dt, num_cam, calibrazione...) verrebbero pure
interpretate erroneamente come covarianza. Il risultato e' un parsing che non
fallisce ma produce numeri senza senso.

I valori di posizione/quaternione nel total state file sono pero' esattamente
gli stessi che sarebbero stati pubblicati in tempo reale su /ov_msckf/poseimu
(stessa chiamata state->_imu->quat()/pos(), nessuna conversione applicata),
quindi basta riordinare le colonne: nessun bisogno di rilanciare OpenVINS.

Covarianza (opzionale, per il NEES)
-----------------------------------
Il file "*_std.txt" gemello (stesso numero di righe, scritto nella stessa
iterazione del filtro) contiene le deviazioni standard di orientazione e
posizione: colonne 1-3 = std orientazione (error-state, 3 valori), colonne
4-6 = std posizione. Sono SOLO le diagonali (nessun termine incrociato tra
gli assi, ne' tra posizione e orientazione) ma e' la stessa approssimazione
diagonale che OpenVINS usa gia' per i suoi bound a 3-sigma. Se passato
--std, il file di output diventera' a 20 colonne (Pr../Pt.. = varianze sulla
diagonale, zero altrove), abilitando il calcolo del NEES in error_singlerun/
error_dataset. La ground truth puo' restare senza covarianza: viene trattata
come "certa" (zero) da ov_eval, comportamento corretto per una GT da TagSLAM.

Per una covarianza posizione/orientazione completa (coi termini incrociati)
servirebbe invece registrare dal vivo /ov_msckf/poseimu (PoseWithCovarianceStamped),
che pubblica la matrice 6x6 completa: non necessario per un consistency-check
standard, ma disponibile se serve maggiore precisione.

Frame di output (--frame)
------------------------
Di default la posa esportata e' quella dell'IMU (p_IinG, q_GtoI). La GT di
TagSLAM (/tagslam/odom/body_rig) e' invece la posa del rig, che coincide con
cam0: con --frame cam0 la posa viene portata su cam0 usando gli estrinseci
camera-IMU scritti nella stessa riga del total state (q_ItoC0, p_IinC0, quindi
anche quelli stimati online se calib_cam_extrinsics e' attivo):
    R_GtoC = R_ItoC * R_GtoI,   p_CinG = p_IinG - R_GtoI^T * R_ItoC^T * p_IinC
Le covarianze (--std) restano quelle dell'IMU: il braccio IMU-cam0 (~7 cm)
aggiungerebbe un termine dovuto all'orientazione, qui trascurato.

Uso:
    python3 state_to_pose.py ov_estimate.txt --out algorithms/config_v1/test_3/run_1.txt
    python3 state_to_pose.py ov_estimate.txt --std ov_estimate_std.txt --out algorithms/config_v1/test_3/run_1.txt
    python3 state_to_pose.py ov_estimate.txt --frame cam0 --out algorithms/config_v1/test_3/run_1.txt
"""

import argparse
import math
from pathlib import Path

# indici nel total state: t(0) q(1-4) p(5-7) v bg ba(8-16) cam_imu_dt(17) num_cam(18),
# poi per ogni camera 8 intrinseci, q_ItoC (4), p_IinC (3)
CAM0_Q = slice(27, 31)
CAM0_P = slice(31, 34)


def _jpl_to_rot(q):
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)],
            [2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
            [2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)]]


def _rot_to_jpl(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        w = 0.5 * math.sqrt(1 + tr)
        x, y, z = (R[1][2] - R[2][1]) / (4 * w), (R[2][0] - R[0][2]) / (4 * w), (R[0][1] - R[1][0]) / (4 * w)
    elif R[0][0] >= R[1][1] and R[0][0] >= R[2][2]:
        x = 0.5 * math.sqrt(1 + R[0][0] - R[1][1] - R[2][2])
        w, y, z = (R[1][2] - R[2][1]) / (4 * x), (R[0][1] + R[1][0]) / (4 * x), (R[0][2] + R[2][0]) / (4 * x)
    elif R[1][1] >= R[2][2]:
        y = 0.5 * math.sqrt(1 - R[0][0] + R[1][1] - R[2][2])
        w, x, z = (R[2][0] - R[0][2]) / (4 * y), (R[0][1] + R[1][0]) / (4 * y), (R[1][2] + R[2][1]) / (4 * y)
    else:
        z = 0.5 * math.sqrt(1 - R[0][0] - R[1][1] + R[2][2])
        w, x, y = (R[0][1] - R[1][0]) / (4 * z), (R[0][2] + R[2][0]) / (4 * z), (R[1][2] + R[2][1]) / (4 * z)
    if w < 0:
        x, y, z, w = -x, -y, -z, -w
    return [x, y, z, w]


def _imu_to_cam0(vals):
    R_GtoI = _jpl_to_rot(vals[1:5])
    R_ItoC = _jpl_to_rot(vals[CAM0_Q])
    p_IinG = vals[5:8]
    p_IinC = vals[CAM0_P]
    R_GtoC = [[sum(R_ItoC[i][k] * R_GtoI[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    # p_CinG = p_IinG - R_GtoC^T * p_IinC
    p_CinG = [p_IinG[j] - sum(R_GtoC[i][j] * p_IinC[i] for i in range(3)) for j in range(3)]
    return p_CinG, _rot_to_jpl(R_GtoC)


def _load_rows(path):
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            vals = line.split()
            if len(vals) < 8:
                continue
            rows.append([float(v) for v in vals])
    return rows


def convert(state_path: str, out_path: str, std_path: str = None, frame: str = "imu"):
    state_rows = _load_rows(state_path)

    std_rows = None
    if std_path:
        std_rows = _load_rows(std_path)
        if len(std_rows) != len(state_rows):
            raise ValueError(
                f"'{state_path}' ha {len(state_rows)} righe ma '{std_path}' ne ha {len(std_rows)}: "
                "devono essere stati scritti insieme dallo stesso run, non posso associarli per indice."
            )

    out_rows = []
    for i, vals in enumerate(state_rows):
        t = vals[0]
        if frame == "cam0":
            (px, py, pz), (qx, qy, qz, qw) = _imu_to_cam0(vals)
        else:
            qx, qy, qz, qw = vals[1:5]
            px, py, pz = vals[5:8]
        row = [t, px, py, pz, qx, qy, qz, qw]
        if std_rows is not None:
            std_ori = std_rows[i][1:4]
            std_pos = std_rows[i][4:7]
            var_ori = [v * v for v in std_ori]
            var_pos = [v * v for v in std_pos]
            # Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 (solo diagonale: incrociati = 0)
            row += [var_ori[0], 0.0, 0.0, var_ori[1], 0.0, var_ori[2]]
            # Pt11 Pt12 Pt13 Pt22 Pt23 Pt33
            row += [var_pos[0], 0.0, 0.0, var_pos[1], 0.0, var_pos[2]]
        out_rows.append(row)

    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        if std_rows is not None:
            f.write("# timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33\n")
            fmt = "{:.5f}" + " {:.6f}" * 7 + " {:.10f}" * 12 + "\n"
        else:
            f.write("# timestamp(s) tx ty tz qx qy qz qw\n")
            fmt = "{:.5f}" + " {:.6f}" * 7 + "\n"
        for r in out_rows:
            f.write(fmt.format(*r))

    print(f"[OK] {len(out_rows)}/{len(state_rows)} pose convertite in: {out_path}")
    print(f"     covarianza: {'si (diagonale, da ' + std_path + ')' if std_rows is not None else 'no'}")
    print(f"     frame:      {frame}")
    if out_rows:
        print(f"     range temporale: {out_rows[0][0]:.3f} -> {out_rows[-1][0]:.3f} s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("state_file", help="file total-state di OpenVINS (es. ov_estimate.txt)")
    parser.add_argument("--std", default=None, help="file *_std.txt gemello (es. ov_estimate_std.txt), opzionale, per il NEES")
    parser.add_argument("--out", required=True, help="file di output .txt in formato ov_eval")
    parser.add_argument("--frame", choices=["imu", "cam0"], default="imu",
                        help="posa esportata: imu (default) o cam0 (come la GT di TagSLAM body_rig)")
    args = parser.parse_args()
    convert(args.state_file, args.out, args.std, args.frame)
