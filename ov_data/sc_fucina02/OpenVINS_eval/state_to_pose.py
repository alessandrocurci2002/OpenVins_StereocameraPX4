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

Uso:
    python3 state_to_pose.py ov_estimate.txt --out algorithms/config_v1/test_3/run_1.txt
    python3 state_to_pose.py ov_estimate.txt --std ov_estimate_std.txt --out algorithms/config_v1/test_3/run_1.txt
"""

import argparse
from pathlib import Path


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


def convert(state_path: str, out_path: str, std_path: str = None):
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
    if out_rows:
        print(f"     range temporale: {out_rows[0][0]:.3f} -> {out_rows[-1][0]:.3f} s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("state_file", help="file total-state di OpenVINS (es. ov_estimate.txt)")
    parser.add_argument("--std", default=None, help="file *_std.txt gemello (es. ov_estimate_std.txt), opzionale, per il NEES")
    parser.add_argument("--out", required=True, help="file di output .txt in formato ov_eval")
    args = parser.parse_args()
    convert(args.state_file, args.out, args.std)
