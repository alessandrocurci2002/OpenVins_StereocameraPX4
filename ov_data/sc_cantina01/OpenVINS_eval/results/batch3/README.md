# Batch 3 (cantina, 07/10/2026): TagSLAM vs OpenVINS vs EKF2

Tre registrazioni a mano con il drone disarmato (`vio_recorder` sul RPi5), bag in
`TagSLAM/kalibr-TagSLAM/data/tagslam_cantina_batch03/` (hard link in `rosbags/batch3_<test>/rec_*`).

| Test | Bag | Durata | Contenuto (dalla GT) |
| --- | --- | --- | --- |
| `2loops` | `rec_20261007_152554` | 146 s | 2 giri della cantina, 29.5 m; arrivo a 3.3 cm dalla partenza |
| `hover` | `rec_20261007_153315` | 48 s | sollevamento di 13 cm (9-12 s), poi fermo 36 s |
| `6dof` | `rec_20261007_153538` | 114 s | rotazioni su tutti gli assi (roll fino a 84 deg), 10.9 m |

## Buchi nella registrazione

Il registratore ha perso **tutti** i topic (immagini, IMU OAK, PX4) per 7.3 s nel `2loops`
(90.7 -> 98.0 s) e 18.5 s nel `6dof` (84.3 -> 102.8 s, piu' 0.4 s a 104.3 s). Senza IMU ne'
immagini OpenVINS offline diverge dopo il buco (velocita' > 14 m/s, posizione a km). Anche
`/odomimu` (OpenVINS online) si ferma prima: 52.0 -> 53.2 s e 84.6 -> 98.0 s nel `2loops`,
81.9 -> 102.8 s nel `6dof`; nel `2loops` l'online diverge dopo il primo stallo (|v| > 2 m/s da 55.5 s).

## Dataset di ov_eval (finestre continue, tempi dall'inizio del bag)

| Dataset | Finestra | Algoritmi |
| --- | --- | --- |
| `batch3_hover` | 0 - 48.2 s | config_v9, openvins_rpi_online, ekf2 |
| `batch3_6dof_seg1` | 0 - 84.2 s | config_v9, openvins_rpi_online, ekf2 |
| `batch3_2loops_seg0` | 0 - 52.0 s | config_v9, openvins_rpi_online, ekf2 |
| `batch3_2loops_seg1` | 0 - 90.6 s | config_v9, ekf2 |
| `batch3_2loops_full` | 0 - 146.3 s | ekf2 |
| `batch3_6dof_full` | 0 - 114.2 s | ekf2 |

- **GT** (`truths/`): TagSLAM `/tagslam/odom/body_rig` (posa di cam0), config `tagslam.yaml` del batch03
  senza modifiche; converge su tutti e tre i bag (`rosbags/batch3_<test>/tagslam/convergence_check.txt`).
  Il mondo del tag 0 e' ruotato per avere z verticale (verticale = media di 6 stime da OpenVINS ed EKF2,
  dispersione max 1.95 deg, `gt_leveling.json`): rotazione rigida, ATE/RPE con allineamento se3 invariati.
- **config_v9**: OpenVINS offline con `rosbags/cantina_03/Openvins_config_v9` (`run_batch3.sh`), posa di cam0
  con covarianza diagonale (`state_to_pose.py --frame cam0 --std`).
- **openvins_rpi_online**: `/odomimu` registrato, cioe' quello che il bridge manda a PX4 (`odomimu_to_pose.py`).
  Sul RPi gira `config/oakdpro_calib05_newImucalib` (Makefile del branch companion), non la config_v9.
- **ekf2**: `/fmu/out/vehicle_odometry` (`ekf2_to_pose.py`), EKF2 con flusso ottico + rangefinder + baro + mag.
  Portato su cam0 con R_B_I stimata dai giroscopi (`calib_px4_oak.py` -> `px4_oak_calib.yaml`:
  [82.41, -0.37, 90.45] deg, 0.8 deg dalla rotazione del bridge; offset PX4-OAK -3 ms). Braccio di leva
  Pixhawk -> cam0 non misurato (0): passarlo con `--lever-arm` quando e' noto.

## Allineamenti

- `se3ori` (principale): rotazione = media cordale delle R_gt R_est^T, traslazione dai baricentri. Aggiunto in
  `ov_eval/src/alignment/AlignTrajectory.cpp`.
- `se3` (Umeyama sulle sole posizioni, come dataset2-5): degenere in `hover` (traiettoria quasi verticale)
  e falsato dalle posizioni di EKF2 nel `6dof`: errori di orientazione fittizi di 14-53 deg.

Sotto-segmenti RPE: 10-50 % del percorso della GT di ogni dataset (`OVEVAL_SEGMENTS`, aggiunto in ov_eval).

## File

- `summary.csv` ATE / NEES per dataset, algoritmo, allineamento; `rpe.csv` mediane RPE per sotto-segmento.
- `datasets.json` finestre e metriche extra (chiusura dei giri, stabilita' in hover, scala sim3 di EKF2).
- `<dataset>/<align>/<algo>/error_singlerun_*.png`, `<dataset>/<align>/trajectories/`, `<dataset>/<align>/comparison/`
  e i log testuali di ov_eval accanto.
- Rigenerare tutto: `python3 evaluate_batch3.py` nel container `provola1`.
