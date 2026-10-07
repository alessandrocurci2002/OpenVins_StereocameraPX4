# dataset_5_batch2 (cantina05, batch 2, 06/10/2026)

Ground truth: `/tagslam/odom/body_rig` di
`TagSLAM/kalibr-TagSLAM/data/tagslam_cantina_batch02/tagslam_cantina05_opt_result_v1/out.bag`,
convertito con `extract_gt.py` (12226 pose, 1791287758.943 -> 1791288166.493 s).
La posa e' quella del rig, cioe' di cam0: confrontare OpenVINS con `state_to_pose.py --frame cam0`.

Bag ROS2 per OpenVINS: `rosbags/cantina_05/tagslam_cantina05/`, convertito con `rosbags-convert`
dal bag ROS1 `tagslam_cantina05.bag` (solo `/oak/imu/data`, `/oak/left/image_rect`, `/oak/right/image_rect`, 407.6 s).

## Protocollo di registrazione

Tempi dall'inizio del bag, ricavati dal GT (camera ferma = velocita' < 2 cm/s per almeno 1 s).

| Fase | Intervallo | Note |
| --- | --- | --- |
| A - a riposo | 0.0 - 16.6 s | camera appoggiata, rivolta verso muro_1 (tag 0, 1, 2, 3) |
| Sollevamento | ~17 - 28 s | camera alzata di ~0.4 m per l'inizializzazione di OpenVINS |
| B - riappoggiata | 28.4 - 41.1 s | stesso punto di A (GT: 0.6 cm da A, incertezza ~1 cm) |
| Loop | ~41 - 387 s | giri attorno alla cantina, ~87 m di percorso (GT) |
| C - riposo finale | 386.8 - 407.5 s | stesso punto (GT: 0.2 cm da B) |

Il drift di OpenVINS si misura tra B e C, cosi' l'inizializzazione (fase A + sollevamento) resta fuori.

## Spostamenti degli AprilTag rispetto al batch 1

Il piano era in `tagslam_cantina01/tagslam_cantina03_tag_moves/` (10 tag "superflui" da spostare nelle
posizioni N1-N10). Gli spostamenti qui sotto sono ricostruiti confrontando le mappe TagSLAM
`tagslam_cantina01/tagslam_cantina04/poses.yaml` (batch 1) e `tagslam_cantina05_opt_result_v1/poses.yaml`
(batch 2). Entrambe hanno il mondo nel tag 0, che non si e' mosso (0.1 cm).
Assi: x a destra lungo muro_1, y in alto (pavimento a y ~ -1.25 m), z dentro la stanza.

Tag spostati (piu' di 0.6 m):

| Tag | Piano | Prima (x, y, z) [m] | Dopo (x, y, z) [m] | Spostamento |
| --- | --- | --- | --- | --- |
| 47 | N5 | -0.44, -1.26, 2.12 (pavimento) | -3.35, -0.52, 1.07 (rialzato, verso muro_2) | 3.18 m |
| 40 | N7 | 0.20, -1.27, 3.19 (pavimento) | -2.64, -1.22, 3.10 (pavimento) | 2.85 m |
| 32 | N8 | -2.51, -1.29, 2.46 (pavimento) | -3.93, 0.18, 3.78 (muro_2) | 2.44 m |
| 31 | - | -2.01, -1.29, 1.39 (pavimento) | -2.82, -1.25, 2.36 (pavimento) | 1.27 m |
| 37 | N4 | -0.32, -1.29, 6.34 (pavimento) | -1.15, -1.15, 7.26 (pavimento) | 1.25 m |
| 42 | - | -0.50, -1.26, 4.10 (pavimento) | -0.51, -1.19, 3.30 (pavimento) | 0.81 m |
| 49 | N1 | -1.06, -1.27, 1.35 (pavimento) | -1.39, -1.26, 0.63 (pavimento) | 0.80 m |
| 45 | N3 | -1.38, -1.28, 6.89 (pavimento) | -1.83, -1.13, 7.36 (pavimento) | 0.67 m |

- Aggiunto: tag 29 (non presente nel batch 1).
- Nel piano ma non spostati (< 6 cm): 26 (N6), 27 (N2), 41 (N10), 46 (N9).
- Differenze di 0.15 - 0.55 m (tag 4, 16-22, 25, 44, 48): soprattutto tag lontani dal tag 0, compatibili con
  la diversa ottimizzazione delle due mappe (nel batch 2 i vincoli di muro_2 e del pavimento sono
  disattivati). Da verificare a mano se qualcuno e' stato mosso davvero (tag 25: 0.55 m, tag 4: 0.40 m).

## Configurazione TagSLAM (batch 2)

- Dimensione dei tag 0.165 m (misurata).
- Vincoli di piano solo su muro_1 (tag 0, 1, 2, 3, 5, 6); muro_2 e pavimento commentati.
- `minimum_viewing_angle` 15 deg, `optimizer_mode: fast`, prior di moto nullo tra frame
  (`fake_odom_translation_noise` 0.015, `fake_odom_rotation_noise` 0.03).
