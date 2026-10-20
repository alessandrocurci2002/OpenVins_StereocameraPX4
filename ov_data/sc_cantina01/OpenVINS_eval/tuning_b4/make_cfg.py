#!/usr/bin/env python3
"""
Crea tuning_b4/runs/<NOME>/ copiando la config_v9 (rosbags/cantina_03/Openvins_config_v9) e
sovrascrivendo i parametri dati come chiave=valore. Le chiavi del rumore IMU
(accelerometer_noise_density, accelerometer_random_walk, gyroscope_noise_density,
gyroscope_random_walk) vanno in kalibr_imu_chain.yaml, tutte le altre in estimator_config.yaml.

Uso: make_cfg.py NOME chiave=valore [chiave=valore ...]
"""
import re
import shutil
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE.parent / "rosbags/cantina_03/Openvins_config_v9"
IMU_KEYS = {"accelerometer_noise_density", "accelerometer_random_walk", "gyroscope_noise_density", "gyroscope_random_walk"}


def set_key(path, key, value, indent):
    txt = path.read_text()
    pat = re.compile(rf"^({indent}{key}:)[^\n#]*", re.M)
    if pat.search(txt):
        txt = pat.sub(rf"\g<1> {value} ", txt, count=1)
    else:
        txt = txt.rstrip("\n") + f"\n{indent}{key}: {value}\n"
    path.write_text(txt)


def main():
    name, overrides = sys.argv[1], sys.argv[2:]
    run = HERE / "runs" / name
    run.mkdir(parents=True, exist_ok=True)
    for f in ("estimator_config.yaml", "kalibr_imu_chain.yaml", "kalibr_imucam_chain.yaml"):
        shutil.copy(BASE / f, run / f)
    for kv in overrides:
        k, v = kv.split("=", 1)
        if k in IMU_KEYS:
            set_key(run / "kalibr_imu_chain.yaml", k, v, "  ")
        else:
            set_key(run / "estimator_config.yaml", k, v, "")
    (run / "overrides.txt").write_text(" ".join(overrides) + "\n")


if __name__ == "__main__":
    main()
