#!/bin/bash
# Lancia le run elencate (righe: NOME BAG_relativo_a_rosbags [RATE]), PARALLEL alla volta. DENTRO il container.
# Le cartelle di config devono gia' esistere (make_cfg.py).  Uso: run_list.sh lista.txt [PARALLEL]
LIST=$1; PARALLEL=${2:-2}; DIR=$(dirname "$(readlink -f "$0")"); slot=0
while read -r NAME BAG RATE; do
  [[ -z "$NAME" || "$NAME" == \#* ]] && continue
  "$DIR"/run_cfg.sh "$NAME" $((70 + slot)) "$BAG" ${RATE:-1.0} < /dev/null &
  slot=$((slot + 1)); if (( slot == PARALLEL )); then wait; slot=0; fi
done < "$LIST"
wait; echo "[run_list] finito: $LIST"
