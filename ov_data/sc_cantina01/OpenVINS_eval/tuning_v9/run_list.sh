#!/bin/bash
# Lancia le run elencate in un file (righe: NOME ACC_ND ACC_RW GYR_ND GYR_RW [BAG]),
# PARALLEL alla volta, ciascuna su un ROS_DOMAIN_ID diverso. Da lanciare DENTRO il container.
#
# Uso: run_list.sh lista.txt [PARALLEL]
LIST=$1; PARALLEL=${2:-4}
DIR=$(dirname "$(readlink -f "$0")")
slot=0
while read -r NAME AN AR GN GR BAG; do
  [[ -z "$NAME" || "$NAME" == \#* ]] && continue
  "$DIR"/run_one.sh "$NAME" "$AN" "$AR" "$GN" "$GR" $((40 + slot)) $BAG < /dev/null &
  slot=$((slot + 1))
  if (( slot == PARALLEL )); then wait; slot=0; fi
done < "$LIST"
wait
echo "[run_list] finito: $LIST"
