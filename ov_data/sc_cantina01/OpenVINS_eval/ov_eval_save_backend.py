"""
Backend matplotlib che, invece di aprire le finestre, salva in PNG ogni figura che
ov_eval (matplotlib-cpp) mostra con plt.show(). Utile per avere i grafici di
error_singlerun / plot_trajectories come file, anche senza display.

Prima di salvare rende ogni figura leggibile senza conoscere ov_eval:
  - legenda su ogni asse che ha curve con etichetta (ov_eval assegna le etichette ma non
    chiama mai legend() in error_singlerun, ne' nel grafico x-y di plot_trajectories);
  - nomi e colori fissi per algoritmo (OVEVAL_LABELS), uguali in tutte le figure;
  - titolo generale (OVEVAL_PLOT_TITLE) e nota a pie' di figura con frame, unita' e
    definizione della grandezza, per ogni tipo di grafico di ov_eval;
  - griglia leggera, "IMU" nei titoli sostituito dal corpo valutato (OVEVAL_BODY, es. cam0).
Senza le variabili d'ambiente opzionali il comportamento e' quello di prima (solo salvataggio
piu' legenda e griglia).

Uso (nel container):
    export PYTHONPATH=/root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval:$PYTHONPATH
    export MPLBACKEND=module://ov_eval_save_backend
    export OVEVAL_PLOT_DIR=results/plots_config_v9 OVEVAL_PLOT_PREFIX=error_singlerun
    # opzionali
    export OVEVAL_PLOT_TITLE="batch3_hover - OpenVINS offline vs TagSLAM"
    export OVEVAL_ALGO=config_v9 OVEVAL_BODY=cam0
    export OVEVAL_LABELS='{"config_v9": ["OpenVINS offline (config_v9)", "#2a78d6"]}'
    ros2 run ov_eval error_singlerun se3 truths/dataset3.txt algorithms/config_v9/dataset3/run_1.txt
"""

import json
import os
import re

import matplotlib.pyplot as plt
from matplotlib.backend_bases import FigureManagerBase, _Backend
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

_count = 0

INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"

# ov_eval colora le curve con nomi di colore fissi: li si riconosce per sostituirli
_OV_BLUE = (0.0, 0.0, 1.0, 1.0)
_OV_RED = (1.0, 0.0, 0.0, 1.0)

# Note a pie' di figura, per titolo del grafico di ov_eval (minuscolo)
_NOTES = {
    "relative orientation error":
        "RPE: errore della posa relativa (rotazione e/o traslazione) tra due pose distanti L metri di percorso della GT\n"
        "(asse x), su tutte le coppie. Box = 25-75 percentile, tacca e linea = mediana, baffi = 1.5 IQR (outlier non disegnati).",
    "relative position error":
        "RPE: errore della posa relativa (rotazione e/o traslazione) tra due pose distanti L metri di percorso della GT\n"
        "(asse x), su tutte le coppie. Box = 25-75 percentile, tacca e linea = mediana, baffi = 1.5 IQR (outlier non disegnati).",
    "normalized estimation error squared":
        "NEES = e' P^-1 e con la covarianza stimata (diagonale). Per uno stimatore consistente la media vale 3 (3 gradi di liberta'):\n"
        "valori molto piu' alti = covarianza ottimista, piu' bassi = pessimista.",
    "position error":
        "Errore = GT allineata - stima, nel frame mondo dello stimatore (z verticale verso l'alto, x-y orizzontali).\n"
        "Le bande +-3 sigma vengono dalla covarianza stimata dall'algoritmo (assenti se l'algoritmo non la fornisce).",
    "orientation error":
        "Errore di orientazione come vettore di rotazione (piccoli angoli) nel frame mondo dello stimatore.\n"
        "Le bande +-3 sigma vengono dalla covarianza stimata dall'algoritmo (assenti se l'algoritmo non la fornisce).",
    "global orientation rpy error":
        "Differenza degli angoli ZYX (roll, pitch, yaw) dell'orientazione del corpo valutato nel mondo dello stimatore.\n"
        "Per cam0 (asse ottico z avanti, y in basso) non coincidono con roll/pitch/yaw del drone; singolari per pitch vicino a +-90 deg.",
    "trajectory_xy":
        "Traiettorie allineate alla GT ({align}), viste dall'alto: frame TagSLAM livellato (x lungo muro_1, y verso il muro, z in alto).",
    "trajectory_z":
        "Quota z nel frame TagSLAM livellato (z verticale verso l'alto) in funzione del tempo dall'inizio della GT.",
}


def _labels():
    try:
        return json.loads(os.environ.get("OVEVAL_LABELS", "{}"))
    except json.JSONDecodeError:
        return {}


def _figure_name(fig):
    titles = [fig._suptitle.get_text()] if fig._suptitle else []
    titles += [ax.get_title() for ax in fig.axes if ax.get_title()]
    title = titles[0] if titles else ""
    return re.sub(r"[^A-Za-z0-9]+", "_", title).strip("_").lower()


def _kind(fig):
    titles = " ".join(ax.get_title() for ax in fig.axes).lower()
    for key in ("relative orientation error", "relative position error", "normalized estimation error squared",
                "global orientation rpy error", "orientation error", "position error"):
        if key in titles:
            return key
    labels = " ".join(ax.get_xlabel() + "|" + ax.get_ylabel() for ax in fig.axes).lower()
    if "x-axis" in labels and "y-axis" in labels:
        return "trajectory_xy"
    if "z-axis" in labels:
        return "trajectory_z"
    return ""


def _same(c, ref):
    try:
        import matplotlib.colors as mc
        return mc.to_rgba(c) == ref
    except ValueError:
        return False


def _enhance(fig):
    names = _labels()
    algo = os.environ.get("OVEVAL_ALGO", "")
    algo_name, algo_color = names.get(algo, [algo, "#2a78d6"]) if algo else ("", "#2a78d6")
    body = os.environ.get("OVEVAL_BODY", "")
    kind = _kind(fig)
    first_labels = None

    for ax in fig.axes:
        if body and "IMU" in ax.get_title():
            ax.set_title(ax.get_title().replace("IMU", body))
        ax.grid(True, color=GRID, linewidth=0.6, zorder=0)
        ax.set_axisbelow(True)
        # error_comparison disegna i box con il colore della curva vuota usata per la legenda:
        # stesso cambio di colore per tutti gli elementi con quel colore
        import matplotlib.colors as mc
        remap = {mc.to_rgba(l.get_color()): names[l.get_label()][1] for l in ax.get_lines() if l.get_label() in names}
        for line in ax.get_lines():
            if line.get_label() not in names and mc.to_rgba(line.get_color()) in remap:
                line.set_color(remap[mc.to_rgba(line.get_color())])
        for line in ax.get_lines():
            lab = line.get_label()
            if lab in names:                       # plot_trajectories / error_comparison: chiave = nome file/cartella
                line.set_label(names[lab][0])
                line.set_color(names[lab][1])
                line.set_linewidth(1.6 if "GT" not in names[lab][0] else 2.0)
            elif lab == "error":
                line.set_label(f"errore {algo_name}".strip())
                line.set_color(algo_color)
            elif lab == "3 sigma bound" or (_same(line.get_color(), _OV_RED) and line.get_linestyle() == "--"):
                line.set_color(INK_2)
                line.set_linewidth(1.0)
                if lab == "3 sigma bound":
                    line.set_label("+-3 sigma stimato")
            elif lab in ("nees position", "nees orientation"):
                line.set_label(f"NEES {algo_name}".strip())
                line.set_color(algo_color)
            elif _same(line.get_color(), _OV_BLUE):  # boxplot di error_singlerun
                line.set_color(algo_color)
        if kind == "normalized estimation error squared" and ax.get_lines():
            ax.axhline(3.0, color=INK, linestyle=":", linewidth=1.2, label="valore atteso (3)")

        handles, labels = ax.get_legend_handles_labels()
        keep = {}
        for h, l in zip(handles, labels):
            if not l.startswith("_") and l not in keep:
                keep[l] = h
        if kind in ("relative orientation error", "relative position error") and not keep and algo_name:
            keep = {algo_name: Patch(facecolor="none", edgecolor=algo_color, label=algo_name),
                    "mediana": Line2D([], [], color=algo_color, label="mediana (tacca = IC 95%)")}
        if ax.get_legend() is not None:
            ax.get_legend().remove()
        if not keep:
            continue
        if first_labels is not None and list(keep) == first_labels:
            continue                               # sottografici con le stesse serie: legenda solo sul primo
        first_labels = list(keep)
        if kind.startswith("trajectory"):          # fuori dall'area dei dati, che nelle traiettorie e' tutta occupata
            ax.legend(list(keep.values()), list(keep.keys()), loc="upper left", bbox_to_anchor=(1.01, 1.0),
                      fontsize=8, framealpha=0.9, borderaxespad=0.0)
        else:
            ax.legend(list(keep.values()), list(keep.keys()), loc="best", fontsize=8, framealpha=0.9)

    title = os.environ.get("OVEVAL_PLOT_TITLE", "")
    if title:
        # sopra il bordo della figura (bbox_inches="tight" la include): non copre il titolo degli assi
        fig.suptitle(title, fontsize=11, color=INK, y=1.0, va="bottom")
    note = _NOTES.get(kind, "").replace("{align}", os.environ.get("OVEVAL_ALIGN", "se3"))
    if note:
        fig.text(0.01, -0.01, note, ha="left", va="top", fontsize=7.5, color=INK_2, wrap=True)


@_Backend.export
class _BackendSave(_Backend):
    FigureCanvas = FigureCanvasAgg
    FigureManager = FigureManagerBase

    @staticmethod
    def show(*args, **kwargs):
        global _count
        out_dir = os.environ.get("OVEVAL_PLOT_DIR", ".")
        prefix = os.environ.get("OVEVAL_PLOT_PREFIX", "ov_eval")
        os.makedirs(out_dir, exist_ok=True)
        for num in plt.get_fignums():
            fig = plt.figure(num)
            _count += 1
            name = _figure_name(fig)
            _enhance(fig)
            path = os.path.join(out_dir, f"{prefix}_{_count:02d}{'_' + name if name else ''}.png")
            fig.savefig(path, dpi=130, bbox_inches="tight")
            print(f"[plot] {path}")
        plt.close("all")
