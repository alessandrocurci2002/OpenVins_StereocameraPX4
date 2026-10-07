"""
Backend matplotlib che, invece di aprire le finestre, salva in PNG ogni figura che
ov_eval (matplotlib-cpp) mostra con plt.show(). Utile per avere i grafici di
error_singlerun / plot_trajectories come file, anche senza display.

Uso (nel container):
    export PYTHONPATH=/root/colcon_ws/src/open_vins/ov_data/sc_cantina01/OpenVINS_eval:$PYTHONPATH
    export MPLBACKEND=module://ov_eval_save_backend
    export OVEVAL_PLOT_DIR=results/plots_config_v9 OVEVAL_PLOT_PREFIX=error_singlerun
    ros2 run ov_eval error_singlerun se3 truths/dataset3.txt algorithms/config_v9/dataset3/run_1.txt
"""

import os
import re

import matplotlib.pyplot as plt
from matplotlib.backend_bases import FigureManagerBase, _Backend
from matplotlib.backends.backend_agg import FigureCanvasAgg

_count = 0


def _figure_name(fig):
    titles = [fig._suptitle.get_text()] if fig._suptitle else []
    titles += [ax.get_title() for ax in fig.axes if ax.get_title()]
    title = titles[0] if titles else ""
    return re.sub(r"[^A-Za-z0-9]+", "_", title).strip("_").lower()


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
            path = os.path.join(out_dir, f"{prefix}_{_count:02d}{'_' + name if name else ''}.png")
            fig.savefig(path, dpi=130, bbox_inches="tight")
            print(f"[plot] {path}")
        plt.close("all")
