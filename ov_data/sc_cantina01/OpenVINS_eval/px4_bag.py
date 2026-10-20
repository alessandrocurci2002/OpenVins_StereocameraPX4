"""
Lettura dei topic px4_msgs dai bag ROS2 registrati da vio_recorder.

I bag (sqlite3, Humble) non contengono le definizioni dei messaggi e px4_msgs non e'
installato nel container: le definizioni sono in px4_msgs/1.16 (scaricate da
github.com/PX4/px4_msgs, branch release/1.16). Sono quelle giuste per il firmware del
Pixhawk: EstimatorStatusFlags della 1.17 ha 3 bool in piu' (105 byte serializzati),
nei bag i messaggi sono lunghi 102 byte + 2 di padding CDR, come la 1.16.
VehicleOdometry ha lo stesso layout in entrambe le versioni.
"""

import sqlite3
from pathlib import Path

from rosbags.typesys import Stores, get_types_from_msg, get_typestore

PX4_MSGS_DIR = Path(__file__).resolve().parent / "px4_msgs" / "1.16"


def typestore():
    ts = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for f in sorted(PX4_MSGS_DIR.glob("*.msg")):
        types.update(get_types_from_msg(f.read_text(), f"px4_msgs/msg/{f.stem}"))
    ts.register(types)
    return ts


def db3_of(bag_dir):
    bag_dir = Path(bag_dir)
    return bag_dir if bag_dir.suffix == ".db3" else next(bag_dir.glob("*.db3"))


def read_topic(bag_dir, topic, ts=None):
    """Lista di (t_bag_ns, msg) per il topic, in ordine di registrazione."""
    ts = ts or typestore()
    db = sqlite3.connect(str(db3_of(bag_dir)))
    row = db.execute("select id, type from topics where name = ?", (topic,)).fetchone()
    if row is None:
        raise KeyError(f"topic {topic} non presente in {bag_dir}")
    tid, typ = row
    out = [(t, ts.deserialize_cdr(raw, typ))
           for t, raw in db.execute("select timestamp, data from messages where topic_id = ? order by timestamp", (tid,))]
    db.close()
    return out
