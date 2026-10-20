#!/usr/bin/env python3
"""
Recupero di un bag ROS2 (sqlite3) troncato: il file su disco e' piu' corto di quanto dichiara la sua
intestazione (pagine scritte nella cache e mai finite su disco, es. chiavetta tolta prima del sync), e
sqlite lo rifiuta con "database disk image is malformed".

Legge direttamente le pagine foglia delle tabelle, ricostruisce le righe di `topics` e di `messages`
(anche i blob lunghi, seguendo le catene di pagine di overflow) e scarta le righe che cadono fuori dal
file. Scrive un nuovo bag valido (schema rosbag2 + metadata.yaml ricalcolato), che ros2 bag play e
rosbags leggono normalmente.

Uso: python3 recover_db3.py BAG_TRONCATO.db3 CARTELLA_USCITA [--metadata metadata.yaml originale]
"""
import argparse
import os
import sqlite3
import struct
from collections import Counter, defaultdict
from pathlib import Path

import yaml


def varint(b, i):
    v = 0
    for k in range(9):
        c = b[i + k]
        if k == 8:
            return (v << 8) | c, i + 9
        v = (v << 7) | (c & 0x7F)
        if c < 0x80:
            return v, i + k + 1


def serial_len(t):
    return {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 6, 6: 8, 7: 8, 8: 0, 9: 0}.get(t, (t - 12) // 2 if t >= 12 else 0)


def decode(t, raw):
    if t == 0:
        return None
    if 1 <= t <= 6:
        return int.from_bytes(raw, "big", signed=True)
    if t == 7:
        return struct.unpack(">d", raw)[0]
    if t in (8, 9):
        return t - 8
    if t >= 12 and t % 2 == 0:
        return bytes(raw)
    return raw.decode("utf-8", "replace")


class Pager:
    def __init__(self, path):
        self.f = open(path, "rb")
        h = self.f.read(100)
        ps = struct.unpack(">H", h[16:18])[0]
        self.ps = 65536 if ps == 1 else ps
        self.usable = self.ps - h[20]
        self.npages = os.path.getsize(path) // self.ps
        self.declared = struct.unpack(">I", h[28:32])[0]

    def page(self, n):
        if n < 1 or n > self.npages:
            return None
        self.f.seek((n - 1) * self.ps)
        return self.f.read(self.ps)

    def payload(self, page, cp):
        """Payload completo di una cella di foglia di tabella (None se l'overflow cade fuori dal file)."""
        plen, i = varint(page, cp)
        rowid, i = varint(page, i)
        U = self.usable
        X = U - 35
        if plen <= X:
            return rowid, page[i:i + plen]
        M = ((U - 12) * 32 // 255) - 23
        K = M + ((plen - M) % (U - 4))
        local = K if K <= X else M
        data = bytearray(page[i:i + local])
        nxt = struct.unpack(">I", page[i + local:i + local + 4])[0]
        while len(data) < plen:
            p = self.page(nxt)
            if p is None:
                return rowid, None
            nxt = struct.unpack(">I", p[:4])[0]
            data += p[4:4 + min(U - 4, plen - len(data))]
        return rowid, bytes(data)


def record(payload):
    hlen, j = varint(payload, 0)
    types = []
    while j < hlen:
        t, j = varint(payload, j)
        types.append(t)
    k, vals = hlen, []
    for t in types:
        n = serial_len(t)
        vals.append(decode(t, payload[k:k + n]))
        k += n
    return vals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("db3")
    ap.add_argument("out")
    ap.add_argument("--metadata", default=None)
    a = ap.parse_args()
    pg = Pager(a.db3)
    print(f"pagine nel file {pg.npages}, dichiarate {pg.declared} ({100 * pg.npages / pg.declared:.1f}%)")
    topics, msgs, lost = {}, [], 0
    for n in range(1, pg.npages + 1):
        page = pg.page(n)
        off = 100 if n == 1 else 0
        if page[off] != 0x0D:
            continue
        ncell = struct.unpack(">H", page[off + 3:off + 5])[0]
        for c in range(ncell):
            try:
                cp = struct.unpack(">H", page[off + 8 + 2 * c:off + 10 + 2 * c])[0]
                rowid, pl = pg.payload(page, cp)
                if pl is None:
                    lost += 1
                    continue
                v = record(pl)
            except Exception:
                continue
            # topics: (id NULL, name, type, serialization_format, offered_qos_profiles[, ...])
            if len(v) >= 5 and v[0] is None and isinstance(v[1], str) and v[1].startswith("/") and isinstance(v[2], str) and "/" in v[2]:
                topics[rowid] = v[1:5]
            # messages: (id NULL, topic_id, timestamp, data)
            elif len(v) == 4 and v[0] is None and isinstance(v[1], int) and isinstance(v[2], int) and isinstance(v[3], bytes) \
                    and 1.0e18 < v[2] < 2.5e18:
                msgs.append((rowid, v[1], v[2], v[3]))
    msgs.sort(key=lambda r: r[2])
    print(f"topic trovati: {len(topics)}; messaggi recuperati: {len(msgs)}; righe con overflow fuori dal file: {lost}")
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    name = out.name + "_0.db3"
    dbp = out / name
    if dbp.exists():
        dbp.unlink()
    db = sqlite3.connect(str(dbp))
    db.execute("CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, "
               "serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)")
    db.execute("CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, data BLOB NOT NULL)")
    db.execute("CREATE INDEX timestamp_idx ON messages (timestamp ASC)")
    db.executemany("INSERT INTO topics VALUES (?,?,?,?,?)", [(k, *v) for k, v in sorted(topics.items())])
    db.executemany("INSERT INTO messages VALUES (?,?,?,?)", msgs)
    db.commit()
    db.close()
    cnt = Counter(m[1] for m in msgs)
    tmin, tmax = msgs[0][2], msgs[-1][2]
    meta = {"rosbag2_bagfile_information": {
        "version": 5, "storage_identifier": "sqlite3", "duration": {"nanoseconds": tmax - tmin},
        "starting_time": {"nanoseconds_since_epoch": tmin}, "message_count": len(msgs),
        "topics_with_message_count": [{"topic_metadata": {"name": v[0], "type": v[1], "serialization_format": v[2],
                                                          "offered_qos_profiles": v[3]}, "message_count": cnt.get(k, 0)}
                                      for k, v in sorted(topics.items())],
        "compression_format": "", "compression_mode": "", "relative_file_paths": [name],
        "files": [{"path": name, "starting_time": {"nanoseconds_since_epoch": tmin}, "duration": {"nanoseconds": tmax - tmin},
                   "message_count": len(msgs)}]}}
    (out / "metadata.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))
    span = defaultdict(list)
    for _, tid, t, _ in msgs:
        span[tid].append(t)
    for k, v in sorted(topics.items()):
        s = span.get(k, [0])
        print(f"  {v[0]:38s} {cnt.get(k, 0):6d} msg  {(min(s) - tmin) * 1e-9:6.1f} -> {(max(s) - tmin) * 1e-9:6.1f} s")
    print(f"[OK] {dbp} ({os.path.getsize(dbp) / 1e9:.2f} GB), durata recuperata {(tmax - tmin) * 1e-9:.1f} s")


if __name__ == "__main__":
    main()
