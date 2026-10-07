#!/usr/bin/env python3
"""
Estrae una traiettoria (nav_msgs/Odometry) da una rosbag ROS1
e la salva nel formato richiesto da ov_eval:
    # timestamp(s) tx ty tz qx qy qz qw

Uso:
    python3 extract_gt.py input.bag --topic /tagslam/odom/body_rig --out truths/dataset1.txt
"""

import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader


def extract(bag_path: str, topic: str, out_path: str):
    rows = []
    frame_ids = set()
    child_frame_ids = set()

    with AnyReader([Path(bag_path)]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in reader.connections})
            raise RuntimeError(
                f"Topic '{topic}' non trovato. Topic disponibili: {available}"
            )

        for connection, _bag_time_ns, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)

            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation

            frame_ids.add(msg.header.frame_id)
            child_frame_ids.add(msg.child_frame_id)

            rows.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    rows.sort(key=lambda r: r[0])

    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        f.write("# timestamp(s) tx ty tz qx qy qz qw\n")
        for r in rows:
            f.write("{:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f}\n".format(*r))

    print(f"[OK] {len(rows)} pose scritte in: {out_path}")
    print(f"     header.frame_id visti:       {frame_ids}")
    print(f"     child_frame_id visti:        {child_frame_ids}")
    if rows:
        print(f"     range temporale: {rows[0][0]:.3f} -> {rows[-1][0]:.3f} s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="path al file .bag")
    parser.add_argument("--topic", default="/tagslam/odom/body_rig")
    parser.add_argument("--out", required=True, help="file di output .txt")
    args = parser.parse_args()
    extract(args.bag, args.topic, args.out)