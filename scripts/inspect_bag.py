#!/usr/bin/env python3
"""Read-only rosbag2 SQLite inspection, using actual message formats and headers."""
import argparse
import json
import os
import sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def inspect_bag(bag):
    bag = Path(bag).expanduser().resolve()
    dbs = sorted(bag.glob('*.db3')) if bag.is_dir() else [bag]
    if len(dbs) != 1:
        raise ValueError('This Indoor demo expects exactly one SQLite bag file')
    with sqlite3.connect(dbs[0].as_uri() + '?mode=ro', uri=True) as con:
        topics, rgb, depth, infos, edges = [], [], [], [], set()
        for tid, name, typ in con.execute('SELECT id,name,type FROM topics'):
            row = con.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1',(tid,)).fetchone()
            count = con.execute('SELECT count(*) FROM messages WHERE topic_id=?',(tid,)).fetchone()[0]
            normalized = '/' + name.lstrip('/')
            topic = dict(recorded_name=name, ros_name=normalized, type=typ, messages=count)
            if row:
                message_type = get_message(typ)
                msg = deserialize_message(row[0], message_type)
                if hasattr(msg, 'header'):
                    topic['frame_id'] = msg.header.frame_id
                    topic['first_stamp_ns'] = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
                if typ.endswith('/CompressedImage'):
                    topic['format'] = msg.format
                    if normalized.endswith('/compressedDepth'):
                        topic['base_topic'] = normalized.removesuffix('/compressedDepth')
                        depth.append(topic)
                    elif normalized.endswith('/compressed'):
                        topic['base_topic'] = normalized.removesuffix('/compressed')
                        rgb.append(topic)
                if typ.endswith('/CameraInfo'):
                    topic.update(width=msg.width, height=msg.height, k=list(msg.k), d=list(msg.d))
                    infos.append(topic)
                if typ.endswith('/TFMessage'):
                    for (data,) in con.execute('SELECT data FROM messages WHERE topic_id=?',(tid,)):
                        for t in deserialize_message(data, message_type).transforms:
                            edges.add((t.header.frame_id, t.child_frame_id))
            topics.append(topic)
        start, end, count = con.execute('SELECT min(timestamp),max(timestamp),count(*) FROM messages').fetchone()
    if len(rgb) != 1 or len(depth) != 1:
        raise ValueError('Cannot uniquely select RGB compressed and compressedDepth topics')
    info = [t for t in infos if t['frame_id'] == rgb[0]['frame_id']]
    if len(info) != 1:
        raise ValueError('Cannot uniquely select matching RGB CameraInfo')
    if depth[0]['frame_id'] != rgb[0]['frame_id']:
        raise ValueError('This demo needs registered depth in the RGB optical frame')
    return dict(bag_db=str(dbs[0]), duration_s=(end-start)/1e9, total_messages=count,
                topics=topics, tf_edges=sorted(edges), has_recorded_map=any(t['ros_name']=='/map' for t in topics),
                selected=dict(rgb_base=rgb[0]['base_topic'], depth_base=depth[0]['base_topic'],
                              camera_info=info[0]['ros_name'], optical_frame=info[0]['frame_id']))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    workspace = Path(os.environ.get('RTABMAP_WS', str(Path.home() / 'ros2')))
    p.add_argument('bag', nargs='?', default=str(workspace / 'data/demo_mapping/demo_mapping_bag'))
    p.add_argument('--output')
    args = p.parse_args()
    result = json.dumps(inspect_bag(args.bag), indent=2)
    if args.output:
        Path(args.output).write_text(result)
    print(result)
