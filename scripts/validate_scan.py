#!/usr/bin/env python3
"""Subscribe to real /scan, save measurements and check timing; no device writes."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--seconds', type=float, default=10)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if not 0 < a.seconds <= 30:
        p.error('--seconds must be in (0, 30]')
    a.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('deployment_scan_observer', enable_rosout=False)
    arrivals, stamps, valid_counts, lengths = [], [], [], []
    latest = None

    def receive(msg):
        nonlocal latest
        latest = msg
        arrivals.append(time.monotonic())
        stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        ranges = np.asarray(msg.ranges)
        good = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges <= msg.range_max)
        valid_counts.append(int(good.sum()))
        lengths.append(len(ranges))

    node.create_subscription(LaserScan, '/scan', receive, qos_profile_sensor_data)
    until = time.monotonic() + a.seconds
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.2)
    result = dict(messages=len(arrivals), seconds=a.seconds,
                  domain_id=rclpy.get_default_context().get_domain_id())
    result['publishers'] = [dict(node=e.node_name, namespace=e.node_namespace,
                                reliability=str(e.qos_profile.reliability),
                                durability=str(e.qos_profile.durability))
                            for e in node.get_publishers_info_by_topic('/scan')]
    if latest is not None:
        m = latest
        ranges = np.asarray(m.ranges, dtype=np.float32)
        good = np.isfinite(ranges) & (ranges > m.range_min) & (ranges <= m.range_max)
        dt = np.diff(stamps)
        result.update(frame_id=m.header.frame_id, readings_min=min(lengths),
                      readings_max=max(lengths), valid_points_min=min(valid_counts),
                      valid_points_max=max(valid_counts),
                      arrival_hz=(len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else 0,
                      stamps_increasing=bool(len(dt)>0 and np.all(dt>0)),
                      timestamp_age_s=time.time()-stamps[-1],
                      angle_min=m.angle_min, angle_max=m.angle_max,
                      angle_increment=m.angle_increment,
                      scan_time=m.scan_time, time_increment=m.time_increment,
                      range_min=m.range_min, range_max=m.range_max,
                      last_valid_min_m=float(ranges[good].min()) if good.any() else None,
                      last_valid_max_m=float(ranges[good].max()) if good.any() else None)
        np.savez_compressed(a.output/'last_scan.npz', ranges=ranges,
                            intensities=np.asarray(m.intensities, dtype=np.float32),
                            angle_min=m.angle_min, angle_increment=m.angle_increment,
                            stamp=stamps[-1], frame_id=m.header.frame_id)
    result['success'] = bool(latest is not None and len(arrivals)>=20
                             and min(valid_counts)>0 and result['stamps_increasing']
                             and abs(result['timestamp_age_s'])<2
                             and math.isfinite(latest.angle_increment)
                             and latest.angle_increment>0)
    node.destroy_node()
    rclpy.shutdown()
    (a.output/'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
