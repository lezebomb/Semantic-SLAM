#!/usr/bin/env python3
"""Reuse mapping verification with wall time and camera-only visual odometry."""
import csv
import json
import math
from pathlib import Path
import sys
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from observe_ros import Observer


class HandpushObserver(Observer):
    def __init__(self, output):
        super().__init__(output, allow_empty=True, use_sim_time=False, geometry_frame='camera_link')
        self.stats['odometry'] = dict(messages=0, valid_messages=0, distance_m=0.0)
        self.last_position = None
        self.stream = (output/'visual_odometry.csv').open('w')
        self.writer = csv.writer(self.stream)
        self.writer.writerow(['stamp_sec','stamp_nanosec','x','y','z','qx','qy','qz','qw','covariance_x'])
        self.subs.append(self.create_subscription(Odometry, '/handpush/odom', self.odom, qos_profile_sensor_data))

    def odom(self, msg):
        item = self.stats['odometry']
        item['messages'] += 1
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        position = [p.x,p.y,p.z]
        item.update(last_arrival_wall_time=time.time(), frame=msg.header.frame_id,
                    child_frame=msg.child_frame_id, last_position=position,
                    last_covariance_x=float(msg.pose.covariance[0]))
        if all(math.isfinite(x) for x in position) and msg.pose.covariance[0] < 999:
            item['valid_messages'] += 1
            if self.last_position is not None:
                item['distance_m'] += math.sqrt(sum((a-b)**2 for a,b in zip(position,self.last_position)))
            self.last_position = position
        self.writer.writerow([msg.header.stamp.sec,msg.header.stamp.nanosec,*position,q.x,q.y,q.z,q.w,msg.pose.covariance[0]])
        self.stream.flush()


if __name__ == '__main__':
    rclpy.init()
    node = HandpushObserver(Path(sys.argv[1]))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save()
        node.stream.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
