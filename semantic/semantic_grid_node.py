#!/usr/bin/env python3
"""Loose coupling: capture-time TF, bounded voxel votes, colored semantic cloud."""
import copy
import json
import time
from collections import deque
from pathlib import Path
import numpy as np
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
import tf2_ros
from common import POINT_DTYPE, make_points, cloud_message, rotation_matrix, run_node


class SemanticGrid(Node):
    def __init__(self):
        super().__init__('semantic_grid_node')
        defaults = dict(map_frame='map', voxel_size=0.05, max_voxels=200000,
                        tf_wait_wall_seconds=5.0, output_dir='')
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.output = Path(self.p['output_dir']) if self.p['output_dir'] else None
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=60))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pending = deque(maxlen=40)
        self.voxels = {}
        self.last_header = None
        self.dirty = False
        self.stats = dict(received=0, transformed=0, transformed_points=0, tf_retries=0,
                          tf_dropped=0, queue_dropped=0, evicted_voxels=0,
                          map_frame=self.p['map_frame'], last_tf_error='', transform_samples=[])
        self.create_subscription(PointCloud2, '/semantic/points_camera', self.receive, 5)
        self.frame_pub = self.create_publisher(PointCloud2, '/semantic/pointcloud', 5)
        self.map_pub = self.create_publisher(PointCloud2, '/semantic/map',
                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(String, '/semantic/map_status', 5)
        self.create_timer(0.05, self.transform_pending)
        self.create_timer(1.0, self.publish_map)
        self.create_timer(10.0, self.save)
        self.create_service(Trigger, '/semantic/save_map', self.save_service)
        self.get_logger().info('GRID_READY capture-time TF to map, voxel size=' + str(self.p['voxel_size']))

    def receive(self, msg):
        self.stats['received'] += 1
        if len(self.pending) == self.pending.maxlen:
            self.stats['queue_dropped'] += 1
        self.pending.append((time.monotonic(), msg))

    def transform_pending(self):
        for _ in range(len(self.pending)):
            arrived, msg = self.pending.popleft()
            try:
                # No latest-TF fallback and no blocking wait in the TF listener executor.
                tf = self.tf_buffer.lookup_transform(self.p['map_frame'], msg.header.frame_id,
                                                     Time.from_msg(msg.header.stamp))
            except tf2_ros.TransformException as e:
                self.stats['tf_retries'] += 1
                self.stats['last_tf_error'] = str(e)
                if time.monotonic() - arrived < self.p['tf_wait_wall_seconds']:
                    self.pending.append((arrived, msg))
                else:
                    self.stats['tf_dropped'] += 1
                    self.get_logger().warning('Capture-time TF unavailable: ' + str(e), throttle_duration_sec=5)
                continue
            points = np.frombuffer(msg.data, dtype=POINT_DTYPE).copy()
            xyz_camera = np.column_stack([points[n] for n in ('x', 'y', 'z')])
            t = tf.transform.translation
            translation = np.array([t.x, t.y, t.z])
            xyz = xyz_camera @ rotation_matrix(tf.transform.rotation).T + translation
            for i, n in enumerate(('x', 'y', 'z')):
                points[n] = xyz[:, i]
            header = copy.deepcopy(msg.header)
            header.frame_id = self.p['map_frame']
            self.frame_pub.publish(cloud_message(points, header))
            self.last_header = header
            self.stats['transformed'] += 1
            self.stats['transformed_points'] += len(points)
            if len(self.stats['transform_samples']) < 3:
                self.stats['transform_samples'].append(dict(source=msg.header.frame_id,
                    stamp_ns=header.stamp.sec*10**9+header.stamp.nanosec,
                    translation=translation.tolist(), quaternion=[tf.transform.rotation.x,
                    tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w],
                    camera_point=xyz_camera[0].tolist(), map_point=xyz[0].tolist()))
            keys = np.floor(xyz / self.p['voxel_size']).astype(np.int32)
            # One vote per voxel/class/frame avoids bias from point density.
            combined = np.column_stack((keys, points['class_id']))
            _, first = np.unique(combined, axis=0, return_index=True)
            for i in first:
                key = tuple(keys[i])
                cid = int(points['class_id'][i])
                iid = int(points['instance_id'][i])
                if key not in self.voxels:
                    if len(self.voxels) >= self.p['max_voxels']:
                        self.voxels.pop(next(iter(self.voxels)))
                        self.stats['evicted_voxels'] += 1
                    self.voxels[key] = [np.zeros(3), 0, {}, iid]
                cell = self.voxels[key]
                cell[0] += xyz[i]
                cell[1] += 1
                cell[2][cid] = cell[2].get(cid, 0) + 1
                cell[3] = iid
            self.dirty = True

    def map_points(self):
        if not self.voxels:
            return np.zeros(0, dtype=POINT_DTYPE)
        xyz, classes, instances = [], [], []
        for sums, count, votes, iid in self.voxels.values():
            xyz.append(sums/count)
            classes.append(max(votes, key=votes.get))
            instances.append(iid)
        return make_points(np.asarray(xyz), classes, instances)

    def publish_map(self):
        if not self.last_header:
            return
        if self.dirty:
            # A accumulated map is fixed in map; publish at current simulated time.
            header = Header(frame_id=self.p['map_frame'], stamp=self.get_clock().now().to_msg())
            self.map_pub.publish(cloud_message(self.map_points(), header))
            self.dirty = False
        self.status_pub.publish(String(data=json.dumps(dict(self.stats, voxels=len(self.voxels)))))

    def save(self):
        if not self.output:
            return
        points = self.map_points()
        self.output.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(self.output/'semantic_map.npz', points=points, frame_id=self.p['map_frame'],
                            voxel_size=self.p['voxel_size'])
        header = ('ply\nformat binary_little_endian 1.0\nelement vertex ' + str(len(points)) +
                  '\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\n'
                  'property uchar green\nproperty uchar blue\nproperty uint class_id\n'
                  'property uint instance_id\nend_header\n')
        dtype = np.dtype([(n,'<f4') for n in ('x','y','z')] +
                         [(n,'u1') for n in ('red','green','blue')] +
                         [(n,'<u4') for n in ('class_id','instance_id')])
        ply = np.zeros(len(points),dtype=dtype)
        for n in ('x','y','z','class_id','instance_id'):
            ply[n]=points[n]
        rgb = points['rgb'].view(np.uint32)
        for n, shift in [('red',16),('green',8),('blue',0)]:
            ply[n]=(rgb >> shift)&255
        with (self.output/'semantic_map.ply').open('wb') as f:
            f.write(header.encode()); f.write(ply.tobytes())
        classes, counts = np.unique(points['class_id'], return_counts=True)
        summary = dict(self.stats, voxels=len(points), pending=len(self.pending),
                       class_voxels={str(int(k)):int(v) for k,v in zip(classes,counts)})
        (self.output/'map_stats.json').write_text(json.dumps(summary,indent=2))

    def save_service(self, request, response):
        self.save()
        response.success = bool(self.output)
        response.message = str(self.output) if self.output else 'No output_dir configured'
        return response

    def close(self):
        self.save()


if __name__ == '__main__':
    run_node(SemanticGrid)
