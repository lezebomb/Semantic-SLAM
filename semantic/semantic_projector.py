#!/usr/bin/env python3
"""Project synchronized masks with 32FC1 meters or 16UC1 millimeters."""
import json
from pathlib import Path
import message_filters
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from common import make_points, cloud_message, run_node


class SemanticProjector(Node):
    def __init__(self):
        super().__init__('semantic_projector')
        for k, v in dict(min_depth=0.3, max_depth=5.0, sample_step=4, output_dir='').items():
            self.declare_parameter(k, v)
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.step = self.get_parameter('sample_step').value
        self.output = self.get_parameter('output_dir').value
        self.bridge = CvBridge()
        self.stats = dict(bundles=0, clouds=0, points=0, invalid_depth=0, errors=0,
                          depth_encodings={}, source_frames={})
        self.pub = self.create_publisher(PointCloud2, '/semantic/points_camera', 5)
        self.subs = [message_filters.Subscriber(self, typ, '/semantic/' + topic)
                     for typ, topic in ((Image, 'label_image'), (Image, 'instance_image'),
                                        (Image, 'paired_depth'), (CameraInfo, 'paired_camera_info'))]
        self.sync = message_filters.TimeSynchronizer(self.subs, 20)
        self.sync.registerCallback(self.project)
        self.get_logger().info('PROJECTOR_READY exact synchronization of inference bundle')

    def project(self, label_msg, instance_msg, depth_msg, info):
        self.stats['bundles'] += 1
        try:
            label = self.bridge.imgmsg_to_cv2(label_msg, 'mono8')
            instance = self.bridge.imgmsg_to_cv2(instance_msg, 'mono16')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            if depth_msg.encoding == '16UC1':
                depth = depth.astype(np.float32) * 0.001
            elif depth_msg.encoding != '32FC1':
                raise ValueError(f'Unsupported depth encoding {depth_msg.encoding}')
            if label.shape != depth.shape or (info.height, info.width) != depth.shape:
                raise ValueError('Mask/depth/CameraInfo resolution mismatch')
            if depth_msg.header.frame_id != info.header.frame_id:
                raise ValueError('Depth and CameraInfo frame mismatch')
            fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
            if fx <= 0 or fy <= 0 or any(abs(d) > 1e-6 for d in info.d):
                raise ValueError('Requires valid rectified pinhole intrinsics')
            ys, xs = np.mgrid[0:depth.shape[0]:self.step, 0:depth.shape[1]:self.step]
            z = depth[::self.step, ::self.step]
            labels = label[::self.step, ::self.step]
            ids = instance[::self.step, ::self.step]
            valid_depth = np.isfinite(z) & (z >= self.min_depth) & (z <= self.max_depth)
            valid = valid_depth & (labels > 0) & (ids > 0)
            self.stats['invalid_depth'] += int(((labels > 0) & ~valid_depth).sum())
            xyz = np.column_stack(((xs[valid]-cx)*z[valid]/fx,
                                   (ys[valid]-cy)*z[valid]/fy, z[valid]))
            points = make_points(xyz, labels[valid].astype(np.uint32)-1, ids[valid])
            if len(points):
                self.pub.publish(cloud_message(points, depth_msg.header))
                self.stats['clouds'] += 1
                self.stats['points'] += len(points)
            for key, value in [('depth_encodings', depth_msg.encoding),
                               ('source_frames', depth_msg.header.frame_id)]:
                self.stats[key][value] = self.stats[key].get(value, 0) + 1
            if self.stats['bundles'] % 25 == 1:
                self.get_logger().info(f"bundle={self.stats['bundles']} valid semantic points={len(points)} frame={depth_msg.header.frame_id}")
        except Exception as e:
            self.stats['errors'] += 1
            self.get_logger().error(f'Projection rejected: {e}')

    def close(self):
        if self.output:
            Path(self.output, 'projector_stats.json').write_text(json.dumps(self.stats, indent=2))


if __name__ == '__main__':
    run_node(SemanticProjector)
