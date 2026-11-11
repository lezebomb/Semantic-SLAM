#!/usr/bin/env python3
"""Read semantic output metadata; no command publisher or service client.

This example was prepared locally, not run on the robot in this audit.
"""
import argparse
import json
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=('map', 'cloud', 'label', 'instance',
                                          'annotated', 'detector_status', 'map_status'),
                        default='map')
    parser.add_argument('--topic', help='Optional agreed topic remapping')
    args = parser.parse_args()

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, PointCloud2
    from std_msgs.msg import String

    specs = {
        'map': ('/semantic/map', PointCloud2),
        'cloud': ('/semantic/pointcloud', PointCloud2),
        'label': ('/semantic/label_image', Image),
        'instance': ('/semantic/instance_image', Image),
        'annotated': ('/semantic/image_annotated', Image),
        'detector_status': ('/semantic/detector_status', String),
        'map_status': ('/semantic/map_status', String),
    }
    topic, message_type = specs[args.kind]
    topic = args.topic or topic
    rclpy.init()
    node = rclpy.create_node('semantic_output_reader', enable_rosout=False)
    qos = QoSProfile(
        depth=1 if args.kind == 'map' else 5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if args.kind == 'map'
                    else DurabilityPolicy.VOLATILE),
    )
    last_print = 0.0

    def receive(msg):
        nonlocal last_print
        now = time.monotonic()
        if now - last_print < 1.0:
            return
        last_print = now
        data = {'topic': topic}
        if isinstance(msg, String):
            data['status'] = json.loads(msg.data)
        else:
            data.update(frame_id=msg.header.frame_id,
                        stamp_sec=msg.header.stamp.sec,
                        stamp_nanosec=msg.header.stamp.nanosec)
            if isinstance(msg, PointCloud2):
                data.update(points=msg.width * msg.height,
                            point_step=msg.point_step,
                            fields=[f.name for f in msg.fields])
            else:
                data.update(width=msg.width, height=msg.height, encoding=msg.encoding)
        print(json.dumps(data, ensure_ascii=False), flush=True)

    subscription = node.create_subscription(message_type, topic, receive, qos)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
