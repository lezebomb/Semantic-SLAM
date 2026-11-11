#!/usr/bin/env python3
"""Subscribe to a whitelist of existing chassis feedback topics; never publish control."""
import argparse
import json
import math
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', choices=('/encoder', '/chassis_charge',
                                          '/chassis_docking', '/chassis_initok',
                                          '/ultrasound_value'), default='/encoder')
    parser.add_argument('--seconds', type=float, default=10.0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 60:
        parser.error('--seconds must be in (0, 60]')

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import Bool, Float32MultiArray

    typ = Bool if args.topic in ('/chassis_docking', '/chassis_initok') else Float32MultiArray
    rclpy.init(args=['--ros-args', '--disable-rosout-logs', '--disable-external-lib-logs'])
    node = rclpy.create_node('chassis_feedback_reader', enable_rosout=False,
                            start_parameter_services=False)
    count = 0
    first = last = None

    def receive(msg):
        nonlocal count, first, last
        last = time.monotonic()
        if first is None:
            first = last
        count += 1
        data = (bool(msg.data) if isinstance(msg, Bool) else
                [x if math.isfinite(x) else str(x) for x in msg.data])
        if count == 1 or count % 20 == 0:
            print(json.dumps({'topic': args.topic, 'count': count, 'raw_data': data,
                              'note': 'Array field meanings and units are not assigned here.'},
                             ensure_ascii=False), flush=True)

    qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(typ, args.topic, receive, qos)
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    hz = (count - 1) / (last - first) if count > 1 and last > first else None
    print(json.dumps({'topic': args.topic, 'received': count,
                      'receive_hz': hz, 'control_publications': 0}), flush=True)


if __name__ == '__main__':
    main()
