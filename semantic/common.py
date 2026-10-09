"""Shared wire format: real display RGB, separate class and observation IDs."""
import colorsys
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2, PointField

POINT_DTYPE = np.dtype([(n, '<f4' if n in ('x', 'y', 'z', 'rgb') else '<u4')
                        for n in ('x', 'y', 'z', 'rgb', 'class_id', 'instance_id')])
COLORS = np.array([tuple(round(c * 255) for c in colorsys.hsv_to_rgb(
    (i * 0.61803398875) % 1, 0.75, 1)) for i in range(80)], dtype=np.uint32)
RGB = (COLORS[:, 0] << 16) | (COLORS[:, 1] << 8) | COLORS[:, 2]


def cloud_message(points, header):
    msg = PointCloud2(header=header, height=1, width=len(points),
                      is_bigendian=False, point_step=POINT_DTYPE.itemsize,
                      row_step=POINT_DTYPE.itemsize * len(points), is_dense=True)
    msg.fields = [PointField(name=n, offset=i * 4, count=1,
                    datatype=PointField.FLOAT32 if i < 4 else PointField.UINT32)
                  for i, n in enumerate(POINT_DTYPE.names)]
    msg.data = points.tobytes()
    return msg


def make_points(xyz, classes, instances):
    points = np.zeros(len(xyz), dtype=POINT_DTYPE)
    for i, n in enumerate(('x', 'y', 'z')):
        points[n] = xyz[:, i]
    points['class_id'] = classes
    points['instance_id'] = instances
    points['rgb'] = RGB[np.asarray(classes, dtype=int)].view(np.float32)
    return points


def rotation_matrix(q):
    a = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    a /= np.linalg.norm(a)
    x, y, z, w = a
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def run_node(cls):
    rclpy.init()
    node = None
    try:
        node = cls()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
