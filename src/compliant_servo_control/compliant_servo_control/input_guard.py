"""ROS-independent freshness contract (source stamp and local receive age)."""
from math import isfinite
from time import monotonic


class InputGuard:
    def __init__(self, timeout=.5):
        if not isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive and finite')
        self.timeout = timeout
        self.received = None
        self.source_ns = None

    def accept(self, frame, stamp_ns, now_ns, values=(), expected_frame='robot_base'):
        valid = (frame == expected_frame and stamp_ns > 0 and
                 0 <= now_ns - stamp_ns <= self.timeout * 1e9 and
                 all(isfinite(v) for v in values))
        if not valid:
            self.received = None
            return False
        self.received = monotonic()
        self.source_ns = stamp_ns
        return True

    def fresh(self, now_ns):
        return (self.received is not None and
                monotonic() - self.received <= self.timeout and
                0 <= now_ns - self.source_ns <= self.timeout * 1e9)


def stamp_ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


def run_node(node_type, args=None):
    import rclpy
    rclpy.init(args=args)
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
