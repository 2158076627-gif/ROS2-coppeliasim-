"""Generate artificial six-component wrench from the toy tool pose."""
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from .synthetic_force import synthetic_wrench
from .input_guard import InputGuard, stamp_ns, run_node


class MockForceSensorNode(Node):
    def __init__(self):
        super().__init__('mock_force_sensor_node')
        self.frame = self.declare_parameter('frame_id', 'robot_base').value
        self.guard = InputGuard(self.declare_parameter('input_timeout_s', .5).value)
        self.position = None
        rate = self.declare_parameter('rate_hz', 50.).value
        if not 1. <= rate <= 500.:
            raise ValueError('rate_hz must be in [1, 500]')
        self.pub = self.create_publisher(WrenchStamped, self.declare_parameter(
            'wrench_topic', '/force_torque/wrench').value, 10)
        self.create_subscription(PoseStamped, '/simulation/tool_pose', self.on_pose, 10)
        self.create_timer(1. / rate, self.tick)

    def on_pose(self, msg):
        p = msg.pose.position
        if self.guard.accept(msg.header.frame_id, stamp_ns(msg),
                             self.get_clock().now().nanoseconds, (p.x, p.y, p.z), self.frame):
            self.position = (p.x, p.y, p.z)

    def tick(self):
        if not self.guard.fresh(self.get_clock().now().nanoseconds):
            return
        sample = synthetic_wrench(self.position)
        msg = WrenchStamped()
        msg.header.frame_id = self.frame
        # Preserve pose measurement time: a stalled source cannot appear fresh.
        msg.header.stamp.sec = self.guard.source_ns // 10**9
        msg.header.stamp.nanosec = self.guard.source_ns % 10**9
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = sample.fx, sample.fy, sample.fz
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = sample.tx, sample.ty, sample.tz
        self.pub.publish(msg)


def main(args=None):
    run_node(MockForceSensorNode, args)
