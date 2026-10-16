"""Integrate ONLY /simulation/cmd_twist; no robot or CoppeliaSim connection."""
from time import monotonic
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from compliant_servo_control.input_guard import InputGuard, stamp_ns, run_node
from .demo_core import DemoPlant


class MockPlantNode(Node):
    def __init__(self):
        super().__init__('mock_plant_node')
        self.plant = DemoPlant()
        self.guard = InputGuard(self.declare_parameter('input_timeout_s', .5).value)
        self.velocity = (0., 0., 0.)
        self.previous = monotonic()
        self.pub = self.create_publisher(PoseStamped, '/simulation/tool_pose', 10)
        self.create_subscription(TwistStamped, '/simulation/cmd_twist', self.on_command, 10)
        self.create_timer(.02, self.tick)

    def on_command(self, msg):
        v, w = msg.twist.linear, msg.twist.angular
        values = (v.x, v.y, v.z, w.x, w.y, w.z)
        valid = self.guard.accept(msg.header.frame_id, stamp_ns(msg),
                                  self.get_clock().now().nanoseconds, values)
        if valid and (w.x, w.y, w.z) == (0., 0., 0.):
            self.velocity = values[:3]
        else:
            self.guard.received = None

    def tick(self):
        now = monotonic()
        dt, self.previous = min(now - self.previous, .05), now
        self.plant.step(self.velocity if self.guard.fresh(self.get_clock().now().nanoseconds)
                        else (0., 0., 0.), dt)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'robot_base'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.plant.position
        msg.pose.orientation.w = 1.
        self.pub.publish(msg)


def main(args=None):
    run_node(MockPlantNode, args)
