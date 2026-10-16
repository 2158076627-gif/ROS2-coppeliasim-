"""Heuristic wrench feedback with timeout-to-zero and latched overload stop."""
from dataclasses import fields
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, TwistStamped
from std_msgs.msg import Bool
from .admittance_controller import AdmittanceConfig, AdmittanceController, WrenchSample, ServoCorrection
from .input_guard import InputGuard, stamp_ns, run_node


class CompliantServoNode(Node):
    def __init__(self):
        super().__init__('compliant_servo_node')
        config = AdmittanceConfig(**{f.name: self.declare_parameter(f.name, f.default).value
                                    for f in fields(AdmittanceConfig)})
        self.controller = AdmittanceController(config)
        self.frame = self.declare_parameter('frame_id', 'robot_base').value
        self.guard = InputGuard(self.declare_parameter('input_timeout_s', .5).value)
        self.sample = WrenchSample()
        self.stopped = False
        self.pub = self.create_publisher(TwistStamped, self.declare_parameter(
            'twist_topic', '/assembly/compliant_twist').value, 10)
        self.contact_pub = self.create_publisher(Bool, self.declare_parameter(
            'contact_topic', '/assembly/contact_state').value, 10)
        self.stop_pub = self.create_publisher(Bool, '/assembly/emergency_stop', 10)
        self.create_subscription(WrenchStamped, self.declare_parameter(
            'wrench_topic', '/force_torque/wrench').value, self.on_wrench, 10)
        self.create_timer(.02, self.tick)

    def on_wrench(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        values = (f.x, f.y, f.z, t.x, t.y, t.z)
        if self.guard.accept(msg.header.frame_id, stamp_ns(msg),
                             self.get_clock().now().nanoseconds, values, self.frame):
            self.sample = WrenchSample(*values)
            self.stopped |= self.controller.compute(self.sample).emergency_stop

    def tick(self):
        now = self.get_clock().now()
        correction = (self.controller.compute(self.sample)
                      if self.guard.fresh(now.nanoseconds) and not self.stopped
                      else ServoCorrection(emergency_stop=self.stopped))
        msg = TwistStamped()
        msg.header.stamp, msg.header.frame_id = now.to_msg(), self.frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = correction.vx, correction.vy, correction.vz
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = correction.wx, correction.wy, correction.wz
        self.pub.publish(msg)
        self.contact_pub.publish(Bool(data=correction.contact))
        self.stop_pub.publish(Bool(data=self.stopped))


def main(args=None):
    run_node(CompliantServoNode, args)
