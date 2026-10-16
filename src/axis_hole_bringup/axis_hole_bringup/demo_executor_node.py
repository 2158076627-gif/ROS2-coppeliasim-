"""Single-shot fixed-orientation toy execution; refuses unsupported plans."""
from time import monotonic
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped, WrenchStamped
from std_msgs.msg import Bool, String
from assembly_trajectory_planner.planner_core import Pose3D, Waypoint
from compliant_servo_control.admittance_controller import ServoCorrection, AdmittanceController, WrenchSample
from compliant_servo_control.input_guard import InputGuard, stamp_ns, run_node
from .demo_core import Executor


class DemoExecutorNode(Node):
    def __init__(self):
        super().__init__('demo_executor_node')
        self.follower = Executor()
        timeout = self.declare_parameter('input_timeout_s', .5).value
        self.input_guards = {key: InputGuard(timeout) for key in ('plan', 'pose', 'twist', 'wrench')}
        self.plan = None
        self.position = (0., 0., 0.)
        self.correction = ServoCorrection()
        self.previous = monotonic()
        self.pub = self.create_publisher(TwistStamped, '/simulation/cmd_twist', 10)
        self.status_pub = self.create_publisher(String, '/assembly/execution_state', 10)
        self.create_subscription(PoseArray, '/assembly/trajectory_plan', self.on_plan, 10)
        self.create_subscription(PoseStamped, '/simulation/tool_pose', self.on_pose, 10)
        self.create_subscription(TwistStamped, '/assembly/compliant_twist', self.on_twist, 10)
        self.create_subscription(WrenchStamped, '/force_torque/wrench', self.on_wrench, 10)
        self.create_subscription(Bool, '/assembly/emergency_stop', self.on_stop, 10)
        self.create_timer(.02, self.tick)

    def valid(self, key, msg, values):
        return self.input_guards[key].accept(msg.header.frame_id, stamp_ns(msg),
                                       self.get_clock().now().nanoseconds, values)

    def on_plan(self, msg):
        values = tuple(v for p in msg.poses for v in (
            p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        if not self.valid('plan', msg, values):
            return
        # This toy executor supports exactly the six canonical +Z-approach stages.
        if len(msg.poses) != 6 or any(
                abs(p.orientation.x) + abs(p.orientation.y) + abs(p.orientation.z) > 1e-6
                or abs(p.orientation.w - 1.) > 1e-6 for p in msg.poses):
            self.follower.halted = True
            return
        poses = [Pose3D((p.position.x, p.position.y, p.position.z)) for p in msg.poses]
        if (any(abs(p.position[0] - poses[0].position[0]) > 1e-6 or
                abs(p.position[1] - poses[0].position[1]) > 1e-6 for p in poses)
                or not (poses[0].position[2] > poses[1].position[2] > poses[2].position[2]
                        > poses[3].position[2] == poses[4].position[2])
                or poses[5].position[2] <= poses[3].position[2]
                or any(not (.3 <= p.position[0] <= .5 and -.1 <= p.position[1] <= .1
                            and .18 <= p.position[2] <= .4) for p in poses)):
            self.follower.halted = True
            return
        names = ('observe', 'pre_align', 'contact_search', 'insert', 'hold', 'retract')
        speeds = (.08, .02, .004, .004, 0., .02)
        plan = [Waypoint(name, pose, speed, .001, .5 if name == 'hold' else 0.)
                for name, pose, speed in zip(names, poses, speeds)]
        if self.plan is None:
            self.plan = plan
        elif self.plan != plan:
            # Replanning during motion is outside the minimal contract.
            self.follower.halted = True

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        if self.valid('pose', msg, (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6 or abs(q.w - 1.) > 1e-6:
                self.input_guards['pose'].received = None
                return
            self.position = (p.x, p.y, p.z)

    def on_twist(self, msg):
        v, w = msg.twist.linear, msg.twist.angular
        if self.valid('twist', msg, (v.x, v.y, v.z, w.x, w.y, w.z)):
            self.correction = ServoCorrection(vx=v.x, vy=v.y, vz=v.z)

    def on_wrench(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        values = (f.x, f.y, f.z, t.x, t.y, t.z)
        if self.valid('wrench', msg, values):
            # Independent default overload guard in addition to servo's configured guard.
            self.follower.halted |= AdmittanceController().compute(WrenchSample(*values)).emergency_stop

    def on_stop(self, msg):
        self.follower.halted |= msg.data

    def tick(self):
        now = monotonic()
        dt, self.previous = max(1e-6, min(now - self.previous, .05)), now
        ready = all(g.fresh(self.get_clock().now().nanoseconds) for g in self.input_guards.values())
        velocity = self.follower.step(self.plan, self.position, self.correction, dt) if ready else (0., 0., 0.)
        state = ('halted' if self.follower.halted else 'complete' if self.follower.done
                 else 'waiting_for_inputs' if not ready else self.plan[self.follower.index].name)
        msg = TwistStamped()
        msg.header.frame_id, msg.header.stamp = 'robot_base', self.get_clock().now().to_msg()
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = velocity
        self.pub.publish(msg)
        self.status_pub.publish(String(data=state))


def main(args=None):
    run_node(DemoExecutorNode, args)
