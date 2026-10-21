"""Wall-stamped ROS bridge. Simulation time remains a separate telemetry field."""
import json
from time import monotonic, sleep
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from std_msgs.msg import Bool, String
from compliant_servo_control.input_guard import run_node
from .backend import CoppeliaBackend, reaction
from .contracts import MotionGuard, wrench_at_tip


class ContactBridge(Node):
    def __init__(self, backend=None):
        super().__init__('coppeliasim_bridge')
        host = self.declare_parameter('host', 'localhost').value
        port = self.declare_parameter('port', 23000).value
        sign = self.declare_parameter('reaction_sign', 0).value
        frame = self.declare_parameter('frame_id', 'robot_base').value
        acceptance_build = self.declare_parameter('acceptance_build', '').value
        acceptance_calibration = self.declare_parameter('acceptance_calibration', '').value
        self.acceptance_mode = bool(acceptance_build)
        if frame != 'robot_base' or self.get_parameter('use_sim_time').value:
            raise ValueError('prototype requires robot_base and wall ROS time')
        self.guard = MotionGuard(sign=sign)
        if acceptance_build or acceptance_calibration:
            if backend is not None or not acceptance_build or not acceptance_calibration:
                raise ValueError('acceptance mode requires build + calibration and a real engine backend')
            from .acceptance_live import open_prepared
            self.backend, evidence = open_prepared(acceptance_build,host,port,acceptance_calibration)
            if sign != evidence['calibration']['sign']:
                self.backend.close()
                raise ValueError('reaction_sign differs from verified calibration')
            self.backend.metadata = {**self.backend.metadata, 'acceptance_evidence': evidence}
        else:
            self.backend = backend or CoppeliaBackend(host, port)
        self.pose_pub = self.create_publisher(PoseStamped, '/simulation/tool_pose', 10)
        self.force_pub = self.create_publisher(WrenchStamped, '/force_torque/wrench', 10)
        self.raw_pub = self.create_publisher(WrenchStamped, '/simulation/raw_sensor_wrench', 10)
        self.telemetry_pub = self.create_publisher(String, '/simulation/contact_telemetry', 10)
        self.stop_pub = self.create_publisher(Bool, '/simulation/bridge_stop', 10)
        self.create_subscription(TwistStamped, '/simulation/cmd_twist', self.on_command, 10)
        self.create_subscription(Bool, '/assembly/emergency_stop', self.on_stop, 10)
        if acceptance_build:
            # Discovery finishes before the first engine step, so an observer
            # cannot silently substitute a successful tail for the full trace.
            deadline = monotonic() + 5.
            while self.telemetry_pub.get_subscription_count() == 0:
                if monotonic() >= deadline:
                    self.backend.close()
                    raise RuntimeError('start the acceptance observer before the bridge')
                sleep(.02)
        self.backend.start()
        self.sample = None
        self.last_tick = monotonic()
        self.create_timer(self.backend.dt, self.tick)
        if sign == 0:
            self.get_logger().warning('Unarmed: reaction_sign=0; raw feedback only, all motion blocked')

    def on_command(self, msg):
        v, w = msg.twist.linear, msg.twist.angular
        self.guard.accept(msg.header.frame_id,
                          msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                          self.get_clock().now().nanoseconds*1e-9, monotonic(),
                          (v.x,v.y,v.z),(w.x,w.y,w.z))

    def on_stop(self, msg):
        self.guard.halted |= msg.data

    def tick(self):
        now = monotonic()
        # A long wall stall invalidates motion even if a queued command is fresh.
        if now-self.last_tick > .5:
            self.guard.halted = True
        self.last_tick = now
        try:
            sample = self.sample
            f = reaction(sample, self.guard.sign) if sample is not None else None
            velocity = self.guard.velocity(self.get_clock().now().nanoseconds*1e-9, now,
                sample['pose'][:3] if sample else (.4,0,.23), f or (0.,)*6, f is not None)
            if self.acceptance_mode and (sample is None or sample['simulation_time_s'] < .2-1e-9):
                velocity = (0.,0.,0.)
            self.backend.command(velocity)
            sample = self.backend.advance()
            self.sample = sample
            f = reaction(sample, self.guard.sign)
            # Inspect fresh force immediately after each dynamics step.
            self.guard.velocity(self.get_clock().now().nanoseconds*1e-9, monotonic(),
                                sample['pose'][:3], f or (0.,)*6, f is not None)
            if self.guard.halted or f is None:
                self.backend.command((0.,0.,0.))
            stamp = self.get_clock().now().to_msg()
            p = PoseStamped()
            p.header.frame_id, p.header.stamp = 'robot_base', stamp
            p.pose.position.x,p.pose.position.y,p.pose.position.z = map(float, sample['pose'][:3])
            p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w = map(float, sample['pose'][3:])
            self.pose_pub.publish(p)
            def publish_wrench(pub, values, frame):
                msg = WrenchStamped()
                msg.header.frame_id, msg.header.stamp = frame, stamp
                msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z = map(float, values[:3])
                msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z = map(float, values[3:])
                pub.publish(msg)
            if sample['sensor_valid']:
                publish_wrench(self.raw_pub, (*sample['raw_force'],*sample['raw_torque']), 'wrist_sensor')
            if f is not None:
                # WrenchStamped frame names include the moment origin: use base
                # origin here; keep the tip-referenced moment separately in telemetry.
                base_wrench = wrench_at_tip(sample['sensor_matrix'], sample['raw_force'],
                    sample['raw_torque'], (0.,0.,0.), self.guard.sign)
                publish_wrench(self.force_pub, base_wrench, 'robot_base')
            telemetry = {**sample, 'source': 'coppeliasim_physics', 'reaction_at_tip': f,
                         'reaction_sign': self.guard.sign, 'stop_latched': self.guard.halted,
                         'command_applied_mps': velocity, 'wall_stamp_ns': self.get_clock().now().nanoseconds,
                         'metadata': self.backend.metadata}
            telemetry['command_requested_mps'] = self.guard.command
            telemetry['command_after_sample_mps'] = ((0.,0.,0.) if self.guard.halted or f is None else velocity)
            if self.acceptance_mode:
                telemetry['phase'] = 'settle' if sample['simulation_time_s'] <= .2+1e-9 else 'trial'
            self.telemetry_pub.publish(String(data=json.dumps(telemetry, allow_nan=False)))
            self.stop_pub.publish(Bool(data=self.guard.halted))
        except Exception:
            self.guard.halted = True
            self.stop_pub.publish(Bool(data=True))
            # Raise to propagate process exit and stop the independent physical launch.
            raise

    def destroy_node(self):
        try:
            self.backend.close()
        except Exception as exc:
            self.get_logger().error(f'Simulator cleanup failed: {exc}')
        return super().destroy_node()


def main(args=None):
    run_node(ContactBridge, args)
