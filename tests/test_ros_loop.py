"""Real middleware/process integration. Skipped explicitly without ROS 2.

Run AFTER colcon build and sourcing install/setup.bash. No ROS mocks here.
"""
import contextlib
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
rclpy = pytest.importorskip('rclpy', reason='ROS 2 is not installed/sourced')
from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped, WrenchStamped
from std_msgs.msg import Bool, String

pytestmark = pytest.mark.ros


@contextlib.contextmanager
def process(command, path):
    with open(path, 'w') as log:
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            yield proc
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)


@pytest.fixture
def observer():
    rclpy.init()
    node = rclpy.create_node('assembly_test_observer')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_launch_completes_feedback_cycle(observer, tmp_path):
    latest, seen, speeds, zs = {}, set(), set(), []
    def receive(topic):
        def callback(msg):
            latest[topic] = msg
            if topic == '/assembly/execution_state':
                seen.add(msg.data)
            if topic == '/assembly/compliant_twist':
                speeds.add(round(msg.twist.linear.z, 6))
            if topic == '/simulation/tool_pose':
                zs.append(msg.pose.position.z)
        return callback
    types = {'/perception/axis_hole_pose': PoseStamped,
             '/assembly/trajectory_plan': PoseArray, '/simulation/tool_pose': PoseStamped,
             '/assembly/trajectory_stage_names': String,
             '/force_torque/wrench': WrenchStamped, '/assembly/compliant_twist': TwistStamped,
             '/assembly/contact_state': Bool, '/assembly/emergency_stop': Bool,
             '/assembly/execution_state': String, '/simulation/cmd_twist': TwistStamped}
    contact_seen = []
    subs = [observer.create_subscription(kind, topic, receive(topic), 10) for topic, kind in types.items()]
    subs.append(observer.create_subscription(Bool, '/assembly/contact_state',
                                             lambda m: contact_seen.append(m.data), 10))
    with process(['ros2', 'launch', 'axis_hole_bringup', 'assembly_agent_demo.launch.py'],
                 tmp_path / 'launch.log') as proc:
        deadline = time.monotonic() + 100
        while time.monotonic() < deadline:
            assert proc.poll() is None, (tmp_path / 'launch.log').read_text()
            rclpy.spin_once(observer, timeout_sec=.05)
            if 'complete' in seen:
                # Drain a few callbacks so final pose and zero command arrive.
                end = time.monotonic() + .5
                while time.monotonic() < end:
                    rclpy.spin_once(observer, timeout_sec=.02)
                break
        assert 'complete' in seen, (seen, (tmp_path / 'launch.log').read_text())
        assert set(types) <= set(latest)
        graph = dict(observer.get_topic_names_and_types())
        for topic, kind in types.items():
            module = kind.__module__.split('.')[0]
            assert graph[topic] == [f'{module}/msg/{kind.__name__}']
        assert len(latest['/assembly/trajectory_plan'].poses) == 6
        # DDS discovery may occur after the short observe/pre_align legs. Validate
        # the complete plan contract separately from observed execution states.
        assert latest['/assembly/trajectory_stage_names'].data == (
            'observe,pre_align,contact_search,insert,hold,retract')
        plan = latest['/assembly/trajectory_plan'].poses
        assert plan[0].position.z > plan[1].position.z > plan[2].position.z > plan[3].position.z
        assert plan[3] == plan[4]
        assert plan[5].position.z > plan[3].position.z
        assert {'contact_search', 'insert', 'hold', 'retract', 'complete'} <= seen
        assert any(contact_seen) and not all(contact_seen)
        assert -.001 in speeds and -.00035 in speeds
        assert min(zs) < .199
        assert zs[-1] >= .209 - 1e-6
        command = latest['/simulation/cmd_twist'].twist
        assert (command.linear.x, command.linear.y, command.linear.z) == (0., 0., 0.)
        assert not latest['/assembly/emergency_stop'].data
        assert latest['/force_torque/wrench'].header.frame_id == 'robot_base'


def test_servo_timeout_invalid_data_and_latched_overload(observer, tmp_path):
    results, stops = [], []
    pub = observer.create_publisher(WrenchStamped, '/guard_test/wrench', 10)
    sub = observer.create_subscription(TwistStamped, '/guard_test/twist',
                                       lambda m: results.append(m.twist.linear.z), 10)
    stop_sub = observer.create_subscription(Bool, '/guard_test/stop', lambda m: stops.append(m.data), 10)
    command = ['ros2', 'run', 'compliant_servo_control', 'compliant_servo_node', '--ros-args',
               '-r', '/force_torque/wrench:=/guard_test/wrench',
               '-r', '/assembly/compliant_twist:=/guard_test/twist',
               '-r', '/assembly/contact_state:=/guard_test/contact',
               '-r', '/assembly/emergency_stop:=/guard_test/stop']
    with process(command, tmp_path / 'servo.log') as proc:
        def exercise(fz=None, frame='robot_base', stale=False, duration=1.2):
            results.clear()
            end = time.monotonic() + duration
            while time.monotonic() < end:
                assert proc.poll() is None, (tmp_path / 'servo.log').read_text()
                if fz is not None:
                    msg = WrenchStamped()
                    msg.header.frame_id = frame
                    msg.header.stamp = observer.get_clock().now().to_msg()
                    if stale:
                        msg.header.stamp.sec -= 2
                    msg.wrench.force.z = fz
                    pub.publish(msg)
                rclpy.spin_once(observer, timeout_sec=.02)
            assert results, 'servo did not publish'
            return results[-10:]
        # Allow DDS discovery by checking actual output rather than sleeping.
        deadline = time.monotonic() + 15
        while not results and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=.1)
        assert results
        assert all(v == pytest.approx(-.00035) for v in exercise(14.))
        assert all(v == 0. for v in exercise())
        assert all(v == 0. for v in exercise(14., frame='wrong_frame'))
        assert all(v == 0. for v in exercise(14., stale=True))
        assert all(v == 0. for v in exercise(math.nan))
        assert all(v == 0. for v in exercise(50.))
        assert all(v == 0. for v in exercise(14.))
        assert stops[-1] is True


def test_feedback_source_loss_stops_executor(observer, tmp_path):
    """Launch nodes in reverse dataflow order; stop only the force source."""
    from ament_index_python.packages import get_package_share_directory
    config = str(Path(get_package_share_directory('axis_hole_bringup')) / 'config' / 'demo.yaml')
    latest = {}
    sub = observer.create_subscription(TwistStamped, '/simulation/cmd_twist',
                                       lambda m: latest.update(command=m), 10)
    pose_sub = observer.create_subscription(PoseStamped, '/simulation/tool_pose',
                                            lambda m: latest.update(pose=m), 10)
    state_sub = observer.create_subscription(String, '/assembly/execution_state',
                                             lambda m: latest.update(state=m.data), 10)
    definitions = [('axis_hole_bringup', 'demo_executor_node'),
                   ('compliant_servo_control', 'compliant_servo_node'),
                   ('compliant_servo_control', 'mock_force_sensor_node'),
                   ('axis_hole_bringup', 'mock_plant_node'),
                   ('assembly_trajectory_planner', 'trajectory_planner_node'),
                   ('axis_hole_bringup', 'mock_perception_node')]
    with contextlib.ExitStack() as stack:
        processes = {}
        for pkg, name in definitions:
            processes[name] = stack.enter_context(process(
                ['ros2', 'run', pkg, name, '--ros-args', '--params-file', config], tmp_path / f'{name}.log'))
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline and latest.get('state') != 'contact_search':
            assert all(p.poll() is None for p in processes.values())
            rclpy.spin_once(observer, timeout_sec=.02)
        assert latest.get('state') == 'contact_search'
        force = processes['mock_force_sensor_node']
        os.killpg(force.pid, signal.SIGINT)
        force.wait(timeout=10)
        end = time.monotonic() + 1.
        while time.monotonic() < end:
            rclpy.spin_once(observer, timeout_sec=.02)
        assert latest['state'] == 'waiting_for_inputs'
        v = latest['command'].twist.linear
        assert (v.x, v.y, v.z) == (0., 0., 0.)
        z = latest['pose'].pose.position.z
        end = time.monotonic() + .3
        while time.monotonic() < end:
            rclpy.spin_once(observer, timeout_sec=.02)
        assert latest['pose'].pose.position.z == pytest.approx(z, abs=1e-9)


def test_plant_command_watchdog(observer, tmp_path):
    poses = []
    pub = observer.create_publisher(TwistStamped, '/simulation/cmd_twist', 10)
    sub = observer.create_subscription(PoseStamped, '/simulation/tool_pose',
                                       lambda m: poses.append(m.pose.position.z), 10)
    with process(['ros2', 'run', 'axis_hole_bringup', 'mock_plant_node'], tmp_path / 'plant.log') as proc:
        deadline = time.monotonic() + 15
        while not poses and time.monotonic() < deadline:
            assert proc.poll() is None
            rclpy.spin_once(observer, timeout_sec=.05)
        assert poses
        start_z = poses[-1]
        end = time.monotonic() + .8
        while time.monotonic() < end:
            msg = TwistStamped()
            msg.header.stamp = observer.get_clock().now().to_msg()
            msg.header.frame_id = 'robot_base'
            msg.twist.linear.z = .01
            pub.publish(msg)
            rclpy.spin_once(observer, timeout_sec=.02)
        assert poses[-1] > start_z + .002
        end = time.monotonic() + .8
        while time.monotonic() < end:
            rclpy.spin_once(observer, timeout_sec=.02)
        stopped_z = poses[-1]
        end = time.monotonic() + .3
        while time.monotonic() < end:
            rclpy.spin_once(observer, timeout_sec=.02)
        assert poses[-1] == pytest.approx(stopped_z, abs=1e-9)
