"""ROS adapter for Cartesian waypoints; no IK or collision checking."""
from dataclasses import fields
from math import isfinite
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import String
from .planner_core import AssemblyTrajectoryPlanner, PlannerConfig, Pose3D


class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner_node')
        config = PlannerConfig(**{f.name: self.declare_parameter(f.name, f.default).value
                                  for f in fields(PlannerConfig)})
        if not all(isfinite(getattr(config, f.name)) and getattr(config, f.name) > 0
                   for f in fields(config)):
            raise ValueError('planner distances, speeds and tolerances must be positive and finite')
        self.planner = AssemblyTrajectoryPlanner(config)
        self.axis = self.declare_parameter('approach_axis', [0., 0., 1.]).value
        self.frame = self.declare_parameter('frame_id', 'robot_base').value
        self.pub = self.create_publisher(PoseArray, self.declare_parameter(
            'trajectory_topic', '/assembly/trajectory_plan').value, 10)
        self.path_pub = self.create_publisher(Path, self.declare_parameter(
            'path_topic', '/assembly/trajectory_path').value, 10)
        self.stage_pub = self.create_publisher(String, self.declare_parameter(
            'stage_topic', '/assembly/trajectory_stage_names').value, 10)
        self.create_subscription(PoseStamped, self.declare_parameter(
            'target_pose_topic', '/perception/axis_hole_pose').value, self.on_target, 10)

    def on_target(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        age = self.get_clock().now().nanoseconds - (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
        if (msg.header.frame_id != self.frame or not all(isfinite(v) for v in values)
                or not 0 <= age <= 500000000
                or abs(sum(v*v for v in values[3:]) - 1.) > 1e-3):
            self.get_logger().warning('Rejected target: frame, freshness, or pose invalid')
            return
        try:
            waypoints = self.planner.plan(Pose3D(values[:3], values[3:]), self.axis)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        result = PoseArray(header=msg.header)
        path = Path(header=msg.header)
        for waypoint in waypoints:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = waypoint.pose.position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = waypoint.pose.orientation
            result.poses.append(pose)
            path.poses.append(PoseStamped(header=msg.header, pose=pose))
        self.pub.publish(result)
        self.path_pub.publish(path)
        self.stage_pub.publish(String(data=','.join(w.name for w in waypoints)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
