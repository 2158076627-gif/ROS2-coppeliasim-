"""A known, synthetic hole pose. No image processing or sensor fusion."""
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from compliant_servo_control.input_guard import run_node


class MockPerceptionNode(Node):
    def __init__(self):
        super().__init__('mock_perception_node')
        self.pub = self.create_publisher(PoseStamped, '/perception/axis_hole_pose', 10)
        self.create_timer(.1, self.tick)

    def tick(self):
        msg = PoseStamped()
        msg.header.frame_id = 'robot_base'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.z = .4, .2
        msg.pose.orientation.w = 1.
        self.pub.publish(msg)


def main(args=None):
    run_node(MockPerceptionNode, args)
