"""Real ROS middleware with injected transport; NOT an engine or physics test."""
import json
import time
import pytest
rclpy=pytest.importorskip('rclpy',reason='ROS 2 is not installed/sourced')
from geometry_msgs.msg import TwistStamped, PoseStamped, WrenchStamped
from std_msgs.msg import Bool, String
from coppeliasim_contact.bridge import ContactBridge


class InjectedBackend:
    dt=.02
    metadata={'source':'injected_transport_no_physics'}
    def __init__(self): self.time=0.; self.commands=[]; self.force=2.; self.valid=True
    def start(self): pass
    def close(self): pass
    def command(self,v): self.commands.append(v)
    def advance(self):
        self.time+=self.dt
        return dict(simulation_time_s=self.time,pose=[.4,0,.23,0,0,0,1],
                    sensor_matrix=[1,0,0,.4,0,1,0,0,0,0,1,.26],
                    raw_force=[0,0,self.force],raw_torque=[0,0,0],sensor_valid=self.valid,
                    contacts=[],contact_list_truncated=False)


@pytest.mark.ros
def test_bridge_ros_contract_rejects_rotation_and_latches_force():
    rclpy.init(args=['--ros-args','-p','reaction_sign:=1'])
    backend=InjectedBackend()
    bridge=ContactBridge(backend=backend)
    node=rclpy.create_node('contact_bridge_test_observer')
    pub=node.create_publisher(TwistStamped,'/simulation/cmd_twist',10)
    stop=node.create_publisher(Bool,'/assembly/emergency_stop',10)
    latest={}
    subs=[node.create_subscription(kind,topic,lambda msg,k=key: latest.update({k:msg}),10)
          for kind,topic,key in [(PoseStamped,'/simulation/tool_pose','pose'),
                                (WrenchStamped,'/force_torque/wrench','force'),
                                (String,'/simulation/contact_telemetry','telemetry')]]
    executor=rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bridge); executor.add_node(node)
    def pump(duration,angular=0.,publish=True):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            if publish:
                msg=TwistStamped(); msg.header.frame_id='robot_base'
                msg.header.stamp=node.get_clock().now().to_msg()
                msg.twist.linear.z=-.001; msg.twist.angular.x=angular
                pub.publish(msg)
            executor.spin_once(timeout_sec=.005)
    try:
        deadline=time.monotonic()+10
        while (len(latest)<3 or pub.get_subscription_count()==0) and time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.02)
        assert len(latest)==3
        pump(.2)
        assert (0.,0.,-.001) in backend.commands
        assert latest['pose'].header.frame_id=='robot_base'
        assert latest['force'].wrench.force.z==2.
        assert json.loads(latest['telemetry'].data)['simulation_time_s']>0
        pump(.2,angular=.01)
        assert backend.commands[-1]==(0.,0.,0.)
        pump(.7,publish=False)
        assert backend.commands[-1]==(0.,0.,0.)
        backend.valid=False
        pump(.15)
        assert backend.commands[-1]==(0.,0.,0.)
        backend.valid=True; backend.force=50.; pump(.15)
        assert bridge.guard.halted
        backend.force=0.; pump(.15)
        assert backend.commands[-1]==(0.,0.,0.)
    finally:
        executor.shutdown(); bridge.destroy_node(); node.destroy_node(); rclpy.shutdown()
