#create a test to send a joint_state msg to the fk service and get the ee position and orientation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from data_generation.srv import Fk


class TestFk(Node):
    def __init__(self):
        super().__init__('test_fk')
        self.client = self.create_client(Fk, '/curobo/fk_poses')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Fk.Request()
        self.req.joint_states = []
        self.req.joint_states.append(JointState())
        self.req.joint_states[0].position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.req.joint_states[0].name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.callback)

    def callback(self, future):
        response = future.result()
        self.get_logger().info('Result %s' % response.ee_poses[0].position)
        self.get_logger().info('Result %s' % response.ee_poses[0].orientation)
        # assert the existanc eof a result
        assert response.ee_poses[0].position

def main(args=None):
    rclpy.init(args=args)
    test_fk = TestFk()
    rclpy.spin(test_fk)
    test_fk.destroy_node()
    rclpy.shutdown()