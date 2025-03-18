#!/usr/bin/env python3

import copy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker


class SingleMarkerBroadcaster(Node):
    '''
    With the new version of curobo_rviz, this class is useless
    '''
    def __init__(self):
        super().__init__('target_pose_publisher')

        # Initialize the InteractiveMarkerServer
        self.server = InteractiveMarkerServer(self, "target_pose")
        self.get_logger().info("InteractiveMarkerServer initialized.")

        # Initialize pose
        self.pose = PoseStamped()

        # Set the global frame
        self.global_frame = 'odom'

        # Publisher for the desired end effector pose
        self.pose_pub = self.create_publisher(PoseStamped, '/perceptive_mpc/desired_end_effector_pose', 10)
        self.get_logger().info("Publisher for desired end effector pose initialized.")

    def make_box(self, msg):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.5
        return marker

    def create_marker(self):
        self.int_marker = InteractiveMarker()
        int_marker = self.int_marker
        int_marker.header.frame_id = self.global_frame
        int_marker.header.stamp = self.get_clock().now().to_msg()
        self.pose.header = int_marker.header

        int_marker.pose.position.x = 1.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.0
        self.pose.pose.position = int_marker.pose.position

        int_marker.pose.orientation.x = -0.707
        int_marker.pose.orientation.y = 0.707
        int_marker.pose.orientation.z = 0.0
        int_marker.pose.orientation.w = 0.0
        self.pose.pose.orientation = int_marker.pose.orientation

        int_marker.scale = 0.2

        int_marker.name = "PoseTarget"
        int_marker.description = "Pose target for the end effector"

        control = InteractiveMarkerControl()

        # Custom move on plane
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.name = "click"
        control.markers.append(self.make_box(int_marker))
        int_marker.controls.append(copy.deepcopy(control))

        # Insertion de l'InteractiveMarker
        self.server.insert(int_marker)
        self.get_logger().info("InteractiveMarker inserted.")

        # Associer le callback à ce marqueur
        self.server.setCallback(int_marker.name, self.update_pose_callback)
        self.get_logger().info("Callback set for InteractiveMarker.")

    def apply_changes(self):
        self.server.applyChanges()
        self.get_logger().info("Applied changes to InteractiveMarkerServer.")

    def update_pose_callback(self, feedback):
        self.get_logger().info(f'Callback triggered with event type: {feedback.event_type}')

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.get_logger().info(f'Pose updated: {feedback.pose.position}, {feedback.pose.orientation}')
            self.pose.header.frame_id = self.global_frame
            self.pose.header.stamp = self.get_clock().now().to_msg()
            self.pose.pose.position = feedback.pose.position
            self.pose.pose.orientation = feedback.pose.orientation
        elif feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.get_logger().info("Marker clicked, publishing new goal!")
            self.pose_pub.publish(self.pose)


def main(args=None):
    rclpy.init(args=args)
    node = SingleMarkerBroadcaster()
    node.create_marker()
    node.apply_changes()

    try:
        rclpy.spin(node)
        node.get_logger().info("Spinning the node.")
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Shutting down the node.")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()