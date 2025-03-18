#!/usr/bin/env python3

import time
from geometry_msgs.msg import Point, PoseStamped
from interactive_markers import InteractiveMarkerServer
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarkerFeedback


class SimpleMarker(Node):
    '''
    Useless this the new curobo_rviz
    '''
    def __init__(self):
        super().__init__('simple_marker')
        self.server = InteractiveMarkerServer(self, 'simple_marker')
        self.pose_publisher = self.create_publisher(PoseStamped, 'marker_pose', 10)

        # To detect double click
        self.last_click_time = 0
        self.double_click_threshold = 0.3  # seconds

        position = Point(x=0.0, y=0.0, z=0.0)
        self.make6DofMarker(position)

    def processFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            current_time = time.time()
            time_since_last_click = current_time - self.last_click_time

            if time_since_last_click < self.double_click_threshold:
                self.get_logger().info(f'{feedback.marker_name} was double-clicked')

                # Create PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header.frame_id = feedback.header.frame_id
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose = feedback.pose

                # Publish the pose
                self.pose_publisher.publish(pose_msg)
                self.get_logger().info(f'Published pose: {pose_msg.pose.position}, {pose_msg.pose.orientation}')

            self.last_click_time = current_time

    def make6DofMarker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'base_0'
        int_marker.pose.position = position
        int_marker.scale = 1.0

        int_marker.name = 'simple_6dof'
        int_marker.description = 'Simple 6-DOF Control'

        # Create a grey arrow marker
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.1  # Length of the arrow
        arrow_marker.scale.y = 0.02  # Width of the arrow shaft
        arrow_marker.scale.z = 0.02  # Height of the arrow shaft
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        # Create a control that contains the arrow
        arrow_control = InteractiveMarkerControl()
        arrow_control.always_visible = True
        arrow_control.markers.append(arrow_marker)
        int_marker.controls.append(arrow_control)

        # Add 6 DOF controls
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = 'rotate_x'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = 'rotate_y'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Add the marker to the server
        self.server.insert(int_marker, feedback_callback=self.processFeedback)
        self.server.applyChanges()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMarker()

    rclpy.spin(node)
    node.server.clear()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()