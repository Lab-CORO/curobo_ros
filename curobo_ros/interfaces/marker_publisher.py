import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
    
    def publish_markers(self, poses):
        marker_array = MarkerArray()
        
        for i, pose in enumerate(poses):
            if i == 0:
                # Create a red sphere marker for the start position
                sphere_marker = Marker()
                sphere_marker.header.frame_id = "base_0"
                sphere_marker.header.stamp = self.get_clock().now().to_msg()
                sphere_marker.ns = "trajectory"
                sphere_marker.id = i
                sphere_marker.type = Marker.SPHERE
                sphere_marker.action = Marker.ADD
                sphere_marker.scale.x = 0.01  # Taille de la sphère
                sphere_marker.scale.y = 0.01
                sphere_marker.scale.z = 0.01
                sphere_marker.color.a = 1.0  # Transparence
                sphere_marker.color.r = 1.0  # Couleur rouge
                sphere_marker.color.g = 0.0  # Couleur verte
                sphere_marker.color.b = 0.0  # Couleur bleue
                sphere_marker.pose = pose

                marker_array.markers.append(sphere_marker)
            elif i == len(poses) - 1:
                # Create a blue sphere marker for the end position
                sphere_marker = Marker()
                sphere_marker.header.frame_id = "base_0"
                sphere_marker.header.stamp = self.get_clock().now().to_msg()
                sphere_marker.ns = "trajectory"
                sphere_marker.id = i
                sphere_marker.type = Marker.SPHERE
                sphere_marker.action = Marker.ADD
                sphere_marker.scale.x = 0.01  # Taille de la sphère
                sphere_marker.scale.y = 0.01
                sphere_marker.scale.z = 0.01
                sphere_marker.color.a = 1.0  # Transparence
                sphere_marker.color.r = 0.0  # Couleur rouge
                sphere_marker.color.g = 0.0  # Couleur verte
                sphere_marker.color.b = 1.0  # Couleur bleue
                sphere_marker.pose = pose

                marker_array.markers.append(sphere_marker)
            else:
                # Create text marker for other positions
                text_marker = Marker()
                text_marker.header.frame_id = "base_0"
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "trajectory"
                text_marker.id = i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale.z = 0.01  # Taille du texte
                text_marker.color.a = 1.0  # Transparence
                text_marker.color.r = 1.0  # Couleur rouge
                text_marker.color.g = 1.0  # Couleur verte
                text_marker.color.b = 1.0  # Couleur bleue
                text_marker.pose = pose
                text_marker.text = str(i)  # Numéro du point

                marker_array.markers.append(text_marker)

        self.get_logger().info(f'Publishing marker array with {len(marker_array.markers)} markers')
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
