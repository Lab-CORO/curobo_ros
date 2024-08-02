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
            marker = Marker()
            marker.header.frame_id = "base_0"  # Assurez-vous que le frame_id est correct
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectory"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.05  # Taille de la sphère
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0  # Transparence
            marker.color.r = 1.0  # Couleur rouge
            marker.color.g = 0.0  # Couleur verte
            marker.color.b = 0.0  # Couleur bleue

            marker.pose = pose

            marker_array.markers.append(marker)

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
