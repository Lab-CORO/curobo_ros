import rclpy
from rclpy.node import Node
from nav2_msgs.msg import VoxelGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from curobo_msgs.srv import GetVoxelGrid

class VoxelGridVisualizer(Node):
    def __init__(self):
        super().__init__('voxel_grid_visualizer')

        self.marker_pub = self.create_publisher(Marker, '/visualise_voxel_grid', 10)

    def get_and_visualize_voxel_grid(self):

        client = self.create_client(GetVoxelGrid, '/curobo_gen_traj/get_voxel_grid')
        while not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/get_voxel_grid service not available. Retrying...')

        request = GetVoxelGrid.Request()

        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Failed to call /get_voxel_grid service')
            return

        voxel_grid_msg = future.result()

        marker = self.create_voxel_grid_marker(voxel_grid_msg.voxel_grid)

        self.marker_pub.publish(marker)
        self.get_logger().info('Published voxel grid visualization.')

    def create_voxel_grid_marker(self, voxel_grid_msg):
        """
        Create a Marker message to visualize the VoxelGrid.

        Args:
            voxel_grid_msg (VoxelGrid): Received VoxelGrid message.

        Returns:
            Marker: Marker message for RViz2 visualization.
        """
        marker = Marker()
        marker.header.frame_id = "base_0"  # Adjust as needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "voxel_grid"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = voxel_grid_msg.resolutions.x
        marker.scale.y = voxel_grid_msg.resolutions.y
        marker.scale.z = voxel_grid_msg.resolutions.z

        # Set marker color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Transparency

        # Convert voxel grid data into cubes
        index = 0
        for x in range(voxel_grid_msg.size_z):
            for y in range(voxel_grid_msg.size_y):
                for z in range(voxel_grid_msg.size_x):
                    # Check if the voxel is occupied
                    if voxel_grid_msg.data[index] > 0:
                        # Compute voxel center position
                        point = Point()
                        point.x = voxel_grid_msg.origin.x + x * voxel_grid_msg.resolutions.x
                        point.y = voxel_grid_msg.origin.y + y * voxel_grid_msg.resolutions.y
                        point.z = voxel_grid_msg.origin.z + z * voxel_grid_msg.resolutions.z
                        marker.points.append(point)
                    index += 1

        return marker

def main(args=None):
    rclpy.init(args=args)
    node = VoxelGridVisualizer()
    node.get_and_visualize_voxel_grid()
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
