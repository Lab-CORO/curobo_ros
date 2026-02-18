from curobo.util_file import load_yaml
from curobo_ros.cameras.camera_context import CameraContext


class CameraSystemManager:
    """
    Manages camera system configuration and setup.
    Responsible for:
    - Loading camera configuration from YAML files
    - Creating and managing CameraContext
    - Configuring multiple cameras with different types
    """

    def __init__(self, node, cameras_config_file: str):
        """
        Initialize camera system manager.

        Args:
            node: ROS2 node instance
            cameras_config_file: Path to YAML file containing camera configuration
        """
        self.node = node
        self.camera_context = None

        if cameras_config_file:
            self._load_and_configure_cameras(cameras_config_file)
        else:
            self.node.get_logger().info("No camera configuration file specified")

    def _load_and_configure_cameras(self, cameras_config_file: str):
        """
        Load camera configuration from YAML and configure all cameras.

        Args:
            cameras_config_file: Path to YAML configuration file
        """
        self.node.get_logger().info(f"Loading camera configuration from: {cameras_config_file}")

        try:
            # Load the YAML file
            camera_config = load_yaml(cameras_config_file)

            # Check if the configuration contains cameras
            if 'cameras' in camera_config and len(camera_config['cameras']) > 0:
                self.camera_context = CameraContext(self.node)
                print(camera_config)

                # Add each camera from the configuration
                for camera in camera_config['cameras']:
                    camera_name = camera.get("name", "unknown")
                    camera_type = camera.get("type", "point_cloud")  # Default to point_cloud
                    camera_topic = camera.get("topic", "")
                    camera_frame_id = camera.get("frame_id", "")
                    camera_info = camera.get("camera_info", '')
                    camera_intrinsics = camera.get("intrinsics", None)
                    camera_extrinsics = camera.get("extrinsics", None)

                    # Get pixel_size parameter if available (for point cloud cameras)
                    pixel_size = 0.01  # Default
                    if self.node.has_parameter('pixel_size'):
                        pixel_size = self.node.get_parameter('pixel_size').get_parameter_value().double_value

                    # Add camera with appropriate type
                    self.camera_context.add_camera(
                        camera_name=camera_name,
                        camera_type=camera_type,
                        topic=camera_topic,
                        camera_info=camera_info,
                        frame_id=camera_frame_id,
                        pixel_size=pixel_size,
                        intrinsics=camera_intrinsics,
                        extrinsics=camera_extrinsics
                    )

                self.node.get_logger().info(f"Successfully loaded {len(camera_config['cameras'])} camera(s)")
            else:
                self.node.get_logger().warn("Camera config file found but no cameras defined")

        except Exception as e:
            self.node.get_logger().error(f"Failed to load camera configuration from {cameras_config_file}: {e}")

    def get_camera_context(self):
        """
        Get the camera context.

        Returns:
            CameraContext instance or None if no cameras configured
        """
        return self.camera_context
