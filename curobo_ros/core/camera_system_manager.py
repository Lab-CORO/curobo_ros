from curobo.config_io import load_yaml
from curobo_ros.cameras.camera_context import CameraContext

# Cadence supposée pour une caméra dont le YAML ne déclare pas `frame_rate_hz`.
# Volontairement HAUTE : cette valeur sert à normaliser la décroissance du TSDF
# (voir ObstacleManager._resolve_time_decay). Surestimer le débit rapproche
# `time_decay` de 1.0 -> oubli trop lent, obstacles rémanents : conservateur pour
# la collision. Le sous-estimer efface la carte plus vite que prévu -> obstacles
# réels qui disparaissent : c'est le sens dangereux. On préfère donc trop haut.
DEFAULT_CAMERA_FRAME_RATE_HZ = 30.0


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
                    camera_frame_rate = self._parse_frame_rate(camera, camera_name)

                    # Add camera with appropriate type
                    self.camera_context.add_camera(
                        camera_name=camera_name,
                        camera_type=camera_type,
                        topic=camera_topic,
                        camera_info=camera_info,
                        frame_id=camera_frame_id,
                        intrinsics=camera_intrinsics,
                        extrinsics=camera_extrinsics,
                        frame_rate_hz=camera_frame_rate
                    )

                self.node.get_logger().info(f"Successfully loaded {len(camera_config['cameras'])} camera(s)")
            else:
                self.node.get_logger().warn("Camera config file found but no cameras defined")

        except Exception as e:
            self.node.get_logger().error(f"Failed to load camera configuration from {cameras_config_file}: {e}")

    def _parse_frame_rate(self, camera: dict, camera_name: str) -> float:
        """Read `frame_rate_hz` for one camera entry, with a safe fallback.

        The rate is not used to drive any subscription: it only feeds the TSDF
        decay normalisation (the decay fires once per integrate(), so the sum of
        the camera rates is the real decay rate). A missing or invalid value is
        replaced by DEFAULT_CAMERA_FRAME_RATE_HZ and logged, because a silently
        wrong rate mistunes how fast the collision map forgets obstacles.

        Args:
            camera: One entry of the `cameras:` list from the YAML config.
            camera_name: Name used in the log messages.

        Returns:
            The publication rate in Hz (strictly positive).
        """
        raw = camera.get("frame_rate_hz", None)
        if raw is None:
            self.node.get_logger().warn(
                f"Camera '{camera_name}' has no 'frame_rate_hz' - assuming "
                f"{DEFAULT_CAMERA_FRAME_RATE_HZ} Hz for the TSDF decay "
                f"normalisation. Declare the real rate in the cameras config."
            )
            return DEFAULT_CAMERA_FRAME_RATE_HZ

        try:
            rate = float(raw)
        except (TypeError, ValueError):
            rate = 0.0
        if rate <= 0.0:
            self.node.get_logger().warn(
                f"Camera '{camera_name}' has an invalid 'frame_rate_hz' "
                f"({raw!r}) - assuming {DEFAULT_CAMERA_FRAME_RATE_HZ} Hz for the "
                f"TSDF decay normalisation."
            )
            return DEFAULT_CAMERA_FRAME_RATE_HZ
        return rate

    def get_camera_context(self):
        """
        Get the camera context.

        Returns:
            CameraContext instance or None if no cameras configured
        """
        return self.camera_context
