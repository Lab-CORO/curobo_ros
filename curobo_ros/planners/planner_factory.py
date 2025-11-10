#!/usr/bin/env python3
"""
Factory for creating trajectory planners.

Provides a centralized way to instantiate different planner types
and manage their lifecycle.
"""

from typing import Dict, Optional
from .trajectory_planner import TrajectoryPlanner
from .classic_planner import ClassicPlanner
from .mpc_planner import MPCPlanner


class PlannerFactory:
    """
    Factory for creating and managing trajectory planners.

    Supports dynamic planner selection and provides a registry of
    available planner types.
    """

    # Registry of available planner types
    _PLANNER_REGISTRY = {
        'classic': ClassicPlanner,
        'motion_gen': ClassicPlanner,  # Alias
        'mpc': MPCPlanner,
        'model_predictive_control': MPCPlanner,  # Alias
        # Future planners can be added here:
        # 'batch': BatchPlanner,
        # 'constrained': ConstrainedPlanner,
    }

    @classmethod
    def create_planner(cls, planner_type: str, node, config_wrapper) -> Optional[TrajectoryPlanner]:
        """
        Create a planner instance.

        Args:
            planner_type: Type of planner ('classic', 'mpc', etc.)
            node: ROS2 node
            config_wrapper: Configuration wrapper

        Returns:
            Instantiated planner or None if type not found

        Raises:
            ValueError: If planner_type is not recognized
        """
        planner_type = planner_type.lower().strip()

        if planner_type not in cls._PLANNER_REGISTRY:
            available = ', '.join(cls.get_available_planners())
            raise ValueError(
                f"Unknown planner type: '{planner_type}'. "
                f"Available planners: {available}"
            )

        planner_class = cls._PLANNER_REGISTRY[planner_type]
        planner = planner_class(node, config_wrapper)

        node.get_logger().info(
            f"Created planner: {planner.get_planner_name()} "
            f"(mode: {planner.get_execution_mode().value})"
        )

        return planner

    @classmethod
    def get_available_planners(cls) -> list:
        """
        Get list of available planner types.

        Returns:
            List of planner type strings
        """
        # Return only primary names (not aliases)
        return ['classic', 'mpc']

    @classmethod
    def register_planner(cls, name: str, planner_class: type):
        """
        Register a custom planner type.

        Allows extending the factory with user-defined planners.

        Args:
            name: Unique name for the planner type
            planner_class: Class inheriting from TrajectoryPlanner

        Raises:
            ValueError: If name already registered
        """
        if name in cls._PLANNER_REGISTRY:
            raise ValueError(f"Planner '{name}' already registered")

        if not issubclass(planner_class, TrajectoryPlanner):
            raise TypeError(
                f"Planner class must inherit from TrajectoryPlanner, "
                f"got {planner_class}"
            )

        cls._PLANNER_REGISTRY[name] = planner_class

    @classmethod
    def get_planner_info(cls, planner_type: str) -> Dict:
        """
        Get information about a planner type.

        Args:
            planner_type: Type of planner

        Returns:
            Dictionary with planner metadata
        """
        if planner_type not in cls._PLANNER_REGISTRY:
            return None

        planner_class = cls._PLANNER_REGISTRY[planner_type]

        # Create a temporary instance to get info
        # (without full initialization)
        info = {
            'name': planner_type,
            'class': planner_class.__name__,
            'module': planner_class.__module__,
        }

        return info


class PlannerManager:
    """
    Manages multiple planner instances and switching between them.

    Useful for nodes that need to support dynamic planner switching.
    """

    def __init__(self, node, config_wrapper):
        """
        Initialize planner manager.

        Args:
            node: ROS2 node
            config_wrapper: Configuration wrapper
        """
        self.node = node
        self.config_wrapper = config_wrapper
        self._planners: Dict[str, TrajectoryPlanner] = {}
        self._current_planner: Optional[TrajectoryPlanner] = None
        self._current_planner_type: Optional[str] = None

    def get_planner(self, planner_type: str) -> TrajectoryPlanner:
        """
        Get a planner instance, creating it if needed.

        Uses caching to avoid recreating planners.

        Args:
            planner_type: Type of planner

        Returns:
            Planner instance
        """
        if planner_type not in self._planners:
            self._planners[planner_type] = PlannerFactory.create_planner(
                planner_type, self.node, self.config_wrapper
            )

        return self._planners[planner_type]

    def set_current_planner(self, planner_type: str):
        """
        Set the active planner.

        Args:
            planner_type: Type of planner to activate
        """
        self._current_planner = self.get_planner(planner_type)
        self._current_planner_type = planner_type

        self.node.get_logger().info(
            f"Switched to planner: {self._current_planner.get_planner_name()}"
        )

    def get_current_planner(self) -> Optional[TrajectoryPlanner]:
        """
        Get the currently active planner.

        Returns:
            Current planner or None if not set
        """
        return self._current_planner

    def get_current_planner_type(self) -> Optional[str]:
        """
        Get the type of the current planner.

        Returns:
            Planner type string or None
        """
        return self._current_planner_type
