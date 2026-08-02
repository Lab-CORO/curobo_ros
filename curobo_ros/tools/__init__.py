"""Outils en ligne de commande livrés avec le paquet.

Contrairement aux nodes de ``curobo_ros/core``, ce sont des utilitaires ponctuels
qu'on lance à la main contre un planner déjà en route (``ros2 run curobo_ros <outil>``).
Ils ne dépendent que de rclpy et des interfaces curobo_msgs.

Les outils d'analyse hors-ligne (matplotlib/pandas, pas de ROS) restent dans
``scripts/`` et ne sont pas installés — voir ``scripts/plot_mpc_diag.py``.
"""
