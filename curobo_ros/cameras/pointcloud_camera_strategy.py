#!/usr/bin/env python3

from curobo_ros.cameras.camera_strategy import CameraStrategy
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.geom.types import WorldConfig, BloxMap, VoxelGrid


from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from std_msgs.msg import Header
import ros2_numpy
import torch
import numpy as np
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2
import numpy as np
import torch
import ros2_numpy as rnp
from curobo.geom.types import WorldConfig
from scipy.ndimage import distance_transform_edt



class PointCloudCameraStrategy(CameraStrategy):
    """
    Strategy that converts point cloud data to orthographic depth images for cuRobo BLOX.

    This strategy subscribes to a PointCloud2 topic (typically the masked point cloud
    from robot_segmentation) and converts it to an orthographic depth image viewed from above.

    The orthographic projection is ideal for point clouds that are not associated with a
    specific camera viewpoint - it treats the point cloud as a top-down heightmap where
    depth represents the Z-coordinate of points.
    """

    def __init__(self, node, name, topic='', camera_info=[], frame_id='', pixel_size=0.01, bounds=None,
                 extrinsic_matrix=None, intrinsic_matrix=None):
        """
        Initialize the PointCloud to orthographic depth image strategy.

        Args:
            node: ROS2 node for creating subscriptions and logging
            topic: Topic name for the point cloud subscription
            camera_info: Not used for point cloud (kept for interface compatibility)
            frame_id: Frame ID for the camera (optional if extrinsic_matrix provided)
            pixel_size: Size of each pixel in meters (default: 1cm for good resolution)
            bounds: Workspace bounds as [min_x, max_x, min_y, max_y, min_z, max_z]
                   If None, uses default [-1.5, 1.5, -1.5, 1.5, -1.5, 1.5]
            extrinsic_matrix: Camera extrinsic matrix [4x4] as fallback for TF
            intrinsic_matrix: Not used for point cloud (kept for interface compatibility)
        """
        super().__init__(node, name, topic, camera_info, frame_id,
                         extrinsic_matrix=extrinsic_matrix, intrinsic_matrix=intrinsic_matrix)

        # Paramètres
        # self.node.declare_parameter('voxel_size', 0.02)
        self.node.declare_parameter('grid_size', [102, 102, 102])  # 2.02m x 2.02m x 2.02m (102 pour correspondre au cache cuRobo)
        self.node.declare_parameter('origin', [-1.0, -1.0, -1.0])
        self.node.declare_parameter('use_gpu', False)  # GPU ou CPU

        voxel_size = 0.02 #self.node.get_parameter('voxel_size').value
        grid_size = tuple(self.node.get_parameter('grid_size').value)
        origin = tuple(self.node.get_parameter('origin').value)
        use_gpu = self.node.get_parameter('use_gpu').value
        
        # Choisir builder (GPU ou CPU)
        if use_gpu:
            self.builder = FastVoxelGridBuilderGPU(
                voxel_size=voxel_size,
                grid_size=grid_size,
                origin=origin
            )
            self.node.get_logger().info('🚀 Mode GPU activé')
        else:
            self.builder = FastVoxelGridBuilder(
                voxel_size=voxel_size,
                grid_size=grid_size,
                origin=origin
            )
            self.node.get_logger().info('🚀 Mode CPU vectorisé activé')

        self.current_voxelgrid = None

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber
        self.subscription = self.node.create_subscription(
            PointCloud2,
            topic,
            self.pointcloud_callback,
            qos
        )


        # TODO Get the camera pose from frame_id
        self.camera_pose = Pose(
                        position=self.tensor_args.to_device([0, 0, 0]),
                        quaternion=self.tensor_args.to_device([1, 0, 0, 0]),
                    )


        
        self.node.get_logger().info(f'✅ Nœud démarré')
        self.node.get_logger().info(f'   Topic: {topic}')
        self.node.get_logger().info(f'   Grille fixe: {grid_size}')



    def pointcloud_callback(self, msg: PointCloud2):
        """
        Callback for receiving point cloud data.

        Args:
            msg: PointCloud2 message
        """
      
        t_start = time.perf_counter()
        
        try:
            # 1. Conversion ROS2→NumPy (rapide avec ros2_numpy)
            cloud_array = rnp.numpify(msg)
            
            # 2. Extraction XYZ vectorisée (pas de boucle)
            points = np.stack([
                cloud_array['x'],
                cloud_array['y'],
                cloud_array['z']
            ], axis=1).astype(np.float32)
            
            # 3. Filtrage NaN vectorisé
            valid_mask = np.all(np.isfinite(points), axis=1)
            points_clean = points[valid_mask]
            
            if len(points_clean) == 0:
                self.node.get_logger().warn('Point cloud vide')
                return
            
            # 4. Créer VoxelGrid (100% vectorisé)
            self.current_voxelgrid = self.builder.pointcloud_to_voxelgrid(points_clean)

            # Update world config
            self.update_world_config(self.current_voxelgrid, self.name)

            t_end = time.perf_counter()
            elapsed_ms = (t_end - t_start) * 1000
            fps = 1000.0 / elapsed_ms
            
            self.node.get_logger().info(
                f'{elapsed_ms:.1f}ms ({fps:.0f} Hz)',
                throttle_duration_sec=2.0
            )
            
            
        except Exception as e:
            self.node.get_logger().error(f'Erreur: {e}')
     
    def update_world_config(self, voxelgrid_dict, name):
        """
        Update world configuration with a VoxelGrid object.
        Converts the dictionary from FastVoxelGridBuilder to a proper VoxelGrid.

        Args:
            voxelgrid_dict: Dictionary containing voxel grid data
                    with keys: 'pose', 'dims', 'voxel_size', 'feature_tensor', 'feature_dtype'
            name: Name identifier for this voxel grid
        """
        try:
            # Calculer les dimensions physiques de la grille à partir du nombre de voxels
            grid_dims_voxels = voxelgrid_dict['dims']  # [nx, ny, nz] en nombre de voxels
            voxel_size = voxelgrid_dict['voxel_size']

            # Dimensions physiques en mètres
            # IMPORTANT: cuRobo ajoute +1 voxel via get_grid_shape()
            # Donc pour obtenir N voxels, il faut dims = (N-1) * voxel_size
            dims_meters = [
                (grid_dims_voxels[0] - 1) * voxel_size,
                (grid_dims_voxels[1] - 1) * voxel_size,
                (grid_dims_voxels[2] - 1) * voxel_size
            ]

            # Créer un VoxelGrid avec le feature_tensor ESDF
            voxel_grid = VoxelGrid(
                name=name,
                pose=voxelgrid_dict['pose'],
                dims=dims_meters,
                voxel_size=voxel_size,
                feature_tensor=voxelgrid_dict['feature_tensor'].flatten(),  # Flatten en 1D
                feature_dtype=voxelgrid_dict['feature_dtype']
            )

            # IMPORTANT: Créer le xyzr_tensor pour que get_occupied_voxels() fonctionne
            voxel_grid.xyzr_tensor = voxel_grid.create_xyzr_tensor(
                transform_to_origin=True,
                tensor_args=self.tensor_args
            )

            # Mettre à jour ou ajouter le voxel grid dans world_config
            updated = False
            for index, voxel in enumerate(self.node.world_model.world_model.voxel):
                if voxel.name == name:
                    self.node.world_model.world_model.voxel[index] = voxel_grid
                    updated = True
                    break

            # Si le voxel grid n'existe pas encore, l'ajouter
            if not updated:
                self.node.world_model.world_model.voxel.append(voxel_grid)

            # Recharger le modèle de collision avec la configuration mise à jour
            self.node.world_model.load_collision_model(self.node.world_model.world_model)

            # IMPORTANT: Mettre à jour les feature_tensor dans le cache de collision
            # load_collision_model() ne copie PAS automatiquement les features
            self.node.world_model.update_voxel_data(voxel_grid, env_idx=0)

            self.node.get_logger().info(
                f'✅ VoxelGrid "{name}" ajouté: dims={dims_meters}, voxel_size={voxel_size}',
                throttle_duration_sec=2.0
            )

        except Exception as e:
            self.node.get_logger().error(f'Erreur dans update_world_config: {e}')
            import traceback
            self.node.get_logger().error(traceback.format_exc())

        




class FastVoxelGridBuilder:
    """
    Constructeur VoxelGrid ultra-rapide avec grille fixe.
    Toutes les opérations sont vectorisées (pas de boucles Python).
    """
    
    def __init__(
        self,
        voxel_size: float = 0.02,
        grid_size: tuple = (100, 100, 100),  # Grille fixe 2m x 2m x 2m
        origin: tuple = (-1.0, -1.0, -1.0),   # Centré sur robot
        device: str = "cuda"
    ):
        self.voxel_size = voxel_size
        self.device = device
        self.grid_size = np.array(grid_size, dtype=np.int32)
        self.origin = np.array(origin, dtype=np.float32)
        
        # Pré-calculer les limites du volume
        self.volume_extent = self.grid_size * voxel_size
        self.volume_max = self.origin + self.volume_extent
        
        # Pré-allouer grille (réutilisée à chaque frame)
        self.occupancy_grid = np.zeros(grid_size, dtype=bool)
        
        # Pré-calculer facteur de conversion point→voxel
        self.inv_voxel_size = 1.0 / voxel_size

        print(f"✅ FastVoxelGrid initialisé:")
        print(f"   Grille: {grid_size} voxels")
        print(f"   Volume: {self.volume_extent} m")
        print(f"   Origine: {self.origin}")
        print(f"   Voxel: {voxel_size}m")
    
    def pointcloud_to_voxelgrid(self, points: np.ndarray):
        """
        Convertit point cloud → VoxelGrid (100% vectorisé, ZÉRO boucle).

        Args:
            points: Array (N, 3)

        Returns:
            dict VoxelGrid pour cuRobo
        """
        # 1. Vectorisation TOTALE : points → indices voxels (1 opération)
        voxel_indices = self._points_to_voxels_vectorized(points)

        # 2. Remplir grille par masque (pas de boucle)
        self._fill_grid_vectorized(voxel_indices)

        # DEBUG: Vérifier combien de voxels sont occupés
        num_occupied = np.sum(self.occupancy_grid)
        print(f"DEBUG: {len(points)} points → {len(voxel_indices)} voxels valides → {num_occupied} voxels occupés")

        # 3. ESDF (déjà vectorisé dans scipy)
        esdf_grid = self._compute_esdf_fast()

        # DEBUG: Vérifier les valeurs ESDF min/max
        print(f"DEBUG ESDF: min={esdf_grid.min():.4f}, max={esdf_grid.max():.4f}, "
              f"positifs={np.sum(esdf_grid > 0)}, négatifs={np.sum(esdf_grid < 0)}")

        # 4. GPU transfer (1 opération)
        esdf_tensor = torch.from_numpy(esdf_grid).float().to(self.device)

        # 5. VoxelGrid dict
        voxelgrid_dict = {
            'pose': [*self.origin.tolist(), 1, 0, 0, 0],
            'dims': self.grid_size.tolist(),
            'voxel_size': self.voxel_size,
            'feature_dtype': torch.float32,
            'feature_tensor': esdf_tensor,
        }

        return voxelgrid_dict
    
    def _points_to_voxels_vectorized(self, points: np.ndarray) -> np.ndarray:
        """
        Conversion vectorisée complète : (N, 3) → (M, 3) en 1 ligne.
        Pas de boucle, pas d'itération.
        """
        # Opération vectorisée pure : (N, 3) → (N, 3)
        voxel_indices = ((points - self.origin) * self.inv_voxel_size).astype(np.int32)
        
        # Filtrage par masque vectorisé (pas de boucle)
        valid_mask = (
            (voxel_indices[:, 0] >= 0) & (voxel_indices[:, 0] < self.grid_size[0]) &
            (voxel_indices[:, 1] >= 0) & (voxel_indices[:, 1] < self.grid_size[1]) &
            (voxel_indices[:, 2] >= 0) & (voxel_indices[:, 2] < self.grid_size[2])
        )
        
        return voxel_indices[valid_mask]
    
    def _fill_grid_vectorized(self, voxel_indices: np.ndarray):
        """
        Remplissage grille SANS boucle via indexing avancé NumPy.
        """
        # Réinitialiser grille (rapide car pré-allouée)
        self.occupancy_grid.fill(False)
        
        if len(voxel_indices) == 0:
            return
        
        # Indexation avancée : 1 opération vectorisée
        # Équivalent à grid[i, j, k] = True pour tous les indices
        self.occupancy_grid[
            voxel_indices[:, 0],
            voxel_indices[:, 1],
            voxel_indices[:, 2]
        ] = True
    
    def _compute_esdf_fast(self) -> np.ndarray:
        """
        ESDF optimisé (déjà vectorisé dans scipy).
        Convention ESDF cuRobo: positif = intérieur obstacle, négatif = extérieur (espace libre)

        Pour un point cloud (obstacles de surface), on donne:
        - Une valeur constante positive (ex: +voxel_size) aux voxels occupés
        - La distance négative à l'obstacle le plus proche pour l'espace libre
        """
        # Distance à l'obstacle le plus proche (pour l'espace libre)
        esdf_outside = distance_transform_edt(~self.occupancy_grid) * self.voxel_size

        # Pour les voxels occupés, donner une valeur constante positive
        # Utiliser voxel_size comme valeur par défaut (assez pour être détecté)
        esdf = np.where(self.occupancy_grid, self.voxel_size, -esdf_outside)

        return esdf.astype(np.float32)


class FastVoxelGridBuilderGPU:
    """
    Version 100% GPU avec PyTorch (encore plus rapide).
    Élimine même le CPU bottleneck du distance transform.
    """
    
    def __init__(
        self,
        voxel_size: float = 0.02,
        grid_size: tuple = (100, 100, 100),
        origin: tuple = (-1.0, -1.0, -1.0),
        device: str = "cuda"
    ):
        self.voxel_size = voxel_size
        self.device = torch.device(device)
        self.grid_size = torch.tensor(grid_size, dtype=torch.int32, device=self.device)
        self.origin = torch.tensor(origin, dtype=torch.float32, device=self.device)
        
        self.inv_voxel_size = 1.0 / voxel_size
        
        # Pré-allouer sur GPU
        self.occupancy_grid = torch.zeros(
            grid_size,
            dtype=torch.bool,
            device=self.device
        )

        print(f"✅ FastVoxelGridGPU initialisé:")
        print(f"   Device: {device}")
        print(f"   Grille: {grid_size} voxels")
    
    def pointcloud_to_voxelgrid(self, points: np.ndarray):
        """
        Pipeline 100% GPU (ultra-rapide).
        """
        # 1. CPU→GPU (1 transfert)
        points_gpu = torch.from_numpy(points).float().to(self.device)

        # 2. Conversion vectorisée GPU
        voxel_indices = self._points_to_voxels_gpu(points_gpu)

        # 3. Remplir grille GPU
        self._fill_grid_gpu(voxel_indices)

        # 4. ESDF GPU (approximatif mais très rapide)
        esdf_tensor = self._compute_esdf_gpu()

        # 5. VoxelGrid dict
        voxelgrid_dict = {
            'pose': [*self.origin.cpu().tolist(), 1, 0, 0, 0],
            'dims': self.grid_size.cpu().tolist(),
            'voxel_size': self.voxel_size,
            'feature_dtype': torch.float32,
            'feature_tensor': esdf_tensor,
        }

        return voxelgrid_dict
    
    def _points_to_voxels_gpu(self, points: torch.Tensor) -> torch.Tensor:
        """
        Conversion GPU pure (parallèle massif).
        """
        # Vectorisé GPU : (N, 3) → (N, 3)
        voxel_indices = ((points - self.origin) * self.inv_voxel_size).long()
        
        # Masque de validité (parallèle GPU)
        valid_mask = (
            (voxel_indices[:, 0] >= 0) & (voxel_indices[:, 0] < self.grid_size[0]) &
            (voxel_indices[:, 1] >= 0) & (voxel_indices[:, 1] < self.grid_size[1]) &
            (voxel_indices[:, 2] >= 0) & (voxel_indices[:, 2] < self.grid_size[2])
        )
        
        return voxel_indices[valid_mask]
    
    def _fill_grid_gpu(self, voxel_indices: torch.Tensor):
        """
        Remplissage grille GPU (scatter atomique).
        """
        # Reset
        self.occupancy_grid.fill_(False)
        
        if len(voxel_indices) == 0:
            return
        
        # Indexation GPU (optimisée par PyTorch)
        self.occupancy_grid[
            voxel_indices[:, 0],
            voxel_indices[:, 1],
            voxel_indices[:, 2]
        ] = True
    
    def _compute_esdf_gpu(self) -> torch.Tensor:
        """
        ESDF approximatif GPU via convolutions.
        Plus rapide mais moins précis que distance_transform_edt.
        """
        # Option 1 : Distance transform GPU approximatif (convolutions)
        # Plus rapide mais approximatif
        esdf = self._fast_distance_transform_gpu(self.occupancy_grid)
        
        return esdf
    
    def _fast_distance_transform_gpu(self, occupancy: torch.Tensor) -> torch.Tensor:
        """
        Distance transform GPU approximatif via max pooling itératif.
        Compromis vitesse/précision.
        Convention ESDF cuRobo: positif = intérieur obstacle, négatif = extérieur (espace libre)
        """
        import torch.nn.functional as F

        # Convertir bool → float
        grid = occupancy.float()

        # Distance extérieure (dilations successives)
        grid_inv = 1.0 - grid

        # Itérations de max pooling (simule propagation distance)
        # Plus d'itérations = plus précis mais plus lent
        for _ in range(5):
            grid_inv = F.max_pool3d(
                grid_inv.unsqueeze(0).unsqueeze(0),
                kernel_size=3,
                stride=1,
                padding=1
            ).squeeze()

        esdf_outside = grid_inv * self.voxel_size * 5  # Facteur d'échelle

        # Distance intérieure (érosions)
        for _ in range(5):
            grid = F.max_pool3d(
                grid.unsqueeze(0).unsqueeze(0),
                kernel_size=3,
                stride=1,
                padding=1
            ).squeeze()

        esdf_inside = grid * self.voxel_size * 5

        # IMPORTANT: Inverser le signe pour respecter la convention cuRobo
        # Positif à l'intérieur des obstacles, négatif à l'extérieur
        esdf = esdf_inside - esdf_outside

        return esdf




