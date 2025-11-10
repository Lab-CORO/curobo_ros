# Plan de Test d'Intégration - CuRobo ROS

## Table des Matières
1. [Vue d'ensemble](#vue-densemble)
2. [Inventaire des Interfaces ROS](#inventaire-des-interfaces-ros)
3. [Plan de Test par Interface](#plan-de-test-par-interface)
4. [Tests d'Intégration de Bout en Bout](#tests-dintégration-de-bout-en-bout)
5. [Stratégie de Test](#stratégie-de-test)

---

## Vue d'ensemble

Ce document recense toutes les interfaces ROS du projet `curobo_ros` et définit un plan de test complet pour valider leur fonctionnement.

### Architecture Générale
- **Package ROS 2**: curobo_ros (ament_python)
- **Nœuds principaux**: 10 nœuds ROS
- **Services**: 20+ services
- **Actions**: 1 action
- **Topics**: 15+ topics
- **Paramètres**: 25+ paramètres configurables

---

## Inventaire des Interfaces ROS

### 1. NŒUDS ROS

#### 1.1 CuRoboTrajectoryMaker (`curobo_gen_traj`)
**Fichier**: `curobo_ros/core/generate_trajectory.py`

**Description**: Nœud principal pour la génération de trajectoires avec cuRobo

**Interfaces exposées**:

**Services serveurs**:
- `/curobo_gen_traj/generate_trajectory` (TrajectoryGeneration)
- `/curobo_gen_traj/add_object` (AddObject)
- `/curobo_gen_traj/remove_object` (RemoveObject)
- `/curobo_gen_traj/get_obstacles` (Trigger)
- `/curobo_gen_traj/is_available` (Trigger)
- `/curobo_gen_traj/remove_all_objects` (Trigger)
- `/curobo_gen_traj/get_voxel_grid` (GetVoxelGrid)
- `/curobo_gen_traj/get_collision_distance` (GetCollisionDistance)
- `/curobo_gen_traj/update_motion_gen_config` (Trigger)
- `/curobo_gen_traj/set_robot_strategy` (Trigger)
- `/curobo_gen_traj/get_robot_strategy` (Trigger)

**Actions serveurs**:
- `/curobo_gen_traj/send_trajectrory` (SendTrajectory)

**Publishers**:
- `/curobo_gen_traj/collision_spheres` (visualization_msgs/MarkerArray)
- `visualization_marker_array` (visualization_msgs/MarkerArray)

**Paramètres**:
- `max_attempts` (int, default: 1)
- `timeout` (double, default: 5.0)
- `time_dilation_factor` (double, default: 0.5)
- `voxel_size` (double, default: 0.05)
- `collision_activation_distance` (double, default: 0.025)
- `robot_config_file` (string)
- `base_link` (string, default: 'base_0')
- `robot_type` (string, default: 'doosan_m1013')
- `cameras_config_file` (string)
- `use_pointcloud_camera` (bool, default: True)
- `pointcloud_topic` (string, default: '/masked_pointcloud')
- `pixel_size` (double, default: 0.01)

---

#### 1.2 IK (Inverse Kinematics) (`curobo_ik`)
**Fichier**: `curobo_ros/core/ik.py`

**Description**: Résolution de cinématique inverse

**Services serveurs**:
- `/curobo_ik/ik_pose` (Ik)
- `/curobo_ik/ik_batch_poses` (IkBatch)
- `/curobo_ik/add_object` (AddObject)
- `/curobo_ik/remove_object` (RemoveObject)
- `/curobo_ik/get_obstacles` (Trigger)
- `/curobo_ik/is_available` (Trigger)
- `/curobo_ik/remove_all_objects` (Trigger)
- `/curobo_ik/get_voxel_grid` (GetVoxelGrid)
- `/curobo_ik/get_collision_distance` (GetCollisionDistance)
- `/curobo_ik/update_motion_gen_config` (Trigger)

**Publishers**:
- `/curobo_ik/collision_spheres` (visualization_msgs/MarkerArray)

**Paramètres**:
- `voxel_size` (double, default: 0.5)
- `init_batch_size` (int, default: 1000)
- `base_link` (string)
- `robot_config_file` (string)
- `robot_type` (string)
- `cameras_config_file` (string)

---

#### 1.3 FK (Forward Kinematics) (`curobo_fk`)
**Fichier**: `curobo_ros/core/fk.py`

**Description**: Calcul de cinématique directe

**Services serveurs**:
- `/curobo/fk_poses` (Fk)

**Paramètres**:
- `robot_type` (string, default: 'ur10e')

---

#### 1.4 DepthMapRobotSegmentation (`robot_segmentation`)
**Fichier**: `curobo_ros/core/robot_segmentation.py`

**Description**: Segmentation du robot dans les images de profondeur

**Subscribers**:
- `/depth_to_rgb/image_raw` (sensor_msgs/Image)
- `/depth_to_rgb/camera_info` (sensor_msgs/CameraInfo)
- Joint states topic (configurable)

**Publishers**:
- `masked_depth_image` (sensor_msgs/Image)
- `collision_spheres` (visualization_msgs/MarkerArray)
- `robot_pointcloud_debug` (sensor_msgs/PointCloud2)

**Paramètres**:
- `robot_config_file` (string)
- `joint_states_topic` (string, default: '/dsr01/joint_states')
- `depth_image_topic` (string, default: '/depth_to_rgb/image_raw')
- `camera_info_topic` (string, default: '/depth_to_rgb/camera_info')
- `robot_base_frame` (string, default: 'base_link')
- `robot_type` (string)

---

#### 1.5 VoxelGridVisualizer (`viz_voxel_grid`)
**Fichier**: `curobo_ros/interfaces/show_voxel_grid.py`

**Description**: Visualisation de la grille voxel dans RViz

**Publishers**:
- `/visualise_voxel_grid` (visualization_msgs/Marker)

**Service clients**:
- Appelle `/curobo_gen_traj/get_voxel_grid`

---

#### 1.6 MarkerPublisher
**Fichier**: `curobo_ros/core/marker_publisher.py`

**Description**: Publication de marqueurs de visualisation

**Publishers**:
- `visualization_marker_array` (visualization_msgs/MarkerArray)
- `visualization_marker_voxel` (visualization_msgs/MarkerArray)

**Paramètres**:
- `base_link` (string, default: 'base_0')

---

#### 1.7 DoosanControl (Robot Strategy)
**Fichier**: `curobo_ros/robot/doosan_strategy.py`

**Description**: Contrôle du robot Doosan M1013

**Publishers**:
- `/leeloo/execute_trajectory` (trajectory_msgs/JointTrajectory)

**Subscribers**:
- `/leeloo/trajectory_state` (std_msgs/Float32)
- `/dsr01/joint_states` (sensor_msgs/JointState)

---

#### 1.8 EmulatorStrategy (Robot Strategy)
**Fichier**: `curobo_ros/robot/emulator_strategy.py`

**Description**: Émulation de robot pour tests sans hardware

**Publishers**:
- `/emulator/joint_states` (sensor_msgs/JointState)

---

#### 1.9 GhostStrategy (Robot Strategy)
**Fichier**: `curobo_ros/robot/ghost_strategy.py`

**Description**: Visualisation fantôme pour prévisualisation

**Publishers**:
- `trajectory` (trajectory_msgs/JointTrajectory)

---

#### 1.10 PointCloudCameraStrategy
**Fichier**: `curobo_ros/cameras/pointcloud_camera_strategy.py`

**Description**: Conversion de point cloud en image de profondeur

**Subscribers**:
- Point cloud topic (configurable) (sensor_msgs/PointCloud2)

**Paramètres**:
- `grid_size` (list, default: [102, 102, 102])
- `origin` (list, default: [-1.0, -1.0, -1.0])
- `use_gpu` (bool, default: False)

---

## Plan de Test par Interface

### CATÉGORIE 1: Services de Planification de Mouvement

#### Service: `/curobo_gen_traj/generate_trajectory`

**Tests Unitaires**:
1. **TEST_TRAJ_001**: Génération de trajectoire vers pose valide
   - **Setup**: Lancer curobo_gen_traj avec config par défaut
   - **Action**: Appeler service avec pose atteignable
   - **Validation**: success=True, trajectoire non vide
   - **Critère**: Temps de réponse < timeout configuré

2. **TEST_TRAJ_002**: Pose inatteignable
   - **Setup**: Lancer curobo_gen_traj
   - **Action**: Appeler service avec pose hors workspace
   - **Validation**: success=False, message d'erreur explicite
   - **Critère**: Pas de crash, réponse cohérente

3. **TEST_TRAJ_003**: Tentatives multiples (max_attempts)
   - **Setup**: Configurer max_attempts=5
   - **Action**: Poser requête avec pose difficile
   - **Validation**: Le nœud tente plusieurs fois avant échec
   - **Critère**: Respect du nombre de tentatives configuré

4. **TEST_TRAJ_004**: Timeout
   - **Setup**: Configurer timeout=2.0 secondes
   - **Action**: Poser requête complexe
   - **Validation**: Échec ou succès avant timeout
   - **Critère**: Temps de réponse ≤ timeout

5. **TEST_TRAJ_005**: Trajectoire avec obstacles
   - **Setup**: Ajouter obstacle via add_object
   - **Action**: Générer trajectoire qui nécessite contournement
   - **Validation**: Trajectoire évite l'obstacle
   - **Critère**: Distance min aux obstacles > collision_activation_distance

6. **TEST_TRAJ_006**: Pose identique à position actuelle
   - **Setup**: Obtenir pose actuelle du robot
   - **Action**: Demander trajectoire vers même pose
   - **Validation**: Trajectoire vide ou simple waypoint
   - **Critère**: Success=True, trajectoire triviale

**Tests d'Intégration**:
1. **TEST_TRAJ_INT_001**: Pipeline complet IK → Trajectory Gen → Execution
   - Chaîne complète de planification et exécution

2. **TEST_TRAJ_INT_002**: Génération avec perception en temps réel
   - Obstacles détectés par caméra influencent trajectoire

---

#### Service: `/curobo_ik/ik_pose`

**Tests Unitaires**:
1. **TEST_IK_001**: Solution IK pour pose valide
   - **Setup**: Lancer curobo_ik
   - **Action**: Requête avec pose atteignable
   - **Validation**: success=True, joint_states valides
   - **Critère**: Pose FK(joint_states) ≈ pose demandée (erreur < 1mm, 1°)

2. **TEST_IK_002**: Pose inatteignable IK
   - **Setup**: Lancer curobo_ik
   - **Action**: Pose hors espace de travail
   - **Validation**: success=False, message d'erreur
   - **Critère**: Pas de crash

3. **TEST_IK_003**: IK avec seed (position initiale)
   - **Setup**: Lancer IK avec position initiale donnée
   - **Action**: Calculer IK proche de seed
   - **Validation**: Solution proche du seed
   - **Critère**: Solution converge rapidement

4. **TEST_IK_004**: Singularités
   - **Setup**: Poser pose près de singularité
   - **Action**: Calculer IK
   - **Validation**: Gestion propre (échec gracieux ou solution stable)
   - **Critère**: Pas de valeurs articulaires aberrantes

**Tests d'Intégration**:
1. **TEST_IK_INT_001**: IK → FK round-trip
   - Pose → IK → joint_states → FK → pose' ≈ pose

2. **TEST_IK_INT_002**: IK avec obstacles
   - IK doit éviter collisions si world model présent

---

#### Service: `/curobo_ik/ik_batch_poses`

**Tests Unitaires**:
1. **TEST_IK_BATCH_001**: Batch de poses valides
   - **Setup**: Liste de 10 poses atteignables
   - **Action**: Appel ik_batch_poses
   - **Validation**: 10 solutions valides
   - **Critère**: Temps < temps_serial * speedup_factor

2. **TEST_IK_BATCH_002**: Batch mixte (valide + invalide)
   - **Setup**: Mix de poses valides et invalides
   - **Action**: Batch IK
   - **Validation**: Flags success appropriés pour chaque pose
   - **Critère**: Solutions partielles OK

3. **TEST_IK_BATCH_003**: Batch vide
   - **Setup**: Liste vide de poses
   - **Action**: Batch IK
   - **Validation**: Réponse vide sans crash
   - **Critère**: Pas d'erreur

4. **TEST_IK_BATCH_004**: Grand batch (stress test)
   - **Setup**: 1000 poses
   - **Action**: Batch IK
   - **Validation**: Toutes traitées
   - **Critère**: Performance acceptable (GPU acceleration)

---

#### Service: `/curobo/fk_poses`

**Tests Unitaires**:
1. **TEST_FK_001**: FK pour configuration valide
   - **Setup**: Joint states valides
   - **Action**: Calculer FK
   - **Validation**: Pose retournée cohérente
   - **Critère**: Pas de NaN, pose dans workspace

2. **TEST_FK_002**: FK pour limites articulaires
   - **Setup**: Joint à limite min/max
   - **Action**: FK
   - **Validation**: Pose calculée
   - **Critère**: Pas d'erreur

3. **TEST_FK_003**: FK batch
   - **Setup**: Multiple configurations
   - **Action**: FK batch
   - **Validation**: Liste de poses
   - **Critère**: Taille input = taille output

4. **TEST_FK_004**: FK configuration nulle
   - **Setup**: Tous joints à 0
   - **Action**: FK
   - **Validation**: Pose par défaut cohérente
   - **Critère**: Position connue vérifiable

---

### CATÉGORIE 2: Services de Gestion de l'Environnement

#### Service: `/curobo_gen_traj/add_object`

**Tests Unitaires**:
1. **TEST_OBJ_001**: Ajouter cube
   - **Setup**: Nœud initialisé
   - **Action**: Ajouter cube 0.1x0.1x0.1 à [0.5, 0, 0.3]
   - **Validation**: success=True, objet dans liste obstacles
   - **Critère**: get_obstacles retourne l'objet

2. **TEST_OBJ_002**: Ajouter sphère
   - **Action**: Ajouter sphère rayon 0.05
   - **Validation**: Objet ajouté
   - **Critère**: Type géométrique correct

3. **TEST_OBJ_003**: Ajouter mesh complexe
   - **Action**: Ajouter mesh depuis fichier STL/OBJ
   - **Validation**: Mesh chargé
   - **Critère**: Pas d'erreur de parsing

4. **TEST_OBJ_004**: Ajouter objet avec même nom (remplacement)
   - **Setup**: Ajouter "box1"
   - **Action**: Ajouter nouveau "box1"
   - **Validation**: Ancien remplacé
   - **Critère**: Une seule instance de "box1"

5. **TEST_OBJ_005**: Ajouter multiples objets
   - **Action**: Ajouter 100 objets
   - **Validation**: Tous présents
   - **Critère**: Performance acceptable

6. **TEST_OBJ_006**: Objets avec transformations
   - **Action**: Ajouter objet avec rotation/translation
   - **Validation**: Pose correcte dans monde
   - **Critère**: Transformation appliquée correctement

---

#### Service: `/curobo_gen_traj/remove_object`

**Tests Unitaires**:
1. **TEST_RM_001**: Supprimer objet existant
   - **Setup**: Ajouter objet "test_box"
   - **Action**: Supprimer "test_box"
   - **Validation**: success=True, objet absent de get_obstacles
   - **Critère**: Objet supprimé

2. **TEST_RM_002**: Supprimer objet inexistant
   - **Action**: Supprimer "nonexistent"
   - **Validation**: Gestion propre (success=False ou warning)
   - **Critère**: Pas de crash

3. **TEST_RM_003**: Supprimer après utilisation en planification
   - **Setup**: Objet utilisé dans trajectoire
   - **Action**: Supprimer objet
   - **Validation**: Planification suivante ne voit plus objet
   - **Critère**: Environnement mis à jour

---

#### Service: `/curobo_gen_traj/remove_all_objects`

**Tests Unitaires**:
1. **TEST_RM_ALL_001**: Vider monde avec objets
   - **Setup**: Ajouter 10 objets
   - **Action**: remove_all_objects
   - **Validation**: get_obstacles retourne liste vide
   - **Critère**: Tous objets supprimés

2. **TEST_RM_ALL_002**: Vider monde déjà vide
   - **Setup**: Monde vide
   - **Action**: remove_all_objects
   - **Validation**: Pas d'erreur
   - **Critère**: Opération idempotente

---

#### Service: `/curobo_gen_traj/get_obstacles`

**Tests Unitaires**:
1. **TEST_GET_OBS_001**: Liste d'obstacles vide
   - **Setup**: Nouveau nœud
   - **Action**: get_obstacles
   - **Validation**: Liste vide
   - **Critère**: Format correct

2. **TEST_GET_OBS_002**: Liste avec objets
   - **Setup**: Ajouter 5 objets
   - **Action**: get_obstacles
   - **Validation**: 5 objets retournés avec détails (nom, type, pose)
   - **Critère**: Toutes infos présentes

---

#### Service: `/curobo_gen_traj/get_voxel_grid`

**Tests Unitaires**:
1. **TEST_VOXEL_001**: Grille vide
   - **Setup**: Monde vide
   - **Action**: get_voxel_grid
   - **Validation**: Grille de taille cohérente, occupancy faible
   - **Critère**: Structure correcte

2. **TEST_VOXEL_002**: Grille avec obstacles
   - **Setup**: Ajouter objets
   - **Action**: get_voxel_grid
   - **Validation**: Voxels occupés correspondent aux objets
   - **Critère**: Occupancy augmentée

3. **TEST_VOXEL_003**: Résolution voxel_size
   - **Setup**: Configurer voxel_size=0.01 vs 0.1
   - **Action**: Obtenir grilles
   - **Validation**: Tailles différentes
   - **Critère**: Résolution respectée

---

#### Service: `/curobo_gen_traj/get_collision_distance`

**Tests Unitaires**:
1. **TEST_COLL_DIST_001**: Distance sans obstacles
   - **Setup**: Monde vide
   - **Action**: Requête distance pour config donnée
   - **Validation**: Distance = infini ou valeur max
   - **Critère**: Pas de collision

2. **TEST_COLL_DIST_002**: Distance avec obstacles proches
   - **Setup**: Objet à 5cm du robot
   - **Action**: get_collision_distance
   - **Validation**: Distance ≈ 0.05m
   - **Critère**: Précision < 1cm

3. **TEST_COLL_DIST_003**: Configuration en collision
   - **Setup**: Objet intersectant robot
   - **Action**: Distance
   - **Validation**: Distance négative ou 0
   - **Critère**: Collision détectée

4. **TEST_COLL_DIST_004**: Distance le long de trajectoire
   - **Setup**: Trajectoire et obstacles
   - **Action**: Distance pour chaque waypoint
   - **Validation**: Distances cohérentes
   - **Critère**: Distance min respectée

---

### CATÉGORIE 3: Services de Configuration et Status

#### Service: `/curobo_gen_traj/is_available`

**Tests Unitaires**:
1. **TEST_AVAIL_001**: Nœud prêt
   - **Setup**: Démarrage complet du nœud
   - **Action**: is_available
   - **Validation**: success=True
   - **Critère**: Prêt à recevoir requêtes

2. **TEST_AVAIL_002**: Nœud en initialisation
   - **Setup**: Interroger pendant startup
   - **Action**: is_available
   - **Validation**: success=False
   - **Critère**: Status clair

3. **TEST_AVAIL_003**: Appels répétés
   - **Action**: Appeler 10x is_available
   - **Validation**: Réponse cohérente
   - **Critère**: Pas de dégradation

---

#### Service: `/curobo_gen_traj/update_motion_gen_config`

**Tests Unitaires**:
1. **TEST_UPDATE_001**: Recharger config valide
   - **Setup**: Modifier fichier config
   - **Action**: update_motion_gen_config
   - **Validation**: Config rechargée
   - **Critère**: Nouveaux paramètres actifs

2. **TEST_UPDATE_002**: Config invalide
   - **Setup**: Fichier config corrompu
   - **Action**: Update config
   - **Validation**: Échec gracieux, config précédente conservée
   - **Critère**: Nœud reste fonctionnel

---

#### Service: `{node}/set_robot_strategy`

**Tests Unitaires**:
1. **TEST_STRAT_001**: Changer vers "emulator"
   - **Setup**: Nœud avec robot réel
   - **Action**: set_robot_strategy("emulator")
   - **Validation**: Stratégie changée
   - **Critère**: get_robot_strategy retourne "emulator"

2. **TEST_STRAT_002**: Changer vers "ghost"
   - **Action**: Changer vers ghost
   - **Validation**: Publications sur topic "trajectory"
   - **Critère**: Visualisation active

3. **TEST_STRAT_003**: Changer vers stratégie invalide
   - **Action**: set_robot_strategy("invalid")
   - **Validation**: Échec avec message d'erreur
   - **Critère**: Stratégie inchangée

4. **TEST_STRAT_004**: Changer pendant exécution
   - **Setup**: Trajectoire en cours
   - **Action**: Changer stratégie
   - **Validation**: Changement après arrêt ou queue
   - **Critère**: Pas de comportement dangereux

---

#### Service: `{node}/get_robot_strategy`

**Tests Unitaires**:
1. **TEST_GET_STRAT_001**: Obtenir stratégie par défaut
   - **Setup**: Nœud démarré avec config par défaut
   - **Action**: get_robot_strategy
   - **Validation**: Retourne "doosan_m1013" (ou default)
   - **Critère**: Cohérent avec config

---

### CATÉGORIE 4: Action Serveurs

#### Action: `/curobo_gen_traj/send_trajectrory`

**Tests Unitaires**:
1. **TEST_ACTION_001**: Exécution trajectoire complète
   - **Setup**: Trajectoire valide générée
   - **Action**: send_trajectrory avec goal
   - **Validation**: Feedback progression 0→1, result success=True
   - **Critère**: Trajectoire exécutée jusqu'au bout

2. **TEST_ACTION_002**: Annulation (cancel) pendant exécution
   - **Setup**: Démarrer action avec trajectoire longue
   - **Action**: Cancel après 50% progression
   - **Validation**: Action annulée proprement
   - **Critère**: Robot s'arrête, result canceled

3. **TEST_ACTION_003**: Feedback temps réel
   - **Setup**: Exécuter trajectoire
   - **Action**: Monitor feedback
   - **Validation**: step_progression augmente de 0 à 1
   - **Critère**: Updates réguliers (>1 Hz)

4. **TEST_ACTION_004**: Trajectoire vide
   - **Setup**: Goal avec trajectoire vide
   - **Action**: send_trajectrory
   - **Validation**: Échec immédiat ou success immédiat
   - **Critère**: Pas de blocage

5. **TEST_ACTION_005**: Multiples requêtes concurrentes
   - **Setup**: Envoyer 2 goals simultanément
   - **Action**: Traiter queue
   - **Validation**: Une seule active, autres queued ou rejected
   - **Critère**: Pas de conflit

6. **TEST_ACTION_006**: Échec pendant exécution
   - **Setup**: Simuler erreur robot (perte comm)
   - **Action**: Action en cours
   - **Validation**: result success=False, message erreur
   - **Critère**: Échec propre

**Tests d'Intégration**:
1. **TEST_ACTION_INT_001**: Pipeline génération + exécution
   - generate_trajectory puis send_trajectrory immédiatement

2. **TEST_ACTION_INT_002**: Exécution avec stratégies différentes
   - Tester avec emulator, ghost, doosan

---

### CATÉGORIE 5: Topics Publishers

#### Publisher: `visualization_marker_array`

**Tests Unitaires**:
1. **TEST_VIZ_001**: Publication marqueurs waypoints
   - **Setup**: Générer trajectoire
   - **Action**: Écouter topic
   - **Validation**: MarkerArray reçu avec waypoints
   - **Critère**: Nombre de markers = nombre de waypoints

2. **TEST_VIZ_002**: Marqueurs dans bon référentiel (frame)
   - **Action**: Vérifier frame_id
   - **Validation**: frame_id = base_link configuré
   - **Critère**: Cohérence TF

3. **TEST_VIZ_003**: Mise à jour après nouvelle trajectoire
   - **Setup**: Trajectoire A puis B
   - **Action**: Observer publications
   - **Validation**: Markers mis à jour
   - **Critère**: Pas de marqueurs fantômes

---

#### Publisher: `collision_spheres`

**Tests Unitaires**:
1. **TEST_SPHERES_001**: Publication sphères de collision
   - **Setup**: Robot en mouvement
   - **Action**: Écouter collision_spheres
   - **Validation**: MarkerArray avec sphères du robot
   - **Critère**: Nombre et positions cohérents avec URDF

2. **TEST_SPHERES_002**: Mise à jour dynamique
   - **Setup**: Robot change de configuration
   - **Action**: Observer sphères
   - **Validation**: Positions mises à jour
   - **Critère**: Synchronisation avec joint_states

---

#### Publisher: `masked_depth_image` (DepthMapRobotSegmentation)

**Tests Unitaires**:
1. **TEST_DEPTH_001**: Publication image maskée
   - **Setup**: Flux depth + joint_states actifs
   - **Action**: Écouter masked_depth_image
   - **Validation**: Image publiée avec robot masqué
   - **Critère**: Fréquence > 5 Hz

2. **TEST_DEPTH_002**: Qualité masquage
   - **Setup**: Image depth avec robot visible
   - **Action**: Observer image maskée
   - **Validation**: Pixels robot = 0 ou valeur invalide
   - **Critère**: Robot correctement segmenté

3. **TEST_DEPTH_003**: Latence
   - **Setup**: Timestamp image depth
   - **Action**: Timestamp image maskée
   - **Validation**: Latence < 100ms
   - **Critère**: Temps réel

---

#### Publisher: `/emulator/joint_states` (EmulatorStrategy)

**Tests Unitaires**:
1. **TEST_EMUL_001**: Publication états joints émulés
   - **Setup**: Mode emulator avec trajectoire
   - **Action**: Écouter joint_states
   - **Validation**: États publiés, suivent trajectoire
   - **Critère**: Fréquence 50+ Hz

2. **TEST_EMUL_002**: Limites articulaires respectées
   - **Setup**: Trajectoire avec limites
   - **Action**: Monitor joint_states
   - **Validation**: Valeurs dans limites URDF
   - **Critère**: Pas de dépassement

---

#### Publisher: `/leeloo/execute_trajectory` (DoosanControl)

**Tests Unitaires**:
1. **TEST_DOOSAN_001**: Publication trajectoire robot réel
   - **Setup**: Mode doosan avec trajectoire
   - **Action**: Observer /leeloo/execute_trajectory
   - **Validation**: JointTrajectory publié
   - **Critère**: Format correct pour contrôleur Doosan

2. **TEST_DOOSAN_002**: Respect time_dilation_factor
   - **Setup**: time_dilation_factor=0.5
   - **Action**: Vérifier timestamps trajectoire
   - **Validation**: Durée × 2
   - **Critère**: Facteur appliqué

---

### CATÉGORIE 6: Topics Subscribers

#### Subscriber: `/depth_to_rgb/image_raw`

**Tests Unitaires**:
1. **TEST_SUB_DEPTH_001**: Réception images depth
   - **Setup**: Publier images test
   - **Action**: Observer traitement par nœud
   - **Validation**: Images reçues et traitées
   - **Critère**: Pas de drop de messages

2. **TEST_SUB_DEPTH_002**: Gestion fréquence variable
   - **Setup**: Publier à 5 Hz puis 30 Hz
   - **Action**: Observer comportement
   - **Validation**: Adaptation à la fréquence
   - **Critère**: Pas de surcharge queue

---

#### Subscriber: `/leeloo/trajectory_state`

**Tests Unitaires**:
1. **TEST_TRAJ_STATE_001**: Réception progression
   - **Setup**: Robot en exécution
   - **Action**: Publier progression 0.0 → 1.0
   - **Validation**: Action feedback mis à jour
   - **Critère**: Synchronisation feedback action

2. **TEST_TRAJ_STATE_002**: Détection fin trajectoire
   - **Setup**: progression = 1.0
   - **Action**: Observer action result
   - **Validation**: Action complétée
   - **Critère**: Result envoyé

---

#### Subscriber: `/dsr01/joint_states`

**Tests Unitaires**:
1. **TEST_JOINT_SUB_001**: Réception états joints
   - **Setup**: Publier joint_states
   - **Action**: Vérifier utilisation par IK/FK
   - **Validation**: États à jour dans contexte robot
   - **Critère**: Latence < 50ms

---

### CATÉGORIE 7: Paramètres ROS

#### Paramètre: `robot_type`

**Tests de Configuration**:
1. **TEST_PARAM_001**: Valeur par défaut
   - **Setup**: Démarrer sans override
   - **Validation**: robot_type = "doosan_m1013"
   - **Critère**: Default correct

2. **TEST_PARAM_002**: Override à launch
   - **Setup**: Launch avec robot_type:="emulator"
   - **Validation**: Mode emulator actif
   - **Critère**: Paramètre pris en compte

3. **TEST_PARAM_003**: Valeur invalide
   - **Setup**: robot_type:="invalid_robot"
   - **Validation**: Erreur ou fallback
   - **Critère**: Pas de crash

---

#### Paramètre: `max_attempts`

**Tests de Configuration**:
1. **TEST_MAX_ATT_001**: Valeur par défaut (1)
   - **Validation**: Une seule tentative par génération

2. **TEST_MAX_ATT_002**: Augmenter tentatives
   - **Setup**: max_attempts:=10
   - **Action**: Requête difficile
   - **Validation**: Jusqu'à 10 tentatives
   - **Critère**: Logs montrent tentatives multiples

---

#### Paramètre: `timeout`

**Tests de Configuration**:
1. **TEST_TIMEOUT_001**: Timeout court (1.0s)
   - **Setup**: timeout:=1.0
   - **Action**: Requête complexe
   - **Validation**: Réponse avant 1s (échec ou succès)
   - **Critère**: Respect timeout

---

#### Paramètre: `voxel_size`

**Tests de Configuration**:
1. **TEST_VOXEL_SIZE_001**: Voxel fin (0.01)
   - **Setup**: voxel_size:=0.01
   - **Action**: get_voxel_grid
   - **Validation**: Grille haute résolution
   - **Critère**: Précision collision accrue

2. **TEST_VOXEL_SIZE_002**: Voxel grossier (0.1)
   - **Setup**: voxel_size:=0.1
   - **Action**: Génération trajectoire
   - **Validation**: Plus rapide, moins précis
   - **Critère**: Trade-off performance/précision

---

#### Paramètre: `collision_activation_distance`

**Tests de Configuration**:
1. **TEST_COLL_ACT_001**: Distance sécurité
   - **Setup**: collision_activation_distance:=0.05
   - **Action**: Générer trajectoire près obstacle
   - **Validation**: Marge ≥ 5cm
   - **Critère**: Sécurité respectée

---

## Tests d'Intégration de Bout en Bout

### SCENARIO 1: Pipeline Complet de Planification et Exécution

**TEST_E2E_001: Pick and Place Simple**

**Objectif**: Valider la chaîne complète depuis la demande de pose jusqu'à l'exécution

**Étapes**:
1. Démarrer tous les nœuds (curobo_gen_traj, curobo_ik, curobo_fk)
2. Vérifier disponibilité via is_available
3. Définir pose target (position pick)
4. Appeler /curobo_ik/ik_pose pour obtenir configuration
5. Appeler /curobo_gen_traj/generate_trajectory pour planifier
6. Exécuter via action /curobo_gen_traj/send_trajectrory
7. Vérifier arrivée à destination via joint_states
8. Répéter pour pose place

**Validations**:
- Chaque service répond success=True
- Trajectoire générée est lisse (jerk limité)
- Exécution complète sans erreur
- Position finale = position demandée (erreur < 5mm)

**Critères de Succès**:
- Temps total < 10s pour pick+place simple
- Aucune collision détectée
- Feedback action progresse de 0 à 1

---

### SCENARIO 2: Gestion d'Environnement Dynamique

**TEST_E2E_002: Ajout/Suppression d'Obstacles Dynamiques**

**Objectif**: Valider la mise à jour de l'environnement et replanification

**Étapes**:
1. Générer trajectoire vers pose A (succès attendu)
2. Ajouter obstacle sur le chemin via add_object
3. Regénérer trajectoire vers pose A
4. Vérifier contournement de l'obstacle
5. Supprimer obstacle via remove_object
6. Regénérer trajectoire (doit être différente, plus directe)

**Validations**:
- Trajectoire 2 évite obstacle (get_collision_distance > seuil)
- Trajectoire 3 est plus courte que trajectoire 2
- Voxel grid montre obstacle aux étapes 3-5

**Critères de Succès**:
- Détection obstacle immédiate
- Pas de collision dans simulation
- Performance temps réel (replanification < 1s)

---

### SCENARIO 3: Perception et Segmentation

**TEST_E2E_003: Pipeline Perception → Planification**

**Objectif**: Intégrer caméra depth, segmentation robot, et planification

**Étapes**:
1. Démarrer robot_segmentation avec caméra simulée/réelle
2. Vérifier publication masked_depth_image
3. Configurer curobo_gen_traj avec use_pointcloud_camera:=true
4. Placer obstacle physique devant robot
5. Observer voxel grid via get_voxel_grid
6. Générer trajectoire qui évite obstacle détecté

**Validations**:
- Obstacle physique apparaît dans voxel grid
- Robot segmenté correctement (collision_spheres cohérentes)
- Trajectoire évite obstacle perçu

**Critères de Succès**:
- Latence perception → planification < 200ms
- Précision détection obstacle > 90%
- Pas de faux positifs du robot comme obstacle

---

### SCENARIO 4: Stratégies Robot Multiples

**TEST_E2E_004: Changement de Stratégie à Chaud**

**Objectif**: Valider le switch entre emulator/ghost/robot réel

**Étapes**:
1. Démarrer en mode emulator
2. Générer et exécuter trajectoire (observer /emulator/joint_states)
3. Appeler set_robot_strategy("ghost")
4. Exécuter nouvelle trajectoire (observer topic trajectory)
5. Appeler set_robot_strategy("doosan_m1013")
6. (Si hardware disponible) exécuter sur robot réel

**Validations**:
- Chaque stratégie publie sur topics appropriés
- get_robot_strategy retourne stratégie active
- Pas de conflit entre stratégies

**Critères de Succès**:
- Switch sans redémarrage nœud
- Comportement cohérent entre stratégies
- Emulator = ghost timing-wise

---

### SCENARIO 5: Stress Test et Robustesse

**TEST_E2E_005: Requêtes Concurrentes et Haute Fréquence**

**Objectif**: Tester stabilité sous charge

**Étapes**:
1. Envoyer 100 requêtes ik_batch_poses simultanément
2. Pendant ce temps, ajouter/supprimer 50 objets
3. Générer 10 trajectoires en parallèle
4. Observer utilisation mémoire/CPU
5. Vérifier absence de crashes/deadlocks

**Validations**:
- Toutes requêtes reçoivent réponse
- Ordre cohérent (pas de race conditions)
- Pas de memory leaks

**Critères de Succès**:
- 95% requêtes répondent avant timeout
- Utilisation mémoire stable
- Pas de dégradation performance sur 10+ minutes

---

### SCENARIO 6: Récupération d'Erreurs

**TEST_E2E_006: Gestion de Pannes et Recovery**

**Objectif**: Valider robustesse aux erreurs

**Étapes**:
1. Simuler perte connexion robot pendant exécution
2. Vérifier action result=failure
3. Reconnecter et relancer
4. Simuler config file corrompu → update_motion_gen_config
5. Vérifier fallback sur config précédente
6. Envoyer requête invalide (NaN dans pose)
7. Vérifier rejection propre

**Validations**:
- Erreurs loggées clairement
- Nœuds restent actifs après erreur
- Pas de crash fatal

**Critères de Succès**:
- Recovery automatique quand possible
- Messages d'erreur informatifs
- Système utilisable après chaque erreur

---

## Stratégie de Test

### 1. Environnement de Test

**Configuration Minimale**:
- ROS 2 (Humble/Iron/Jazzy selon projet)
- cuRobo installé
- Fichiers de configuration robot (URDF, YAML)
- pytest + pytest-ros

**Matériel Optionnel**:
- Caméra RealSense (pour tests perception)
- Robot Doosan M1013 (pour tests hardware-in-loop)
- GPU CUDA (pour performance optimale)

---

### 2. Framework de Test

**Structure Proposée**:
```
curobo_ros/
├── test/
│   ├── unit/
│   │   ├── test_services_motion.py
│   │   ├── test_services_world.py
│   │   ├── test_publishers.py
│   │   ├── test_subscribers.py
│   │   └── test_parameters.py
│   ├── integration/
│   │   ├── test_e2e_planning.py
│   │   ├── test_e2e_perception.py
│   │   ├── test_e2e_strategies.py
│   │   └── test_e2e_robustness.py
│   └── fixtures/
│       ├── mock_robot.py
│       ├── test_poses.yaml
│       └── test_obstacles.yaml
```

**Outils Recommandés**:
- `pytest` pour structure de tests
- `launch_testing` pour tests d'intégration ROS 2
- `unittest.mock` pour mocks de hardware
- `ros2_test` pour validation topics/services
- `tf2_ros` pour validation transforms
- `rosbag2` pour tests replay

---

### 3. Automatisation CI/CD

**Pipeline Proposé**:

```yaml
# .github/workflows/ros_tests.yml
name: ROS Integration Tests

on: [push, pull_request]

jobs:
  unit_tests:
    runs-on: ubuntu-22.04
    container: ros:humble
    steps:
      - checkout code
      - install dependencies (cuRobo, curobo_msgs)
      - colcon build
      - colcon test (unit tests)
      - publish results

  integration_tests:
    runs-on: ubuntu-22.04
    needs: unit_tests
    steps:
      - run E2E scenarios (emulator only)
      - check test coverage > 80%

  hardware_tests:
    runs-on: [self-hosted, robot-lab]
    if: github.ref == 'refs/heads/main'
    steps:
      - run hardware-in-loop tests
      - safety checks
```

---

### 4. Couverture de Test

**Objectifs**:
- **Couverture Code**: >80% des lignes
- **Couverture Interfaces**: 100% services/actions/topics testés
- **Couverture Paramètres**: 100% paramètres validés

**Priorité**:
1. **Critique** (P0): Services core (IK, FK, generate_trajectory)
2. **Haute** (P1): Gestion environnement, actions
3. **Moyenne** (P2): Visualisation, stratégies robot
4. **Basse** (P3): Paramètres optionnels

---

### 5. Métriques de Performance

**À Mesurer**:
- Temps de réponse services (p50, p95, p99)
- Latence bout-en-bout perception → exécution
- Utilisation CPU/GPU/mémoire
- Fréquence publication topics
- Taux de succès planification
- Temps de startup nœuds

**Seuils**:
- IK single pose: <50ms (p95)
- IK batch 100 poses: <500ms (p95)
- Generate trajectory: <1s (p95)
- Perception latency: <100ms
- Action feedback rate: >10 Hz

---

### 6. Tests de Régression

**Déclencheurs**:
- Chaque commit sur branche principale
- Pull requests
- Release candidates

**Snapshots**:
- Capturer trajectoires de référence
- Comparer nouvelles trajectoires (distance euclidienne < seuil)
- Vérifier non-régression performance

---

### 7. Documentation des Tests

**Pour Chaque Test**:
- ID unique (TEST_XXX_NNN)
- Objectif clair
- Préconditions
- Étapes détaillées
- Critères de succès mesurables
- Données de test (poses, objets, configs)

**Maintenance**:
- Revue des tests à chaque release
- Mise à jour si interfaces changent
- Archivage des tests obsolètes

---

## Prochaines Étapes

### Phase 1: Implémentation Tests Unitaires (Semaine 1-2)
1. Créer structure de test (dossiers, fixtures)
2. Implémenter tests services critiques (IK, FK, generate_trajectory)
3. Mock des dépendances hardware

### Phase 2: Tests d'Intégration (Semaine 3-4)
1. Scénarios E2E avec emulator
2. Tests perception avec données enregistrées
3. Validation stratégies robot

### Phase 3: CI/CD (Semaine 5)
1. Configurer GitHub Actions
2. Automatiser exécution tests
3. Rapports de couverture

### Phase 4: Tests Hardware (Semaine 6+)
1. Tests avec robot réel (si disponible)
2. Tests caméra RealSense
3. Validation performance GPU

---

## Annexe: Checklist Validation

### Services
- [ ] generate_trajectory (6 tests unitaires, 2 intégration)
- [ ] ik_pose (4 tests)
- [ ] ik_batch_poses (4 tests)
- [ ] fk_poses (4 tests)
- [ ] add_object (6 tests)
- [ ] remove_object (3 tests)
- [ ] remove_all_objects (2 tests)
- [ ] get_obstacles (2 tests)
- [ ] get_voxel_grid (3 tests)
- [ ] get_collision_distance (4 tests)
- [ ] is_available (3 tests)
- [ ] update_motion_gen_config (2 tests)
- [ ] set_robot_strategy (4 tests)
- [ ] get_robot_strategy (1 test)

### Actions
- [ ] send_trajectrory (6 tests unitaires, 2 intégration)

### Topics Publishers
- [ ] visualization_marker_array (3 tests)
- [ ] collision_spheres (2 tests)
- [ ] masked_depth_image (3 tests)
- [ ] /emulator/joint_states (2 tests)
- [ ] /leeloo/execute_trajectory (2 tests)

### Topics Subscribers
- [ ] /depth_to_rgb/image_raw (2 tests)
- [ ] /leeloo/trajectory_state (2 tests)
- [ ] /dsr01/joint_states (1 test)

### Paramètres
- [ ] robot_type (3 tests)
- [ ] max_attempts (2 tests)
- [ ] timeout (1 test)
- [ ] voxel_size (2 tests)
- [ ] collision_activation_distance (1 test)

### Scénarios E2E
- [ ] Pipeline complet pick-place
- [ ] Environnement dynamique
- [ ] Perception → Planification
- [ ] Stratégies multiples
- [ ] Stress test
- [ ] Recovery erreurs

---

**Total Estimé**: ~100 tests unitaires + 6 scénarios E2E
**Effort Estimé**: 6-8 semaines (1 développeur)
**Couverture Cible**: >80% code, 100% interfaces

---

## Spécification des Rosbags pour Tests

### Vue d'ensemble

Les rosbags sont essentiels pour les tests reproductibles, en particulier pour:
- Tests de perception sans caméra physique
- Tests de régression (comparaison avant/après modifications)
- Tests sans robot réel (replay des états joints)
- Validation des performances sur données réelles
- CI/CD automatisé sans hardware

---

## Rosbags Requis par Catégorie

### 1. ROSBAG PERCEPTION - Caméra Depth

#### 1.1 Rosbag: `test_depth_static_scene.bag`

**Description**: Scène statique avec objets connus pour validation de la segmentation

**Topics requis**:
```
/depth_to_rgb/image_raw              [sensor_msgs/Image]
/depth_to_rgb/camera_info            [sensor_msgs/CameraInfo]
/dsr01/joint_states                  [sensor_msgs/JointState]
/tf                                   [tf2_msgs/TFMessage]
/tf_static                            [tf2_msgs/TFMessage]
```

**Spécifications**:
- **Durée**: 30 secondes minimum
- **Fréquence**:
  - Depth images: 15-30 Hz
  - Joint states: 50-100 Hz
  - TF: selon publication
- **Scène**:
  - Robot dans 3-5 configurations différentes
  - 2-3 objets statiques visibles (boîtes, cylindres)
  - Fond avec mur/table
  - Bon éclairage, pas de zones saturées

**Utilisation dans tests**:
- `TEST_DEPTH_001`: Validation publication masked_depth_image
- `TEST_DEPTH_002`: Qualité du masquage robot
- `TEST_E2E_003`: Pipeline perception → planification

**Validation du rosbag**:
- [ ] Camera_info constant et valide (matrice intrinsèque, distorsion)
- [ ] Timestamps synchronisés entre depth et joint_states (<50ms écart)
- [ ] Pas de frames dropout
- [ ] Profondeur dans range valide (0.3m - 3m typiquement)

---

#### 1.2 Rosbag: `test_depth_dynamic_obstacle.bag`

**Description**: Objet mobile pour test de mise à jour temps réel de l'environnement

**Topics requis**:
```
/depth_to_rgb/image_raw              [sensor_msgs/Image]
/depth_to_rgb/camera_info            [sensor_msgs/CameraInfo]
/dsr01/joint_states                  [sensor_msgs/JointState]
/tf                                   [tf2_msgs/TFMessage]
/tf_static                            [tf2_msgs/TFMessage]
```

**Spécifications**:
- **Durée**: 60 secondes
- **Fréquence**: Identique à 1.1
- **Scène**:
  - Robot statique ou mouvement lent
  - Objet entrant dans champ de vision (main, boîte)
  - Objet se déplaçant dans workspace robot
  - Objet sortant du champ

**Utilisation dans tests**:
- `TEST_E2E_002`: Environnement dynamique
- `TEST_VOXEL_002`: Mise à jour voxel grid
- Validation latence détection obstacle

**Métadonnées requises** (dans fichier .yaml associé):
```yaml
rosbag: test_depth_dynamic_obstacle.bag
timestamps:
  object_enters: 5.0s
  object_closest: 15.0s
  object_exits: 25.0s
ground_truth:
  - time: 15.0s
    object_position: [0.5, 0.2, 0.3]  # relative to base_link
    object_size: [0.1, 0.1, 0.15]
```

---

#### 1.3 Rosbag: `test_depth_robot_motion.bag`

**Description**: Robot en mouvement pour test segmentation dynamique

**Topics requis**:
```
/depth_to_rgb/image_raw              [sensor_msgs/Image]
/depth_to_rgb/camera_info            [sensor_msgs/CameraInfo]
/dsr01/joint_states                  [sensor_msgs/JointState]
/tf                                   [tf2_msgs/TFMessage]
/tf_static                            [tf2_msgs/TFMessage]
```

**Spécifications**:
- **Durée**: 45 secondes
- **Scène**:
  - Robot exécutant mouvement complet (home → target → home)
  - Scène statique (pas d'objets mobiles)
  - Caméra fixe

**Utilisation dans tests**:
- Validation que robot est correctement masqué dans toutes configurations
- `TEST_DEPTH_002`: Qualité segmentation
- `robot_pointcloud_debug` topic validation

---

### 2. ROSBAG PERCEPTION - Point Cloud

#### 2.1 Rosbag: `test_pointcloud_workspace.bag`

**Description**: Point cloud du workspace pour tests PointCloudCameraStrategy

**Topics requis**:
```
/camera/depth/points                 [sensor_msgs/PointCloud2]
/dsr01/joint_states                  [sensor_msgs/JointState]
/tf                                   [tf2_msgs/TFMessage]
/tf_static                            [tf2_msgs/TFMessage]
```

**Spécifications**:
- **Durée**: 30 secondes
- **Fréquence**: Point cloud 10-30 Hz
- **Scène**:
  - Workspace avec table, objets posés
  - Point cloud dense (>100k points par frame)
  - Couverture 360° si possible (ou au moins 180°)

**Utilisation dans tests**:
- Validation conversion point cloud → depth orthographique
- Test de `PointCloudCameraStrategy`
- Validation grille voxel depuis point cloud

---

### 3. ROSBAG ROBOT CONTROL

#### 3.1 Rosbag: `test_robot_trajectory_execution.bag`

**Description**: Exécution complète de trajectoire sur robot réel

**Topics requis**:
```
/dsr01/joint_states                  [sensor_msgs/JointState]
/leeloo/execute_trajectory           [trajectory_msgs/JointTrajectory]
/leeloo/trajectory_state             [std_msgs/Float32]
/tf                                   [tf2_msgs/TFMessage]
```

**Spécifications**:
- **Durée**: 20-30 secondes (trajectoire complète)
- **Contenu**:
  - État initial (5s statique)
  - Publication de la trajectoire sur /leeloo/execute_trajectory
  - Exécution (10-15s)
  - État final (5s statique)
  - Progression sur /leeloo/trajectory_state de 0.0 → 1.0

**Utilisation dans tests**:
- `TEST_ACTION_001`: Exécution trajectoire complète
- `TEST_ACTION_003`: Validation feedback temps réel
- `TEST_DOOSAN_001`: Publication sur contrôleur Doosan
- Validation que trajectoire planifiée est suivie (erreur tracking)

**Ground truth requis** (fichier .yaml):
```yaml
rosbag: test_robot_trajectory_execution.bag
trajectory:
  start_time: 5.0s
  end_time: 20.0s
  waypoints_count: 10
  joint_names: [joint1, joint2, joint3, joint4, joint5, joint6]
  expected_positions:
    - time: 5.0s
      joints: [0.0, -0.5, 1.0, 0.0, 1.57, 0.0]
    - time: 20.0s
      joints: [1.57, -1.0, 1.5, 0.5, 1.0, 0.0]
  max_tracking_error: 0.05  # radians
```

---

#### 3.2 Rosbag: `test_robot_joint_states_variety.bag`

**Description**: Diverses configurations du robot pour tests FK/IK

**Topics requis**:
```
/dsr01/joint_states                  [sensor_msgs/JointState]
/tf                                   [tf2_msgs/TFMessage]
```

**Spécifications**:
- **Durée**: 60 secondes
- **Contenu**: Robot passant par:
  - Configuration home
  - Configuration près des limites articulaires
  - Configuration près de singularités
  - 5-10 configurations intermédiaires variées
  - Pause de 5s à chaque configuration

**Utilisation dans tests**:
- `TEST_FK_001-004`: Validation FK
- `TEST_IK_001-004`: Seed pour IK
- Tests de cohérence IK→FK round-trip

**Métadonnées** (.yaml):
```yaml
rosbag: test_robot_joint_states_variety.bag
configurations:
  - name: "home"
    time: 5.0s
    joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ee_pose:
      position: [x, y, z]
      orientation: [qx, qy, qz, qw]
  - name: "singularity_near"
    time: 15.0s
    joints: [1.57, 0.0, 0.0, 0.0, 0.0, 0.0]
    note: "Near wrist singularity"
  # ... autres configs
```

---

### 4. ROSBAG VISUALISATION

#### 4.1 Rosbag: `test_visualization_markers.bag`

**Description**: Marqueurs de visualisation pour validation RViz

**Topics requis**:
```
/curobo_gen_traj/collision_spheres   [visualization_msgs/MarkerArray]
/visualization_marker_array          [visualization_msgs/MarkerArray]
/visualise_voxel_grid                [visualization_msgs/Marker]
/tf                                   [tf2_msgs/TFMessage]
/tf_static                            [tf2_msgs/TFMessage]
```

**Spécifications**:
- **Durée**: 30 secondes
- **Contenu**:
  - Génération de trajectoire avec visualisation waypoints
  - Affichage sphères de collision robot
  - Voxel grid avec obstacles

**Utilisation dans tests**:
- `TEST_VIZ_001-003`: Validation publications marqueurs
- `TEST_SPHERES_001-002`: Sphères de collision
- Tests RViz configs

---

### 5. ROSBAG SCÉNARIOS COMPLETS

#### 5.1 Rosbag: `test_e2e_pick_place.bag`

**Description**: Scénario complet pick-and-place avec perception

**Topics requis**:
```
# Perception
/depth_to_rgb/image_raw              [sensor_msgs/Image]
/depth_to_rgb/camera_info            [sensor_msgs/CameraInfo]

# Robot
/dsr01/joint_states                  [sensor_msgs/JointState]
/leeloo/execute_trajectory           [trajectory_msgs/JointTrajectory]
/leeloo/trajectory_state             [std_msgs/Float32]

# Segmentation
/masked_depth_image                  [sensor_msgs/Image]
/collision_spheres                   [visualization_msgs/MarkerArray]

# Visualisation
/visualization_marker_array          [visualization_msgs/MarkerArray]
/visualise_voxel_grid                [visualization_msgs/Marker]

# TF
/tf                                   [tf2_msgs/TFMessage]
/tf_static                            [tf2_msgs/TFMessage]
```

**Spécifications**:
- **Durée**: 120 secondes (scénario complet)
- **Phases**:
  1. Scan initial scène (10s)
  2. Planification approche objet (5s)
  3. Exécution mouvement vers objet (15s)
  4. Préhension (5s statique)
  5. Planification mouvement vers zone placement (5s)
  6. Exécution mouvement (15s)
  7. Dépose (5s)
  8. Retour home (15s)

**Utilisation dans tests**:
- `TEST_E2E_001`: Pipeline complet pick-place
- Validation latences bout-en-bout
- Test d'intégration global

**Métadonnées détaillées requises**:
```yaml
rosbag: test_e2e_pick_place.bag
scenario: pick_and_place
object:
  type: "box"
  dimensions: [0.05, 0.05, 0.08]
  pick_pose:
    position: [0.5, 0.2, 0.0]
    orientation: [0, 0, 0, 1]
  place_pose:
    position: [0.3, -0.3, 0.1]
    orientation: [0, 0, 0.707, 0.707]
phases:
  - name: "scan"
    start_time: 0.0s
    end_time: 10.0s
  - name: "plan_approach"
    start_time: 10.0s
    end_time: 15.0s
  - name: "execute_approach"
    start_time: 15.0s
    end_time: 30.0s
    expected_ee_pose: [0.5, 0.2, 0.1]
  # ... autres phases
success_criteria:
  position_accuracy: 0.01  # m
  orientation_accuracy: 0.05  # rad
  total_time_max: 120.0  # s
```

---

#### 5.2 Rosbag: `test_e2e_obstacle_avoidance.bag`

**Description**: Évitement d'obstacles dynamiques

**Topics requis**: Identiques à 5.1

**Spécifications**:
- **Durée**: 90 secondes
- **Scène**:
  - Trajectoire planifiée initialement
  - Obstacle apparaît dans le chemin (t=20s)
  - Replanification nécessaire
  - Exécution de trajectoire modifiée

**Utilisation dans tests**:
- `TEST_E2E_002`: Environnement dynamique
- Validation replanification temps réel

---

### 6. ROSBAG STRESS & PERFORMANCE

#### 6.1 Rosbag: `test_high_frequency_data.bag`

**Description**: Flux haute fréquence pour tests de performance

**Topics requis**:
```
/depth_to_rgb/image_raw              [sensor_msgs/Image] @ 60Hz
/dsr01/joint_states                  [sensor_msgs/JointState] @ 200Hz
/camera/depth/points                 [sensor_msgs/PointCloud2] @ 30Hz
```

**Spécifications**:
- **Durée**: 300 secondes (5 minutes)
- **But**: Tester stabilité sous charge continue

**Utilisation dans tests**:
- `TEST_E2E_005`: Stress test
- Validation pas de memory leaks
- Mesure utilisation CPU/GPU

---

## Structure de Fichiers Rosbag Proposée

```
curobo_ros/
├── test/
│   └── rosbags/
│       ├── perception/
│       │   ├── test_depth_static_scene.db3
│       │   ├── test_depth_static_scene.yaml
│       │   ├── test_depth_dynamic_obstacle.db3
│       │   ├── test_depth_dynamic_obstacle.yaml
│       │   ├── test_depth_robot_motion.db3
│       │   ├── test_depth_robot_motion.yaml
│       │   ├── test_pointcloud_workspace.db3
│       │   └── test_pointcloud_workspace.yaml
│       ├── robot_control/
│       │   ├── test_robot_trajectory_execution.db3
│       │   ├── test_robot_trajectory_execution.yaml
│       │   ├── test_robot_joint_states_variety.db3
│       │   └── test_robot_joint_states_variety.yaml
│       ├── visualization/
│       │   ├── test_visualization_markers.db3
│       │   └── test_visualization_markers.yaml
│       ├── e2e_scenarios/
│       │   ├── test_e2e_pick_place.db3
│       │   ├── test_e2e_pick_place.yaml
│       │   ├── test_e2e_obstacle_avoidance.db3
│       │   └── test_e2e_obstacle_avoidance.yaml
│       ├── stress/
│       │   ├── test_high_frequency_data.db3
│       │   └── test_high_frequency_data.yaml
│       └── README.md
```

---

## Commandes pour Enregistrer les Rosbags

### Perception - Scène Statique
```bash
ros2 bag record -o test_depth_static_scene \
  /depth_to_rgb/image_raw \
  /depth_to_rgb/camera_info \
  /dsr01/joint_states \
  /tf /tf_static \
  --duration 30
```

### Perception - Obstacle Dynamique
```bash
ros2 bag record -o test_depth_dynamic_obstacle \
  /depth_to_rgb/image_raw \
  /depth_to_rgb/camera_info \
  /dsr01/joint_states \
  /tf /tf_static \
  --duration 60
```

### Perception - Robot en Mouvement
```bash
ros2 bag record -o test_depth_robot_motion \
  /depth_to_rgb/image_raw \
  /depth_to_rgb/camera_info \
  /dsr01/joint_states \
  /tf /tf_static \
  --duration 45
```

### Point Cloud
```bash
ros2 bag record -o test_pointcloud_workspace \
  /camera/depth/points \
  /dsr01/joint_states \
  /tf /tf_static \
  --duration 30
```

### Exécution Trajectoire
```bash
ros2 bag record -o test_robot_trajectory_execution \
  /dsr01/joint_states \
  /leeloo/execute_trajectory \
  /leeloo/trajectory_state \
  /tf \
  --duration 30
```

### États Joints Variés
```bash
ros2 bag record -o test_robot_joint_states_variety \
  /dsr01/joint_states \
  /tf \
  --duration 60
```

### Marqueurs Visualisation
```bash
ros2 bag record -o test_visualization_markers \
  /curobo_gen_traj/collision_spheres \
  /visualization_marker_array \
  /visualise_voxel_grid \
  /tf /tf_static \
  --duration 30
```

### Scénario E2E Pick-Place
```bash
ros2 bag record -o test_e2e_pick_place \
  /depth_to_rgb/image_raw \
  /depth_to_rgb/camera_info \
  /dsr01/joint_states \
  /leeloo/execute_trajectory \
  /leeloo/trajectory_state \
  /masked_depth_image \
  /collision_spheres \
  /visualization_marker_array \
  /visualise_voxel_grid \
  /tf /tf_static \
  --duration 120
```

### Scénario E2E Évitement Obstacles
```bash
ros2 bag record -o test_e2e_obstacle_avoidance \
  /depth_to_rgb/image_raw \
  /depth_to_rgb/camera_info \
  /dsr01/joint_states \
  /leeloo/execute_trajectory \
  /leeloo/trajectory_state \
  /masked_depth_image \
  /collision_spheres \
  /visualization_marker_array \
  /tf /tf_static \
  --duration 90
```

### Stress Test Haute Fréquence
```bash
ros2 bag record -o test_high_frequency_data \
  /depth_to_rgb/image_raw \
  /dsr01/joint_states \
  /camera/depth/points \
  --duration 300
```

---

## Format des Fichiers Métadonnées (.yaml)

Chaque rosbag doit avoir un fichier .yaml associé avec les métadonnées suivantes:

```yaml
# test_depth_static_scene.yaml
rosbag:
  name: test_depth_static_scene
  version: 1.0
  date_recorded: 2025-11-10
  duration: 30.0  # seconds

hardware:
  robot: Doosan M1013
  camera: RealSense D435
  camera_serial: 123456789

environment:
  lighting: normal_indoor
  objects:
    - name: "box_1"
      type: "cube"
      dimensions: [0.1, 0.1, 0.1]
      position: [0.5, 0.2, 0.0]  # relative to base_link
      color: "red"
    - name: "cylinder_1"
      type: "cylinder"
      radius: 0.05
      height: 0.15
      position: [0.4, -0.2, 0.0]
      color: "blue"

robot_configs:
  initial_joints: [0.0, -0.5, 1.0, 0.0, 1.57, 0.0]
  configurations_count: 5
  joint_range_coverage: 70  # percentage of joint limits explored

quality:
  frame_drops: 0
  timestamp_sync_max_error: 0.030  # seconds
  depth_min: 0.3  # meters
  depth_max: 2.5  # meters

test_compatibility:
  - TEST_DEPTH_001
  - TEST_DEPTH_002
  - TEST_E2E_003

notes: |
  Good lighting conditions.
  All objects clearly visible.
  No occlusions.
  Robot moves slowly through 5 configurations.
```

---

## Validation des Rosbags

Avant d'utiliser un rosbag pour les tests, valider avec ce script:

```python
# validate_rosbag.py
import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import yaml

def validate_rosbag(bag_path, metadata_path):
    """Valide qu'un rosbag contient toutes les données requises"""

    # Charger métadonnées
    with open(metadata_path, 'r') as f:
        metadata = yaml.safe_load(f)

    # Ouvrir rosbag
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Vérifications
    checks = {
        'topics_present': False,
        'duration_correct': False,
        'no_gaps': False,
        'timestamps_sync': False
    }

    # Vérifier topics
    topics_info = reader.get_all_topics_and_types()
    required_topics = [...]  # From metadata
    checks['topics_present'] = all(t in topics_info for t in required_topics)

    # Vérifier durée
    metadata_info = reader.get_metadata()
    duration = (metadata_info.duration.nanoseconds) / 1e9
    expected_duration = metadata['rosbag']['duration']
    checks['duration_correct'] = abs(duration - expected_duration) < 1.0

    # ... autres vérifications

    return checks

# Utilisation
if __name__ == '__main__':
    result = validate_rosbag(
        'test/rosbags/perception/test_depth_static_scene.db3',
        'test/rosbags/perception/test_depth_static_scene.yaml'
    )
    print(f"Validation results: {result}")
```

---

## Utilisation des Rosbags dans les Tests

### Exemple: Test avec Replay Rosbag

```python
# test_depth_perception.py
import pytest
import rclpy
from rosbag2_py import Player
import launch
import launch_ros

@pytest.fixture
def rosbag_player():
    """Lance player rosbag"""
    player = Player()
    player.play('test/rosbags/perception/test_depth_static_scene.db3')
    yield player
    player.stop()

def test_depth_segmentation_static_scene(rosbag_player):
    """TEST_DEPTH_001: Validation publication masked_depth_image"""

    # Lancer nœud robot_segmentation
    node = DepthMapRobotSegmentation()

    # Attendre messages
    masked_images = []

    def callback(msg):
        masked_images.append(msg)

    node.create_subscription(Image, 'masked_depth_image', callback, 10)

    # Laisser rosbag jouer
    rclpy.spin_for(node, timeout_sec=30.0)

    # Validations
    assert len(masked_images) > 100  # Au moins 100 images à 15Hz sur 30s
    assert all(img.encoding == '32FC1' for img in masked_images)

    # Vérifier que robot est masqué
    # ... analyse des images
```

---

## Checklist Enregistrement Rosbags

Avant chaque session d'enregistrement:

- [ ] Calibration caméra vérifiée (camera_info correct)
- [ ] TF tree complet publié
- [ ] Éclairage stable et adéquat
- [ ] Espace de travail dégagé
- [ ] Robot homed et prêt
- [ ] Stockage suffisant (>10GB disponible)
- [ ] Noms de fichiers selon convention
- [ ] Notes manuscrites des conditions d'enregistrement

Après chaque enregistrement:

- [ ] Vérifier durée du rosbag
- [ ] Vérifier tous topics présents
- [ ] Pas de warnings/erreurs pendant enregistrement
- [ ] Créer fichier .yaml métadonnées
- [ ] Validation avec script validate_rosbag.py
- [ ] Backup dans stockage partagé
- [ ] Ajout dans documentation

---

## Estimation Taille des Rosbags

| Rosbag | Durée | Topics Principaux | Taille Estimée |
|--------|-------|-------------------|----------------|
| test_depth_static_scene | 30s | depth (30Hz) + joints | ~500 MB |
| test_depth_dynamic_obstacle | 60s | depth (30Hz) + joints | ~1 GB |
| test_depth_robot_motion | 45s | depth (30Hz) + joints | ~750 MB |
| test_pointcloud_workspace | 30s | pointcloud (15Hz) | ~2 GB |
| test_robot_trajectory_execution | 30s | joints + traj | ~50 MB |
| test_robot_joint_states_variety | 60s | joints | ~100 MB |
| test_visualization_markers | 30s | markers | ~100 MB |
| test_e2e_pick_place | 120s | tous topics | ~3 GB |
| test_e2e_obstacle_avoidance | 90s | tous topics | ~2.5 GB |
| test_high_frequency_data | 300s | high freq | ~10 GB |

**Total estimé**: ~20-25 GB pour suite complète de rosbags

---

## Stockage et Gestion

### Organisation Git LFS

Les rosbags sont trop volumineux pour Git standard. Utiliser Git LFS:

```bash
# Installer Git LFS
git lfs install

# Tracker les rosbags
git lfs track "test/rosbags/**/*.db3"
git add .gitattributes

# Commit
git add test/rosbags/
git commit -m "Add test rosbags"
```

### Alternative: Stockage Cloud

Si Git LFS n'est pas disponible, héberger sur:
- Google Drive / OneDrive
- AWS S3
- Lab server NAS
- DVC (Data Version Control)

Ajouter script de téléchargement:

```bash
# download_test_data.sh
#!/bin/bash

echo "Downloading test rosbags..."

# URL du stockage
BASE_URL="https://lab-server.com/curobo_ros/test_data"

# Télécharger chaque rosbag
wget $BASE_URL/test_depth_static_scene.db3 -P test/rosbags/perception/
wget $BASE_URL/test_depth_static_scene.yaml -P test/rosbags/perception/
# ... autres fichiers

echo "Download complete!"
```

---

## Résumé des Rosbags Prioritaires

### Priorité HAUTE (P0) - Essentiel pour CI/CD

1. ✅ `test_depth_static_scene.bag` - Perception de base
2. ✅ `test_robot_joint_states_variety.bag` - IK/FK tests
3. ✅ `test_robot_trajectory_execution.bag` - Exécution

### Priorité MOYENNE (P1) - Tests avancés

4. ⚠️ `test_depth_dynamic_obstacle.bag` - Environnement dynamique
5. ⚠️ `test_e2e_pick_place.bag` - Scénario complet
6. ⚠️ `test_pointcloud_workspace.bag` - Point cloud

### Priorité BASSE (P2) - Tests spécifiques

7. 🔵 `test_visualization_markers.bag` - Visualisation
8. 🔵 `test_high_frequency_data.bag` - Performance
9. 🔵 `test_e2e_obstacle_avoidance.bag` - Avancé

---

## Prochaines Actions

1. **Créer le dossier** `test/rosbags/` avec sous-dossiers
2. **Enregistrer les 3 rosbags P0** en priorité
3. **Créer fichiers .yaml** métadonnées pour chaque rosbag
4. **Valider** les rosbags avec script validation
5. **Configurer Git LFS** ou alternative stockage
6. **Documenter** procédure d'enregistrement
7. **Intégrer** dans tests CI/CD
