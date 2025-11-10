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
