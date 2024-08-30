# Curobo_ros
This package is a light implementation for curobo. Three services are exeposed:
- The inverse kinematics
- The forward kinematic
- The trajectory generation with a camera and a rviz interactive marker (only the D405 camera have been tested currently)
## Future works and implementations
- Rviz interface
  - Add collisions primitive geometry
  - Visualize the generated trajectory
  - Add trajectory constrainte
  - Switch from trajectory generation and MPC active trajectory
- MPC trajectories
- Robot segmentation
   

## Instructions pour lancer le Docker x86

### Étape 1 : Construire le conteneur Docker
Pour construire le conteneur Docker avec la configuration x86, exécutez la commande suivante :
```bash
bash build_docker.sh x86
```

### Étape 2 : Démarrer le conteneur Docker
Pour démarrer le conteneur Docker avec la configuration x86, exécutez la commande suivante :
```bash
bash start_docker.sh x86
```


## Utilisation des fonctionalites du Docker

### Instructions pour lancer les scripts de curobo

Pour exécuter les scripts de curobo, utilisez la commande suivante :

```bash
python /chemin/vers/le/script
```

Remplacez `/chemin/vers/le/script` par le chemin réel du script que vous souhaitez exécuter.

### Utilisation de curobo pour générer une trajectoire avec incorporation de la caméra

Pour utiliser curobo et générer une trajectoire intégrant la caméra, veuillez suivre les étapes suivantes :

1. Assurez-vous d'avoir complété l'étape 4.
2. Accédez au répertoire `/home/ros2_ws`.
3. Exécutez la commande suivante :

```bash
ros2 run trajectory_publisher trajectory_pub_node
```

### Lancement du Doosan M1013 dans Rviz2

Pour utiliser Rviz2 afin de visualiser le robot ou toute autre chose, veuillez exécuter la commande suivante :

```bash
ros2 launch trajectory_publisher launch_rviz2.launch.py
```
Ce fichier de lancement est configurable a votre souhait.

### Lancement du package de la camera Intel Realsense D405
Pour utiliser la caméra afin de générer une trajectoire, veuillez exécuter la commande suivante :

```bash
ros2 launch realsense2_camera rs_launch.py
```
De nombreux arguments peuvent être utilisés (comme le clip_distance), vous pouvez vous référer à ce [repo](https://github.com/IntelRealSense/realsense-ros) GitHub.
### Ajout du TF de la camera
Pour ajouter le TF de la caméra afin de visualiser le DepthCloud et le robot en simultané, vous pouvez exécuter cette commande :
```bash
ros2 run tf2_ros static_transform_publisher x y z tx ty tz departure_link arrival_link
```
```bash
ros2 run tf2_ros static_transform_publisher 0.5 1.0 0.5 2.8250572 -0.9121486 2.9853193 base_0 camera_link
```

Remplacer ces champs par ce qui vous convient.

### Ajout de l'interactive marker dans Rviz
Pour ajouter un interactive marker dans Rviz, vous pouvez exécuter cette commande :
```bash
./src/curobo_ros/curobo_ros/simple_arrow.py
```

## Potentiels problèmes rencontrés 

### Problème potentiel avec CUDA

Lors de l'utilisation de ce projet Docker, il est possible que vous rencontriez des problèmes avec CUDA, comme le message d'erreur `CUDA_DEVICE_NOT_FOUND`. Si cela se produit, il peut être nécessaire de redémarrer le serveur graphique GDM pour résoudre ce problème.

Pour redémarrer le serveur graphique GDM, exécutez la commande suivante :

```bash
sudo systemctl stop gdm
```
puis
```bash
sudo systemctl start gdm
```

### Problème avec le package OpenCV
Lors de l'utilisation de la camera, il se peut que vous rencontriez l'erreur `AttributeError: module 'cv2.dnn' has no attribute 'DictValue' `
Pour resoudre cela, vous pouvez commenter la ligne 171 du fichier suivant :
```bash
code /usr/local/lib/python3.10/dist-packages/cv2/typing/__init__.py
```
### Résolution du problème de symbole

Pour résoudre le problème de symbole manquant `ucm_set_global_opts`, veuillez exécuter le script ci-dessous à l'intérieur du conteneur Docker :

```bash
sudo apt-get update && sudo apt-get install --reinstall -y \
  libmpich-dev \
  hwloc-nox libmpich12 mpich
```
