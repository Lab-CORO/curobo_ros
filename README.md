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
bash start_docker_x86.sh
```

Pour lancer le conteneur sur une branche autre que `main`, ajoutez le nom de la branche désirée à la commande précédente:

```bash
bash start_docker_x86.sh <branche>
```


## Utilisation des fonctionalites du Docker

<!-- ### Instructions pour lancer les scripts de curobo

Pour exécuter les scripts de curobo, utilisez la commande suivante :

```bash
python /chemin/vers/le/script
```

Remplacez `/chemin/vers/le/script` par le chemin réel du script que vous souhaitez exécuter. -->
### Ouvrir plusieurs terminals dans le docker

```bash
docker exec -it x86docker bash
```

### Utilisation de curobo pour générer une trajectoire avec incorporation de la caméra

Pour utiliser curobo et générer une trajectoire intégrant la caméra, veuillez suivre les étapes suivantes :

1. Assurez-vous d'avoir complété les étapes précédentes.
2. Accédez au répertoire `/home/ros2_ws`.
3. Exécutez la commande suivante :

```bash
ros2 run curobo_ros curobo_gen_traj
```

### Lancement du Doosan M1013 dans Rviz2

Pour utiliser Rviz2 afin de visualiser le robot ou toute autre chose, veuillez exécuter la commande suivante :

```bash
ros2 launch curobo_ros gen_traj.launch.py
```

Ce fichier de lancement est configurable a votre souhait.

### Lancement du package de la camera Intel Realsense D405

Pour utiliser la caméra afin de générer une trajectoire, veuillez exécuter la commande suivante :

```bash
ros2 launch realsense2_camera rs_launch.py clip_distance:=0.8
```

De nombreux arguments peuvent être utilisés (comme le clip_distance), vous pouvez vous référer à ce [repo](https://github.com/IntelRealSense/realsense-ros) GitHub.

### Ajout du TF de la camera

Pour ajouter le TF de la caméra afin de visualiser le DepthCloud et le robot en simultané, vous pouvez exécuter cette commande :

```bash
ros2 run tf2_ros static_transform_publisher x y z tx ty tz departure_link arrival_link
```

```bash
ros2 run tf2_ros static_transform_publisher  0.5 0 0.5 0 0 0  base_0 camera_link
```

Remplacer ces champs par ce qui vous convient.

### Gestion manuelle d'objets virtuels pour évitement

Des services ont été ajoutés pour ajouter et retirer différents types d'objets à l'environnement. Il est à noté que, dans tous les cas, les objets sont transformés en Cuboid avant d'être ajoutés au système. Ceci est dû à une limitation actuelle du [Collision World Representation](https://curobo.org/get_started/2c_world_collision.html).

Présentement, il est possible d'ajouter les types suivants. Dans chacun des cas, l'interprétation du champ `dimensions` varie selon la définition associée ci-bas :

| Type      | Interprétation de `dimensions` |
| --------- | ----------------------------- |
| Cube (Cuboid selon l'appellation cuRobo) | `[longueur en x, largeur en y, hauteur en z]` |
| Sphère | `[rayon, _, _]` |
| Capsule | `[rayon, _, _]` |
| Cylindre | `[rayon, hauteur, _]` |
| Mesh (à partir d'un fichier fournis par le paramètre optionnel `mesh_file_path`) | Échelles en `[x, y, z]` |

À noter: le symbole `_` signifie que la valeur qui y est mise est inutilisée.

Voici un exemple pour l'ajout et le retrait d'un cube de 25 cm:

Ajout:

```bash
ros2 service call /curobo_gen_traj/add_object curobo_msgs/srv/AddObject "{type: 0, name: 'test_cuboid', pose: {position: {x: 0.70, y: 0.0, z: 1.0}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 1.0}}, dimensions: {x: 0.25, y: 0.25, z: 0.25}, color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}"
```

Retrait:

```bash
ros2 service call /curobo_gen_traj/remove_object curobo_msgs/srv/RemoveObject "{name: 'test_cuboid'}"
```

Il est aussi possible de retirer tous les objets ajoutés d'un seul coup:

```bash
ros2 service call /curobo_gen_traj/remove_all_objects std_srvs/srv/Trigger
```

Ajout dans le cas d'un mesh:

```bash
ros2 service call /curobo_gen_traj/add_object curobo_msgs/srv/AddObject "{type: 4, name: 'test_mesh', mesh_file_path: '/home/ros2_ws/src/curobo_ros/curobo_doosan/src/m1013/meshes/m1013_white/MF1013_0_0.dae', pose: {position: {x: 0.40, y: 0.0, z: 1.0}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 1.0}}, dimensions: {x: 0.0025, y: 0.0025, z: 0.0025}, color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}"
```

## Potentiels problèmes rencontrés

### Problème potentiel avec CUDA

Lors de l'utilisation de ce projet Docker, il est possible que vous rencontriez des problèmes avec CUDA, comme le message d'erreur `CUDA_DEVICE_NOT_FOUND`. Si cela se produit, il peut être nécessaire de redémarrer le serveur graphique GDM pour résoudre ce problème.

Pour redémarrer le serveur graphique GDM, exécutez la commande suivante :

```bash
sudo systemctl restart gdm
```

### Problème avec le package OpenCV

Lors de l'utilisation de la camera, il se peut que vous rencontriez l'erreur `AttributeError: module 'cv2.dnn' has no attribute 'DictValue'`
Pour resoudre cela, vous pouvez commenter la ligne 171 du fichier suivant :

```bash
nano /usr/local/lib/python3.10/dist-packages/cv2/typing/__init__.py
```

### Résolution du problème de symbole

Pour résoudre le problème de symbole manquant `ucm_set_global_opts`, veuillez exécuter le script ci-dessous à l'intérieur du conteneur Docker :

```bash
sudo apt-get update && sudo apt-get install --reinstall -y \
  libmpich-dev \
  hwloc-nox libmpich12 mpich
```

### Problèmes exclusifs à Windows

#### Incapacité de lancer les conteneurs

Dépendamment de la configuration du matériel, Docker pourrait ne pas fonctionner du tout dû à une absence de mémoire virtuelle. Pour régler cela, il suffit d'activer l'option de `virtualisation` du CPU dans votre BIOS. Le nom exacte de l'option et la méthode pour effectuer le changement variera d'un ordinateur à l'autre.

#### Incapacité de lancer de fenêtres à partir du Docker sur Windows

Le projet nécessite un `Xserver` pour lancer des fenêtres à partir du Docker sur l'ordinateur local. Une solution sous Windows est l'utilisation de [XLaunch](https://x.cygwin.com/docs/xlaunch/) disponible [ici](https://sourceforge.net/projects/vcxsrv/).

Préalablement au lancement initial du script `start_docker_x86.sh`, il faut:

- Lancer XLaunch. Choissisez les options par défaut, sauf pour la case `Disable access control` qu'il faut cocher.
- Dans le terminal que vous utiliserez pour lancer le script, effectuez:

  ```bash
  export DISPLAY=<adresse_IP>:0
  ```

  Votre adresse IP locale peut être trouvé grâce à la commande `ipconfig`
