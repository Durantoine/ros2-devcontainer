# Quick Reference - Commandes ROS2 Multi-Workspace

## 🚀 Démarrage

```bash
# Sur votre Mac
cd /Users/durantoine/Dev/ros2_devcontainer
code .
# Puis : Dev Containers: Reopen in Container
```

## 📦 Gestion des Workspaces

### Lister les workspaces
```bash
lsws
# ou
list_workspaces
```

### Switcher de workspace
```bash
ws mon_projet
# ou
switch_workspace mon_projet
```

### Builder un workspace
```bash
bws mon_projet
# ou
build_workspace mon_projet
# ou (après avoir switché)
build
```

### Créer un nouveau workspace
```bash
# Depuis votre Mac (hors container)
cd /Users/durantoine/Dev/ros2_devcontainer
./create_workspace.sh nouveau_projet

# Dans le container
mkdir /workspaces/nouveau_projet
```

## 🔧 Commandes ROS2

### Créer un package
```bash
cd /workspaces/mon_projet

# Package Python
ros2 pkg create --build-type ament_python mon_package

# Package C++
ros2 pkg create --build-type ament_cmake mon_package_cpp

# Avec dépendances
ros2 pkg create --build-type ament_python mon_package \
  --dependencies rclpy std_msgs sensor_msgs
```

### Builder
```bash
# Build complet
colcon build --symlink-install

# Build avec nettoyage du cache
cbuild

# Build d'un package spécifique
colcon build --packages-select mon_package

# Build parallèle (plus rapide)
colcon build --parallel-workers 4
```

### Sourcer l'environnement
```bash
source /ros2_ws/install/setup.bash
# ou
source_ws
```

### Lancer des nodes
```bash
# Lancer un node
ros2 run mon_package mon_node

# Lancer un launch file
ros2 launch mon_package mon_launch.py

# Lister les nodes actifs
ros2 node list

# Infos sur un node
ros2 node info /mon_node
```

### Topics
```bash
# Lister les topics
ros2 topic list

# Voir les messages d'un topic
ros2 topic echo /mon_topic

# Publier sur un topic
ros2 topic pub /mon_topic std_msgs/msg/String "data: 'Hello'"

# Info sur un topic
ros2 topic info /mon_topic
```

### Services
```bash
# Lister les services
ros2 service list

# Appeler un service
ros2 service call /mon_service std_srvs/srv/Trigger

# Type d'un service
ros2 service type /mon_service
```

## 🖥️ Interface Graphique

### Accès VNC
- **noVNC Web** : http://localhost:6080 (ouvre automatiquement)
- **VNC Direct** : localhost:5900

### Applications GUI
```bash
# Turtlesim (test)
ros2 run turtlesim turtlesim_node

# RViz (visualisation)
rviz2

# Gazebo (simulation)
gazebo

# rqt (GUI tools)
rqt
rqt_graph  # Voir le graph des nodes
rqt_console  # Console de logs
```

## 🐛 Debug & Logs

### Voir les logs
```bash
# Logs d'un node
ros2 run mon_package mon_node --ros-args --log-level debug

# Console de logs
ros2 topic echo /rosout

# rqt_console (GUI)
rqt_console
```

### Inspecter le système
```bash
# Informations système
ros2 wtf

# Vérifier les paramètres
ros2 param list
ros2 param get /mon_node mon_parametre

# Graph des nodes (console)
ros2 node list
ros2 topic list
ros2 service list

# Graph des nodes (GUI)
rqt_graph
```

## 🧹 Nettoyage

### Nettoyer les builds
```bash
# Dans le container
rm -rf /ros2_ws/build /ros2_ws/install /ros2_ws/log

# Puis rebuilder
cbuild
```

### Supprimer les volumes Docker
```bash
# Sur votre Mac (ATTENTION : efface tous les builds)
cd /Users/durantoine/Dev/ros2_devcontainer
docker-compose down -v
docker-compose up -d
```

## 🔄 Workflow recommandé

1. **Créer un workspace**
   ```bash
   ./create_workspace.sh mon_robot
   ```

2. **Ouvrir le container**
   ```bash
   code /Users/durantoine/Dev/ros2_devcontainer
   # Dev Containers: Reopen in Container
   ```

3. **Switcher vers le workspace**
   ```bash
   ws mon_robot
   cd /workspaces/mon_robot
   ```

4. **Créer des packages**
   ```bash
   ros2 pkg create --build-type ament_python navigation_pkg
   ros2 pkg create --build-type ament_python perception_pkg
   ```

5. **Développer, builder, tester**
   ```bash
   # Éditer le code...
   bws mon_robot
   source_ws
   ros2 run navigation_pkg mon_node
   ```

6. **Lancer des applications GUI**
   ```bash
   rviz2  # Dans l'interface VNC
   ```

## 🎯 Tips & Tricks

### Alias disponibles
- `build` = `colcon build --symlink-install`
- `cbuild` = `colcon build --symlink-install --cmake-clean-cache`
- `source_ws` = `source /ros2_ws/install/setup.bash`
- `lsws` = `list_workspaces`
- `ws` = `switch_workspace`
- `bws` = `build_workspace`

### Auto-complétion
Le shell dans le container a l'auto-complétion ROS2 activée. Utilisez `Tab` !

### Plusieurs terminaux
Vous pouvez ouvrir plusieurs terminaux dans VSCode, tous auront accès au même container et aux mêmes workspaces.

### Ports forwarded
- 5900 : VNC
- 6080 : noVNC Web

Vérifiez l'onglet "Ports" en bas de VSCode.

## 📚 Documentation

- [README.md](README.md) : Documentation complète
- [GUIDE_MIGRATION.md](GUIDE_MIGRATION.md) : Guide de migration
- [ROS2 Jazzy Docs](https://docs.ros.org/en/jazzy/)
