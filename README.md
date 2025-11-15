# ROS2 Multi-Workspace Dev Container

Configuration centralisée pour développer plusieurs projets ROS2 indépendants dans un seul container.

## 🏗️ Structure

```
ros2_devcontainer/
├── .devcontainer/           # Configuration VSCode Dev Container
│   └── devcontainer.json
├── workspaces/              # Tous vos projets ROS2
│   ├── projet_current/      # Votre projet actuel
│   ├── projet2/            # Créez autant de workspaces que nécessaire
│   └── projet3/
├── Dockerfile              # Image Docker ROS2 avec GUI
├── docker-compose.yml      # Configuration du container
├── supervisord.conf        # Configuration services (VNC, etc.)
├── create_workspace.sh     # Script pour créer un nouveau workspace
└── workspace_helpers.sh    # Fonctions helper dans le container
```

## 🚀 Démarrage rapide

### 1. Ouvrir le container

```bash
cd /Users/durantoine/Dev/ros2_devcontainer
code .
```

Dans VSCode : **Dev Containers: Reopen in Container**

L'interface VNC s'ouvrira automatiquement dans votre navigateur à http://localhost:6080

### 2. Créer un nouveau workspace

**Depuis votre Mac (hors container) :**
```bash
cd /Users/durantoine/Dev/ros2_devcontainer
./create_workspace.sh mon_nouveau_projet
```

**Dans le container :**
```bash
mkdir /workspaces/mon_nouveau_projet
```

### 3. Travailler avec les workspaces

**Dans le container**, vous avez accès à des commandes helper :

```bash
# Lister tous les workspaces
lsws
# ou
list_workspaces

# Switcher vers un workspace
ws projet_current
# ou
switch_workspace projet_current

# Builder un workspace
bws projet_current
# ou
build_workspace projet_current
```

## 📦 Gestion des workspaces

### Créer un package ROS2

```bash
# Switcher vers votre workspace
cd /workspaces/mon_projet

# Créer un package Python
ros2 pkg create --build-type ament_python mon_package

# Créer un package C++
ros2 pkg create --build-type ament_cmake mon_package_cpp
```

### Builder votre workspace

```bash
# Option 1 : Utiliser le helper
bws mon_projet

# Option 2 : Manuellement
cd /ros2_ws
rm -rf src
ln -s /workspaces/mon_projet src
colcon build --symlink-install
```

### Sourcer votre workspace

```bash
source /ros2_ws/install/setup.bash
# ou
source_ws
```

## 🖥️ Interface graphique (GUI)

L'interface VNC est automatiquement démarrée et accessible :

- **noVNC (Web)** : http://localhost:6080 (s'ouvre automatiquement)
- **VNC Direct** : localhost:5900

Pour lancer des applications GUI (Gazebo, RViz, etc.) :

```bash
# Exemple : Turtlesim
ros2 run turtlesim turtlesim_node

# RViz
rviz2

# Gazebo
gazebo
```

## 🔧 Commandes utiles

### Aliases disponibles

- `build` : `colcon build --symlink-install`
- `cbuild` : `colcon build --symlink-install --cmake-clean-cache`
- `source_ws` : `source /ros2_ws/install/setup.bash`
- `lsws` : Liste les workspaces
- `ws <name>` : Switcher de workspace
- `bws <name>` : Builder un workspace

### Rebuilder l'image Docker

Si vous modifiez le Dockerfile :

```bash
cd /Users/durantoine/Dev/ros2_devcontainer
docker-compose build --no-cache
```

Puis dans VSCode : **Dev Containers: Rebuild Container**

## 📝 Notes importantes

- **Isolation des projets** : Chaque workspace dans `/workspaces/` est indépendant
- **Build artifacts partagés** : Par défaut, `build/`, `install/`, `log/` sont partagés entre workspaces (volumes Docker persistants)
- **Pour isoler complètement les builds** : Modifiez `docker-compose.yml` pour créer des volumes séparés par workspace

## 🐛 Troubleshooting

### Le VNC ne s'ouvre pas automatiquement

Vérifiez que le port 6080 n'est pas déjà utilisé :
```bash
lsof -i :6080
```

Ouvrez manuellement : http://localhost:6080/vnc.html?autoconnect=true&resize=scale

### Erreur de build

```bash
# Nettoyer le build
rm -rf /ros2_ws/build /ros2_ws/install /ros2_ws/log
cbuild
```

### Changer le workspace actif ne fonctionne pas

```bash
# Vérifier le lien symbolique
ls -la /ros2_ws/src

# Recréer le lien
switch_workspace mon_projet
```

## 🎯 Workflow recommandé

1. Créer un workspace pour chaque projet ROS2 distinct
2. Ouvrir le dev container une seule fois
3. Utiliser `ws <projet>` pour basculer entre projets
4. Builder avec `bws <projet>` ou `build`
5. L'interface GUI est toujours disponible pour tous les projets

## 🔗 Ressources

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [VSCode Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)
- [Colcon Documentation](https://colcon.readthedocs.io/)
