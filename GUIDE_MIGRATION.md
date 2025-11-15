# Guide de Migration - Setup Multi-Workspace

## ✅ Ce qui a été fait

Votre setup ROS2 a été migré d'une structure monolithique vers une architecture multi-workspace centralisée.

### Avant
```
ros2_workspace/
├── .devcontainer/
├── Dockerfile
├── docker-compose.yml
└── workspace/          # Un seul projet
    ├── interface_pkg
    └── robot_pkg
```

### Après
```
ros2_devcontainer/           # Configuration centralisée
├── .devcontainer/
├── Dockerfile
├── docker-compose.yml
├── workspaces/              # Plusieurs projets possibles
│   ├── projet_current/     # Votre projet migré
│   └── exemple_projet/     # Nouveaux projets...
├── create_workspace.sh
├── workspace_helpers.sh
└── README.md
```

## 🚀 Comment utiliser

### 1. Ouvrir VSCode dans le nouveau dossier

```bash
cd /Users/durantoine/Dev/ros2_devcontainer
code .
```

### 2. Ouvrir dans le container

- Cmd+Shift+P (ou Ctrl+Shift+P)
- Taper : "Dev Containers: Reopen in Container"
- Attendre le build (première fois seulement)
- **L'interface VNC s'ouvrira automatiquement dans votre navigateur** 🎉

### 3. Travailler sur votre projet actuel

Dans le terminal du container :

```bash
# Lister les workspaces
lsws

# Switcher vers votre projet
ws projet_current

# Builder
bws projet_current
# ou simplement
build

# Sourcer
source_ws
```

### 4. Créer de nouveaux projets

**Depuis votre Mac (terminal hors container) :**
```bash
cd /Users/durantoine/Dev/ros2_devcontainer
./create_workspace.sh mon_nouveau_robot
```

**Puis dans le container :**
```bash
ws mon_nouveau_robot
cd /workspaces/mon_nouveau_robot
ros2 pkg create --build-type ament_python mon_package
bws mon_nouveau_robot
```

## 🔧 Changements importants

### VNC s'ouvre automatiquement
- Le paramètre `"onAutoForward": "openBrowser"` dans [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json:52) fait que VSCode ouvre automatiquement http://localhost:6080 au démarrage du container

### Tous les workspaces montés
- Le dossier `/workspaces/` dans le container contient tous vos projets
- Vous pouvez travailler sur plusieurs projets simultanément sans redémarrer le container

### Helpers automatiquement chargés
- Les fonctions `ws`, `bws`, `lsws` sont automatiquement disponibles dans chaque terminal

## 📦 Volumes Docker

Les build artifacts sont partagés entre workspaces par défaut :
- `ros2_build` : Fichiers compilés
- `ros2_install` : Packages installés
- `ros2_log` : Logs ROS2

Si vous voulez isoler complètement les builds entre projets, modifiez [docker-compose.yml](docker-compose.yml) pour créer des volumes séparés.

## 🐛 Troubleshooting

### Le VNC ne s'ouvre pas automatiquement

1. Vérifiez que le port 6080 n'est pas utilisé : `lsof -i :6080`
2. Ouvrez manuellement : http://localhost:6080/vnc.html?autoconnect=true&resize=scale
3. Vérifiez les ports forwarded dans VSCode (onglet "Ports" en bas)

### Je veux revenir à l'ancien setup

Votre ancien workspace est toujours disponible dans `/Users/durantoine/Dev/ros2_workspace` (sans le dossier workspace qui a été migré).

Pour revenir :
1. Copiez les fichiers de configuration depuis `ros2_devcontainer/`
2. Copiez `workspaces/projet_current/` vers `ros2_workspace/workspace/`

### Erreur "workspace not found"

```bash
# Vérifiez que le workspace existe
ls /workspaces/

# Recréez le lien symbolique
switch_workspace projet_current
```

## 📚 Ressources

- [README.md](README.md) : Documentation complète
- [workspace_helpers.sh](workspace_helpers.sh) : Code des fonctions helper
- [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json) : Configuration du dev container

## 🎯 Prochaines étapes

1. Testez en ouvrant le container
2. Vérifiez que le VNC s'ouvre automatiquement
3. Testez les commandes `lsws`, `ws`, `bws`
4. Créez un nouveau workspace pour votre prochain projet
5. Profitez de ne plus avoir à dupliquer toute la config ! 🎉
