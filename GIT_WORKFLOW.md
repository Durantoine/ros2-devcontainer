# Git Workflow pour ROS2 Dev Container

## 🎯 Concept : Chaque workspace = 1 repo Git indépendant

```
ros2_devcontainer/              # Configuration Docker (ce repo)
├── .git/                       # Git pour la config Docker
├── docker-compose.yml
├── Dockerfile
└── workspaces/
    ├── projet_current/         # 🔹 Repo Git séparé #1
    │   ├── .git/
    │   ├── src/
    │   ├── build/              # Ignoré par Git
    │   ├── install/            # Ignoré par Git
    │   └── log/                # Ignoré par Git
    └── mon_robot/              # 🔹 Repo Git séparé #2
        ├── .git/
        └── src/
```

**Point clé** : Les dossiers sont synchronisés en temps réel entre Mac et Container
- **Mac** : `/Users/durantoine/Dev/ros2_devcontainer/workspaces/`
- **Container** : `/workspaces/`
- **= MÊME DOSSIER**

## 🆕 Créer un nouveau workspace

**Sur votre Mac :**
```bash
cd /Users/durantoine/Dev/ros2_devcontainer/workspaces
mkdir mon_robot && cd mon_robot
mkdir src

# Créer .gitignore
cat > .gitignore << 'EOF'
build/
install/
log/
__pycache__/
*.py[cod]
*.o
*.so
CMakeFiles/
EOF

# Initialiser Git
git init
git add .
git commit -m "Initial ROS2 workspace"
git remote add origin git@github.com:votre-user/mon-robot.git
git push -u origin main
```

**Dans le Container (terminal VS Code) :**
```bash
bws mon_robot
```

## 🔄 Workflow quotidien

```bash
# 1️⃣ Sur Mac - Créer une branche
cd /Users/durantoine/Dev/ros2_devcontainer/workspaces/mon_robot
git checkout -b feature/navigation

# 2️⃣ Dans VS Code - Éditer le code (automatiquement synchronisé)

# 3️⃣ Dans le Container - Builder et tester
bws mon_robot
ros2 run mon_package mon_node

# 4️⃣ Sur Mac - Commiter
git add .
git commit -m "Add navigation feature"
git push origin feature/navigation

# 5️⃣ Sur GitHub - Créer une Pull Request
```

## ⚠️ À ne PAS commiter

Déjà dans `.gitignore` :
- ❌ `build/` - Artefacts de compilation
- ❌ `install/` - Fichiers installés
- ❌ `log/` - Logs ROS2

## 📥 Cloner un workspace existant

```bash
# Sur Mac
cd /Users/durantoine/Dev/ros2_devcontainer/workspaces
git clone git@github.com:votre-user/mon-projet.git

# Dans le Container
bws mon-projet
```

## ❓ FAQ

**Où faire les commandes Git ?**
→ Sur votre Mac (pas dans le container)

**Où éditer le code ?**
→ Dans VS Code (qui tourne dans le container)

**Où builder ?**
→ Dans le container avec `bws mon_robot`

**Les changements sont synchronisés ?**
→ Oui, instantanément entre Mac et Container
