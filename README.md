# 🏥 HospiBot - Système de Navigation Robotique Hospitalière

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)](https://classic.gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10%2B-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-yellow)](LICENSE)

**HospiBot** est un système complet de navigation robotique autonome conçu pour guider les patients et visiteurs dans les établissements hospitaliers. Le projet combine robotique mobile (ROS2), simulation (Gazebo), interfaces utilisateur (Android & Web) et intelligence artificielle pour offrir une solution moderne de navigation hospitalière.

---

## 📑 Table des Matières

- [Aperçu](#aperçu)
- [Architecture](#architecture)
- [Fonctionnalités](#fonctionnalités)
- [Prérequis](#prérequis)
- [Installation](#installation)
- [Configuration](#configuration)
- [Utilisation](#utilisation)
- [API REST](#api-rest)
- [Interfaces Utilisateur](#interfaces-utilisateur)
- [Développement](#développement)
- [Dépannage](#dépannage)
- [Contributions](#contributions)
- [Licence](#licence)
- [Équipe](#équipe)

---

## 🎯 Aperçu

### Problématique

Les grands établissements hospitaliers font face à des défis récurrents :
- **Désorientation** des patients et visiteurs dans des infrastructures complexes
- **Retards** dans les rendez-vous médicaux
- **Surcharge** du personnel d'accueil
- **Coûts opérationnels** élevés en personnel d'orientation

### Solution

HospiBot offre un robot mobile autonome capable de :
- ✅ Guider les patients vers leur destination (8 services hospitaliers)
- ✅ Naviguer de manière autonome en évitant les obstacles
- ✅ Être contrôlé via une application mobile Android ou une interface web
- ✅ Cartographier l'environnement en temps réel (SLAM)
- ✅ Gérer des points de navigation personnalisés (waypoints)

### Démonstration

```
🏥 Hôpital (24m × 24m)
├── 📍 Réception (0, -8)
├── 🚨 Urgences (-8, -8)
├── 💊 Pharmacie (8, -8)
├── 🔬 Laboratoire (-8, 8)
├── 👨‍⚕️ Consultation 1 (-8, 0)
├── 👩‍⚕️ Consultation 2 (8, 0)
├── 🧑‍⚕️ Consultation 3 (0, 8)
└── 🚪 Entrée (8, 8)
```

---

## 🏗️ Architecture

### Vue d'Ensemble

```
┌─────────────────────────────────────────────────────┐
│         INTERFACES UTILISATEUR                       │
│  ┌──────────────────┐    ┌──────────────────┐      │
│  │  📱 Android App  │    │  🌐 Web Interface│      │
│  │  - User Mode     │    │  - Admin Panel   │      │
│  │  - Admin Mode    │    │  - Monitoring    │      │
│  └────────┬─────────┘    └────────┬─────────┘      │
└───────────┼──────────────────────┼─────────────────┘
            │                       │
            │   HTTP/REST (JSON)    │
            │   Port 5000           │
            │                       │
┌───────────▼───────────────────────▼─────────────────┐
│              FLASK API SERVER                        │
│  - 15 REST Endpoints                                 │
│  - WebSocket (temps réel)                            │
│  - Authentication & Session Management               │
│  - ROS2 Bridge (rclpy)                              │
└──────────────────────┬──────────────────────────────┘
                       │
                       │   ROS2 Topics/Services
                       │   (DDS Middleware)
                       │
┌──────────────────────▼──────────────────────────────┐
│              ROS2 HUMBLE ECOSYSTEM                   │
│  ┌────────────────────────────────────────────────┐ │
│  │  Navigation Stack (Nav2)                       │ │
│  │  - Global Planner (Dijkstra/A*)                │ │
│  │  - Local Planner (DWB Controller)              │ │
│  │  - Costmaps (Global + Local)                   │ │
│  │  - Recovery Behaviors                          │ │
│  └────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────┐ │
│  │  SLAM Toolbox                                  │ │
│  │  - Synchronous SLAM                            │ │
│  │  - Loop Closure Detection                      │ │
│  │  - Map Serialization (PGM + YAML)             │ │
│  └────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────┐ │
│  │  Robot State Management                        │ │
│  │  - TF Tree (base_link, odom, map)             │ │
│  │  - Joint States                                │ │
│  │  - URDF Model (TurtleBot3 Burger)             │ │
│  └────────────────────────────────────────────────┘ │
└──────────────────────┬──────────────────────────────┘
                       │
                       │   Gazebo Plugins
                       │
┌──────────────────────▼──────────────────────────────┐
│              GAZEBO 11 SIMULATOR                     │
│  - Physics Engine (ODE)                              │
│  - 3D Visualization                                  │
│  - Sensor Simulation (LiDAR 360°)                   │
│  - Hospital World (24×24m, 8 rooms, obstacles)      │
└─────────────────────────────────────────────────────┘
```

### Stack Technologique

| Composant | Technologie | Version | Usage |
|-----------|-------------|---------|-------|
| **Middleware** | ROS2 | Humble | Communication robot |
| **Simulation** | Gazebo | 11.x | Environnement 3D |
| **Robot** | TurtleBot3 | Burger | Plateforme mobile |
| **Navigation** | Nav2 | Latest | Navigation autonome |
| **SLAM** | SLAM Toolbox | Latest | Cartographie |
| **Backend** | Flask + ROS2 | 2.3 + rclpy | API REST |
| **Mobile** | Android | 7.0+ (API 24+) | Interface patient/admin |
| **Web** | HTML5 + JavaScript | ES6+ | Interface monitoring |
| **Language** | Python | 3.10+ | Scripts ROS2 |

---

## ✨ Fonctionnalités

### 🤖 Fonctionnalités Robotiques (ROS2)

#### Navigation Autonome
- ✅ Planification de trajectoire globale (Dijkstra, A*)
- ✅ Évitement d'obstacles en temps réel (DWB Controller)
- ✅ Navigation vers 8 destinations prédéfinies
- ✅ Support waypoints personnalisés
- ✅ Recovery behaviors (rotation, backup)

#### SLAM (Cartographie)
- ✅ Cartographie temps réel avec SLAM Toolbox
- ✅ Loop closure detection
- ✅ Sauvegarde/chargement de cartes (PGM + YAML)
- ✅ Relocalisation sur carte existante

#### Capteurs & Actionneurs
- ✅ LiDAR 360° (5 Hz, portée 12m)
- ✅ Odométrie (50 Hz, précision ±2cm)
- ✅ Commandes vitesse différentielle (cmd_vel)
- ✅ IMU pour orientation (optionnel)

#### Contrôle
- ✅ Contrôle manuel 9 directions (↑↓←→↖↗↙↘ + STOP)
- ✅ Ajustement vitesses (linéaire: 0.1-2.0 m/s, angulaire: 0.1-3.0 rad/s)
- ✅ Mode SLAM / Mode Navigation (toggle)
- ✅ Annulation navigation en temps réel

### 📱 Interfaces Utilisateur

#### Application Android (Mobile)
- **Mode Utilisateur:**
  - Sélection destination (grille 2×4)
  - Suivi navigation en temps réel
  - Notifications arrivée
  - Interface Material Design 3

- **Mode Administrateur:**
  - Dashboard complet avec métriques temps réel
  - Contrôle manuel du robot
  - Visualisation carte 2D
  - Gestion waypoints
  - Mode SLAM/Nav2 toggle

#### Interface Web (Dashboard Admin)
- Monitoring temps réel
- Visualisation trajectoire
- Logs système
- Configuration paramètres

---

## 🛠️ Prérequis

### Système d'Exploitation

- **Ubuntu 22.04 LTS (Jammy Jellyfish)** - Recommandé
- Alternative: Ubuntu 20.04 LTS (Focal Fossa)

### Logiciels Requis

```bash
# ROS2 Humble
ros2 --version  # humble

# Gazebo 11
gazebo --version  # 11.x

# Python 3.10+
python3 --version  # 3.10.x
```

### Matériel Recommandé

- **CPU:** Intel i5 / AMD Ryzen 5 ou supérieur (4 cores)
- **RAM:** 8 GB minimum, 16 GB recommandé
- **GPU:** Pas obligatoire, mais améliore Gazebo
- **Stockage:** 10 GB espace libre
- **Réseau:** WiFi pour connexion Android

---

## 📥 Installation

### Étape 1: Installer ROS2 Humble

```bash
# Configuration locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Ajouter repository ROS2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Installer ROS2
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop-full

# Outils développement
sudo apt install ros-dev-tools

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Étape 2: Installer Dépendances

```bash
# Gazebo 11
sudo apt install gazebo11 ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# TurtleBot3
sudo apt install ros-humble-turtlebot3*

# Autres packages ROS2
sudo apt install \
  ros-humble-tf2-tools \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2

# Python packages
pip3 install flask flask-cors rclpy
```

### Étape 3: Cloner le Projet

```bash
# Créer workspace
mkdir -p ~/hospibot_ws/src
cd ~/hospibot_ws/src

# Cloner repository
git clone https://github.com/votre-repo/hospibot.git

# Ou créer package manuellement
ros2 pkg create hospibot \
  --build-type ament_cmake \
  --dependencies rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs \
                 tf2 tf2_ros gazebo_ros navigation2 slam_toolbox
```

### Étape 4: Structure du Projet

```bash
cd ~/hospibot_ws/src/hospibot

# Créer structure de dossiers
mkdir -p config description/urdf description/meshes launch maps scripts worlds

# Télécharger URDF TurtleBot3 (si nécessaire)
cd description/urdf
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/humble-devel/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro
```

### Étape 5: Build Workspace

```bash
cd ~/hospibot_ws

# Source ROS2
source /opt/ros/humble/setup.bash

# Build avec colcon
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Ajouter au bashrc
echo "source ~/hospibot_ws/install/setup.bash" >> ~/.bashrc
```

---

## ⚙️ Configuration

### Variables d'Environnement

```bash
# Ajouter au ~/.bashrc
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/hospibot_ws/src/hospibot/description
export ROS_DOMAIN_ID=30  # Optionnel, pour isoler réseau ROS2
```

### Configuration Réseau (pour connexion Android/Web)

```bash
# Récupérer IP de la machine
ip addr show

# Exemple: 10.10.5.182
# Configurer dans android/Constants.java:
# public static final String BASE_URL = "http://10.10.5.182:5000/";
```

### Fichiers de Configuration

Les fichiers de configuration sont dans `config/`:

- `nav2_params.yaml` - Paramètres Navigation2
- `slam_params.yaml` - Paramètres SLAM Toolbox
- `costmap_common.yaml` - Configuration costmaps
- `local_costmap.yaml` - Costmap local
- `global_costmap.yaml` - Costmap global

---

## 🚀 Utilisation

### Lancement Rapide (3 terminaux)

#### Terminal 1: Gazebo + Robot

```bash
cd ~/hospibot_ws
source install/setup.bash

# Lancer simulation
ros2 launch hospibot gazebo.launch.py
```

**Résultat attendu:**
- Fenêtre Gazebo s'ouvre
- Monde hospitalier chargé (8 salles)
- Robot TurtleBot3 au centre (0, 0)

#### Terminal 2: Flask API Server

```bash
cd ~/hospibot_ws
source install/setup.bash

# Lancer API
python3 src/hospibot/scripts/flask_api.py
```

**Résultat attendu:**
```
🤖 HospiBot Flask API started
📡 Listening on http://0.0.0.0:5000
🏥 Hospital navigation system ready!
```

#### Terminal 3: Tests (Optionnel)

```bash
# Health check
curl http://localhost:5000/health

# Contrôle manuel
curl -X POST http://localhost:5000/control \
  -H "Content-Type: application/json" \
  -d '{"command":"forward","linear_speed":0.3,"angular_speed":0.0}'

# Navigation vers Réception
curl -X POST http://localhost:5000/goto_location \
  -H "Content-Type: application/json" \
  -d '{"location":"reception","mode":"nav2"}'
```

### Lancement avec Navigation2

```bash
# Terminal 1: Gazebo
ros2 launch hospibot gazebo.launch.py

# Terminal 2: Navigation2
ros2 launch hospibot navigation.launch.py

# Terminal 3: Flask API
python3 src/hospibot/scripts/flask_api.py
```

### Lancement avec SLAM

```bash
# Terminal 1: Gazebo
ros2 launch hospibot gazebo.launch.py

# Terminal 2: SLAM Toolbox
ros2 launch hospibot slam.launch.py

# Terminal 3: Flask API
python3 src/hospibot/scripts/flask_api.py
```

### Visualisation RViz2

```bash
# Lancer RViz2
ros2 run rviz2 rviz2

# Ou avec config personnalisée
ros2 launch hospibot rviz.launch.py
```

---

## 📡 API REST

### Base URL

```
http://localhost:5000
```

### Endpoints Disponibles

#### Contrôle Robot

```http
POST /control
Content-Type: application/json

{
  "command": "forward|backward|left|right|forward_left|forward_right|backward_left|backward_right|stop",
  "linear_speed": 0.5,
  "angular_speed": 1.0
}
```

#### Récupérer Données

```http
GET /data?mode=slam

Response:
{
  "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
  "velocity": 0.0,
  "laser": {
    "ranges": [12.0, 11.8, ..., 12.0],
    "angle_min": -3.14159,
    "angle_max": 3.14159,
    ...
  }
}
```

#### Navigation

```http
POST /goto_location
Content-Type: application/json

{
  "location": "reception|emergency|pharmacy|lab|consult1|consult2|consult3|entrance",
  "mode": "nav2"
}
```

### Documentation Complète

Liste complète des 15 endpoints disponibles dans [API_DOCUMENTATION.md](docs/API_DOCUMENTATION.md).

---

## 🖥️ Interfaces Utilisateur

### Application Android

**Fonctionnalités principales:**
- Mode utilisateur pour patients/visiteurs
- Mode administrateur pour gestion avancée
- Communication temps réel avec le robot
- Visualisation carte et LiDAR

**Technologies:** Java 8, Material Design 3, Retrofit 2.9.0

**Installation:** APK disponible dans `releases/` ou via Google Play (futur)

**Documentation:** [ANDROID_README.md](android/README.md)

### Interface Web

**Fonctionnalités principales:**
- Dashboard administrateur
- Monitoring temps réel
- Configuration système
- Logs et diagnostics

**Technologies:** HTML5, JavaScript ES6, WebSocket

**Accès:** `http://localhost:5000/web` (après lancement Flask API)

**Documentation:** [WEB_README.md](web/README.md)

---

## 👨‍💻 Développement

### Topics ROS2 Utilisés

| Topic | Type | Fréquence | Direction | Description |
|-------|------|-----------|-----------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Variable | Pub | Commandes vitesse |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | Sub | Odométrie robot |
| `/scan` | `sensor_msgs/LaserScan` | 5 Hz | Sub | Données LiDAR |
| `/goal_pose` | `geometry_msgs/PoseStamped` | On-demand | Pub | Goal navigation |
| `/map` | `nav_msgs/OccupancyGrid` | 1 Hz | Sub | Carte SLAM |
| `/tf` | `tf2_msgs/TFMessage` | 50 Hz | Sub | Transformations |

### Commandes Utiles

```bash
# Lister topics
ros2 topic list

# Echo topic
ros2 topic echo /odom

# Publier manuellement
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Lister nodes
ros2 node list

# Infos node
ros2 node info /hospibot_api_node

# Visualiser TF tree
ros2 run tf2_tools view_frames
```

### Structure du Code

```
hospibot/
├── config/              # Fichiers de configuration (YAML)
├── description/         # URDF, Xacro, Meshes
├── launch/              # Launch files Python
├── maps/                # Cartes sauvegardées (PGM, YAML)
├── scripts/             # Scripts Python
│   ├── flask_api.py           # API REST + ROS2 Node
│   ├── robot_controller.py    # Contrôleur robot
│   └── waypoint_manager.py    # Gestion waypoints
├── worlds/              # Mondes Gazebo (SDF)
├── CMakeLists.txt       # Configuration build
└── package.xml          # Métadonnées package
```

### Tests

```bash
# Tests unitaires Python
pytest tests/

# Tests ROS2
colcon test

# Vérifier qualité code
flake8 scripts/
pylint scripts/
```

---

## 🔧 Dépannage

### Gazebo ne démarre pas

```bash
# Vérifier installation
gazebo --version

# Réinstaller si nécessaire
sudo apt install --reinstall gazebo11
```

### Erreur "Package 'hospibot' not found"

```bash
# Rebuild workspace
cd ~/hospibot_ws
colcon build --symlink-install
source install/setup.bash
```

### Robot ne bouge pas

```bash
# Vérifier /cmd_vel publié
ros2 topic echo /cmd_vel

# Vérifier connection Gazebo
ros2 topic list | grep cmd_vel

# Publier manuellement pour tester
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}}" --once
```

### Flask API ne démarre pas

```bash
# Vérifier Python packages
pip3 install --upgrade flask flask-cors rclpy

# Vérifier ROS2 sourcé
source /opt/ros/humble/setup.bash
source ~/hospibot_ws/install/setup.bash
```

### Android ne se connecte pas

```bash
# Vérifier IP machine
ip addr show

# Vérifier port 5000 ouvert
sudo ufw allow 5000

# Tester depuis Android (via Terminal)
curl http://<IP_MACHINE>:5000/health
```

### Problèmes Performance Gazebo

```bash
# Réduire qualité graphique
export GAZEBO_MODEL_PATH=""
gazebo --verbose

# Désactiver ombres dans world file
<cast_shadows>false</cast_shadows>
```

---

## 📚 Documentation Additionnelle

- **ROS2 Humble:** https://docs.ros.org/en/humble/
- **Gazebo 11:** https://classic.gazebosim.org/
- **Nav2:** https://navigation.ros.org/
- **SLAM Toolbox:** https://github.com/SteveMacenski/slam_toolbox
- **TurtleBot3:** https://emanual.robotis.com/docs/en/platform/turtlebot3/

---

## 🤝 Contributions

Les contributions sont les bienvenues ! Veuillez suivre ces étapes :

1. Fork le projet
2. Créer une branche (`git checkout -b feature/AmazingFeature`)
3. Commit les changements (`git commit -m 'Add AmazingFeature'`)
4. Push la branche (`git push origin feature/AmazingFeature`)
5. Ouvrir une Pull Request

### Guidelines

- Suivre PEP 8 pour Python
- Documenter toutes les fonctions
- Ajouter tests unitaires
- Mettre à jour README si nécessaire

---

## 📄 Licence

Ce projet est sous licence **Apache 2.0**. Voir [LICENSE](LICENSE) pour plus de détails.

```
Copyright 2026 Équipe HospitalBot

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

---

## 👥 Équipe

### Développeurs

- **NGOMA Fortune** - Chef de projet
- **SIDIBE Lacina** - Développement robotique
- **ASSI Marc-Aurèle Stéphane** - Développement Android
- **N'DRI Hans Samuel** - Intégration système

### Superviseur

- **[Nom du superviseur]** - Encadrant technique

### Contact

- **Email:** team@hospibot.com
- **GitHub:** https://github.com/hospibot
- **Documentation:** https://hospibot.readthedocs.io

---

## 🏆 Remerciements

- **ROBOTIS** pour TurtleBot3
- **Open Robotics** pour ROS2 et Gazebo
- **Steve Macenski** pour SLAM Toolbox et Nav2
- **Communauté ROS** pour le support

---

## 📊 Statistiques Projet

- **Lignes de code:** ~10,000+
  - Python (ROS2): 3,000
  - Java (Android): 5,000
  - JavaScript (Web): 1,000
  - XML/YAML/SDF: 1,000+

- **Fichiers:** 100+
- **Commits:** 150+
- **Durée développement:** 8 semaines
- **Contributors:** 4

---

## 🗺️ Roadmap

### Version 1.0 (Actuelle) ✅
- [x] Simulation Gazebo complète
- [x] Navigation autonome Nav2
- [x] Cartographie SLAM
- [x] API REST Flask
- [x] Application Android
- [x] Interface Web basique

### Version 1.1 (Q2 2026) 🚧
- [ ] WebSocket temps réel
- [ ] Persistance données (SQLite/Firebase)
- [ ] Multi-langues (FR, EN, AR)
- [ ] Mode nuit interface
- [ ] Logs avancés

### Version 2.0 (Q3 2026) 📋
- [ ] Multi-robots (flotte)
- [ ] IA pour optimisation trajectoire
- [ ] Vision par ordinateur (caméra)
- [ ] Intégration système rendez-vous hôpital
- [ ] Déploiement robot physique

### Version 3.0 (Q4 2026) 💡
- [ ] Reconnaissance vocale
- [ ] Réalité augmentée (AR)
- [ ] Apprentissage par renforcement
- [ ] Cloud deployment (AWS/Azure)

---

## 📸 Screenshots

### Gazebo Simulation
![Gazebo Hospital World](docs/images/gazebo_world.png)

### Android App
![Android User Dashboard](docs/images/android_user.png)
![Android Admin Dashboard](docs/images/android_admin.png)

### RViz2 Visualization
![RViz SLAM](docs/images/rviz_slam.png)

---

## 🎓 Publications & Présentations

- **Rapport Technique:** [HospiBot_Rapport_Final.pdf](docs/HospiBot_Rapport_Final.pdf)
- **Cahier des Charges:** [CahierDesCharges_HospiBot.pdf](docs/CahierDesCharges_HospiBot.pdf)
- **Présentation:** [HospiBot_Presentation.pptx](docs/HospiBot_Presentation.pptx)

---

## ⭐ Star History

[![Star History Chart](https://api.star-history.com/svg?repos=votre-repo/hospibot&type=Date)](https://star-history.com/#votre-repo/hospibot&Date)

---

<div align="center">

**Fait avec ❤️ par l'équipe HospitalBot**

[⬆ Retour en haut](#-hospibot---système-de-navigation-robotique-hospitalière)

</div>
