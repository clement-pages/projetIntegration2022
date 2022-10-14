# projet Integration 2022
## MURATORY Bastian - PAGES Clément - 3A SRI

## Pré-requis 

L'installation et le lancement du projet décrit ci-après suppose que l'ensemble des dépendances nécessaires ont été préalablement installées (en particulier ROS noetic)

## Installation

Pour installer le répertoire du projet suivre les instructions suivantes :

1. Télécharger l'archive et la décompresser ou cloner le répertoire

2. Se déplacer dans le répertoire
```
cd projetIntegration2022/catkin_ws
```

3. Sourcer ROS noetic
```
source /opt/ros/noetic/setup.bash
```

4. Builder le projet
```
catkin build
```

5. Sourcer le projet
```
. /devel/setup.bash
```

## Lancement de la simulation Rviz

Toujours dans le répertoire `catkin_ws` exécuter les commandes suivantes pour lancer la simulation dans **Rviz**
1. Lancer la simulation
```
roslaunch yaskawa_moveit_config demo.launch
```
2. Dans un second terminal, sourcer le projet et lancer le script permettant au robot de réaliser une action de pick and place
```
python3 src/script_python/pick_and_place.py
```
Le robot réalise alors une action de pick and place dans la simulation d'un cube ajouté par le script python

## Lancement de la simulation Rviz + Gazebo

Toujours dans le répertoire `catkin_ws` exécuter les commandes suivantes pour lancer la simulation dans **Rviz** et **Gazebo**

1. Lancer la simulation
```
roslaunch yaskawa_moveit_config demo_gazebo.launch
```
2. Dans un second terminal, sourcer et lancer le script python permettant de générer le nuage de point d'une boîte dans la scène
```
python3 src/script_python/gazebot_box.py
```
