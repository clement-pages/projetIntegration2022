# projet Integration 2022
## MURATORY Bastian - PAGES Clément - 3A SRI

## Pré-requis 

L'installation et le lancement du projet décrit ci-après suppose que l'ensemble des dépendances nécessaires ont été préalablement installées (en particulier ROS noetic)

## Installation

Pour installer le répertoire du projet suivre les instructions suivantes :

1. Télécharger l'archive et la décompresser

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

## Lancement de la simulation

Toujours dans le répertoire `catkin_ws` exécuter la commande suivante selon ce que vous souhaitez lancer
* Pour lancer uniquement la simulation dans **Rviz**
```
roslaunch yaskawa_moveit_config demo.launch
```
1. Pour lancer la simulation dans **Rviz** et dans **Gazebo**
```
roslaunch yaskawa_moveit_config demo_gazebo.launch
```
2. Lancer le script python
```
python3 src/test.py
```
