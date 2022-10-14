from Fonctions import *
# Ce script doit être lancé après gazebo:
# roslaunch yaskawa_moveit_config demo_gazebo.launch 
import os
import time

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)


robot = moveit_commander.RobotCommander()
# variable internes du robot (position des joints)

scene = moveit_commander.PlanningSceneInterface()
# info sur l'environement connu du robot (permet de rajouter des elements dans le monde)

group_name = "yaskawa_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

BOX_NAME_1 = "boite1"

PARKING = [0, 0, 3.1415, 0, 0, 0] # move_group.get_current_joint_values()

(planning_frame,eef_link,group_names) = get_robot_status(robot,move_group,display=False)

vue1 = [1.9829302118916168, -0.22905570628213479, 1.4608830537862474, 0.7570236094355414, -1.9461455883741818, -1.7851170322729715]
vue2 = [-0.7939154686524033, -0.24929951638884518, 0.9373244355103125, -0.12233043352488604, -1.5333189821275361, 2.4327421546564487]
vue3 = [-0.39042068724932477, 0.12867781815322843, 1.0703092919598953, -0.6139778380972114, -1.4666487831505286, 2.2046507304087584]
vue4 = [0.024478329740491134, 0.8975454547180837, 2.1693865926194427, 2.4205564077567203, 2.2405453562601743, -1.1342359308572743]


pose_exploration_cube = []

pose_droite_cube = [0.018704078113794154, 0.8942304579370797, 2.1722334964954886, 2.423272052680254, 2.2376457388086894, -1.1341980019273201]

pose_gauche_cube = [-1.293647128505997, -0.6039819278349512, 5.709298203713587, 0.33714556979278765, 2.1206931969062897, 0.20128487539278428]


(planning_frame,eef_link,group_names) = get_robot_status(robot,move_group,display=True)
