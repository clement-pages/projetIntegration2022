from Fonctions import *
from pcg_gazebo.simulation import create_object


# Ce script doit être lancé après moveit:
# roslaunch yaskawa_moveit_config demo.launch 



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

(planning_frame,eef_link,group_names) = get_robot_status(robot,move_group,print=False)

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=50
)

move_group.go(PARKING,wait=True)
move_group.stop()

add_box(scene, "world", "floor", 0, 0, -0.06, 5, 5, 0.1)

add_box(scene, "world", "boite1", 0.5, 0.5, 0.1, 0.2, 0.2, 0.2) # boite a saisir
add_box(scene, "world", "boite2", -0.5, -0.5, 0.1, 0.2, 0.2, 0.2)

pose_goal = new_goal_pose_message(0.5, 0.5, 0.5 , 0.5, 0.5, -0.5, -0.5)
move_group.set_pose_target(pose_goal)
move_group.go()
move_group.stop()
## It is always good to clear your targets after planning with poses.
## Note: there is no equivalent function for clear_joint_value_targets().
move_group.clear_pose_targets()
print("robot au dessus de la boite")

# attraper la boite
attach_box(scene, robot, eef_link, "boite1", "hand")

# TODO remplacer parking par looping (peutere rajouter une boite genante sur le chemin)
move_group.go(PARKING)
move_group.stop()

# TODO aller a une autre position 
pose_goal = new_goal_pose_message(-0.5, -0.5, 0.7 , 0.5, 0.5, -0.5, -0.5)
move_group.set_pose_target(pose_goal)
move_group.go()
move_group.stop()

# 
detach_box(scene, eef_link, "boite1")

move_group.go(PARKING)
move_group.stop()

remove_box(scene, "boite1")
remove_box(scene, "boite2")
remove_box(scene, "floor")
