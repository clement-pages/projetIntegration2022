# Python 2/3 compatibility imports
from __future__ import print_function
from time import time
from xmlrpc.client import Boolean
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



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

display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=50
)

def get_robot_status(robot,move_group,print:bool = False) -> tuple:
    """This function allows to get infos about specified robot and move_group. Also,
    the function can optionally print state info in a terminal if ```print``` argument
    is set to ```True```
    ## Parameters :
    * robot [```in```] : robot whose we want know state
    * move_group [```in```] : group whose we want know info
    * print [```in```] : optional. Set to ```True``` to print infos in a terminal. Default value is ```False```
    
    ## Return value :
    A tuple containing planning frame name and end effector name as a string, and groupes' name that take
    part to the specified robot"""

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the robot:
    if(print):
        print("============ Planning frame: %s" % planning_frame)
        print("============ End effector link: %s" % eef_link)
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())

    return(planning_frame,eef_link,group_names)

(planning_frame,eef_link,group_names) = get_robot_status(robot,move_group,print=False)

def new_goal_pose_message(x : float,y : float, z : float, qw : float, qx : float, qy : float, qz : float) -> geometry_msgs.msg.Pose:
    """Define a new goal Pose message for the end effector, in operational coordinates specified in arguments
    (position and orientation)"""
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = qw
    pose_goal.orientation.x = qx
    pose_goal.orientation.y = qy
    pose_goal.orientation.z = qz

    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    return pose_goal


def cartesian_path(scale:int):
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
    )  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

def wait_for_state_update(scene, box_name, box_is_known=False,box_is_attached = False, timeout = 10):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            print("box "+box_name+" updated\n")
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()
    print("timeout for box "+box_name+"\n")
    return False


def add_box(scene, frame_id:str, name:str, x:float, y:float, z:float, width:float, lenght:float, height:float):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame_id
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = z
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    scene.add_box(name, box_pose, size=(width, lenght, height ))
    return wait_for_state_update(scene,name,True,False)

def remove_box(scene, name):
    taille = scene.get_attached_objects()
    #Make sure the box is detached
    scene.remove_world_object(name)
    return wait_for_state_update(scene,name,False,False)

def attach_box(scene, robot, eef_link, box_name : str, grasping_group : str):
    """"""
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    print("boite attrapee")
    return wait_for_state_update(scene, box_name, box_is_known=False,box_is_attached = True, timeout = 4)

def detach_box(scene, eef_link, box_name):
    """"""
    scene.remove_attached_object(eef_link, name=box_name)
    return wait_for_state_update(scene, box_name, box_is_known=True,box_is_attached = False, timeout = 4)

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




'''
(plan,fraction) = cartesian_path(scale=1)
# Affichage 
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory)

move_group.execute(plan, wait=True)
'''

# robot_end_effector_pose = robot.get_group("yaskawa_arm").get_current_pose()
