# Python 2/3 compatibility imports
from __future__ import print_function
from time import time
from six.moves import input
from typing import Union

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




def get_robot_status(robot : moveit_commander.RobotCommander, move_group : moveit_commander.MoveGroupCommander, display:bool = False) -> tuple:
    """This function allows to get infos about specified robot and move_group. Also,
    the function can optionally print state info in a terminal if `print` argument
    is set to `True`
    ## Parameters :
    * `robot` : robot whose we want know state
    * `move_group` : group whose we want know info
    * `display` : optional. Set to ```True``` to print infos in a terminal. Default value is ```False```
    ## Return value :
    A `tuple` containing planning frame name and end effector name as a string, and groupes' name that take
    part to the specified robot"""

    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the robot:
    if(display):
        print("============ Planning frame: %s" % planning_frame)
        print("============ End effector link: %s" % eef_link)
        print("============ Available Planning Groups:", robot.get_group_names())
        print("============ Printing robot state")
        print(robot.get_current_state())

    return(planning_frame,eef_link,group_names)


def go_to_target(move_group : moveit_commander.MoveGroupCommander, target : Union[list, geometry_msgs.msg.Pose]) -> None:
    """Perform a movement for the specified `move_group` to the indicated `target`
    ## Parameters:
    * `move_group` : robot group to move
    * `target` : target towards the end effector must go, as a list of 6 parameters with cartesian position and 
    rotation for the end effector or a list of seven arguments with cartesain position and four quaternions parameters 
    for rotation, or a Pose message
    ## Return value:
    No return value"""

    move_group.set_pose_target(target)
    move_group.go()
    move_group.stop()
    move_group.clear_pose_targets()


def go_to_target_joints_values(move_group : moveit_commander.MoveGroupCommander, target : list) -> None:
    """Perform a movement for the specified `move_group` to the indicated `targe``given in joint space
    ## Parameters :
    * `move_group` : robot group to move
    * `target` : target is a list that contains goal state for each robot joint
    ## Return value :
    No return value"""

    move_group.go(joints = target)
    move_group.stop()
    move_group.clear_pose_targets()


def cartesian_path(scale : float, move_group : moveit_commander.MoveGroupCommander) -> tuple:
    """Create a basic movement for the arm in a cartesian frame First move up (z-axis), then (y-axis)
    before move forward (x-axis) and finally a second sideway movement (y-axis)
    ## Parameters:
    * `scale` : specified movement amplitude
    * `move_group` : robot group to move
    ##Return value :
    Return  a `tuple` that contains planned path and a fraction of how much of the path was followed """

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


def wait_for_state_update(scene : moveit_commander.PlanningSceneInterface, box_name : str, box_is_known : bool = False, box_is_attached : bool = False, timeout : float = 10) -> bool:
    """Function to wait for the simulation to be updated with added, deleted, attached or detached components
    ## Parameters:
    * `scene` : scene to be updated
    * `box_name` : box name whose we wait to be added on the scene
    * `box_is_know` : boolean to indicate if the box must be known by the scene or not
    * `box_is_attached` : boolean to indicate if the box must be attached with an element in the scene or not
    * `timeout` : float that indicates number of seconds to wait before timeout
    ## Return value
    Return `True` if the scene was successfully update, `False` otherwise
    *"""

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
            print(f"box {box_name} updated")
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()
    print(f"timeout for box {box_name}")
    return False


def add_box(scene : moveit_commander.PlanningSceneInterface, frame_id : str, name : str, x : float, y : float, z : float, width : float, lenght : float, height : float) -> bool:
    """Add a box to the scene at the specified cartesian position and with specified dimensions
    ## Parameters :
    * `scene` : new box will be added to this `PlanningSceneInterface scene`
    * `frame_id` : string representing the frame at which the box will be added
    * `name` : name for the box to add, as a string
    * `x` : coordinate along x-axis for the box cartesian position
    * `y` : coordinate along y-axis for the box cartesian position
    * `z` : coordinate along z-axis for the box cartesian position
    * `width` : box width
    * `length` : box length
    * `height` : box height
    ## Return value :
    `True` if the box was succesfully added to the specified scene, `False` otherwise"""

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = frame_id
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = z
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    scene.add_box(name, box_pose, size=(width, lenght, height ))
    return wait_for_state_update(scene,name,True,False)


def remove_box(scene : moveit_commander.PlanningSceneInterface, name : str) -> bool:
    """Remove box whose `name` is specified in argument from the specified `scene`
    ## Parameters :
    * `scene` : scene where is the box to remove, as `PlanningSceneInterface`
    * `name` : name of the box to be remove, as a string
    ## Return value :
    `True` if the box was the box was succesfully remove from the scene, `False` otherwise
    """

    taille = scene.get_attached_objects()
    #Make sure the box is detached
    scene.remove_world_object(name)
    return wait_for_state_update(scene,name,False,False)


def attach_box(scene : moveit_commander.PlanningSceneInterface, robot : moveit_commander.RobotCommander, eef_link : str, box_name : str, grasping_group : str) -> bool:
    """Attach box whose name is specified in argument to the indicated r`obot` end effector `eef_link`
    ## Paramters :
    * `scene` : scene at which the box must be attached, as a `PlanningSceneInterface`
    * `robot` : robot at which the box must be attached, as `RobotCommander`
    * `eef_link` : robot end effector name, as a string
    * `box_name` : name of the box to be attached
    * `grasping_group` : group at which the bow will be attached. Group must take part of the robot structure definition and correspond to the end effector
    ##Return value :
    `True` if the box was successfully attached to the robot end effector, otherwise `False`
    *"""

    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    print(f"box {box_name} was picked")
    return wait_for_state_update(scene, box_name, box_is_known=False,box_is_attached = True, timeout = 4)


def detach_box(scene : moveit_commander.PlanningSceneInterface, eef_link : str, box_name : str) -> bool:
    """Detach box whose the name is passed in arguments from the specified robot end effector
    ## Parameters :
    * `scene` : scene where is attached the box, as a `PlanningSceneInterface`
    * `eef_link` : end effector name, as a string
    * `box_name` : name of the box to be detached from the robot end effector
    ## Return value :
    `True` if the box was successfully detached from the robot end effector, `False` otherwise"""

    scene.remove_attached_object(eef_link, name=box_name)
    print(f"box {box_name} was placed")
    return wait_for_state_update(scene, box_name, box_is_known=True,box_is_attached = False, timeout = 4)
