#***********************************************#
#                                               #
#        PROJET INTEGRATION 2022-2023           #
#       Muratory Bastian - Pagés Clément        #
#                                               #
#***********************************************#

#======================================================
#   Launch the script after launching Moveit and Rviz:
#      roslaunch yaskawa_moveit_config demo.launch
#======================================================


from simulation import *


#Init ros node :
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)



robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "yaskawa_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

BOX_NAME_1 = "box1"
BOX_NAME_2 = "box2"
BOX_FLOOR_NAME = "floor"

PARKING_POSE = [0, 0, 3.1415, 0, 0, 0]
BOX_1_POSE = [0.5, 0.5, 0.2, 0, 3.1415, 0]
BOX_2_POSE = [-0.5, -0.5, 0.4, 0, 3.1415, 0]


display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=50
)

if __name__ == '__main__':
    print("START OF THE SIMULATION")

    (planning_frame,eef_link,group_names) = get_robot_status(robot,move_group,display=False)

    #Go to the parking position :
    go_to_target_joints_values(move_group, PARKING_POSE)

    #Add some boxes to the scene :
    add_box(scene, "world", BOX_FLOOR_NAME, 0, 0, -0.06, 5, 5, 0.1)                           #box that represents the floor
    add_box(scene, "world", BOX_NAME_1, BOX_1_POSE[0], BOX_1_POSE[1], 0.1, 0.2, 0.2, 0.2)     #box to pick
    add_box(scene, "world", BOX_NAME_2, BOX_2_POSE[0], BOX_2_POSE[1], 0.1, 0.2, 0.2, 0.2)     #box where we want to place the first box

    #Make the pick and place action:

    #go to the box 1:
    go_to_target(move_group, BOX_1_POSE)
    print(f"robot above the box {BOX_NAME_1}")
    #Pick the box:
    attach_box(scene, robot, eef_link, BOX_NAME_1, "hand")
    #Return to the parking pose:
    go_to_target_joints_values(move_group, PARKING_POSE)
    #Move to the box 1 final position
    go_to_target(move_group, BOX_2_POSE)
    #Place the first box on the second box:
    detach_box(scene, eef_link, BOX_NAME_1)
    #Move the robot to its init pose, at the parking position:
    go_to_target_joints_values(move_group, PARKING_POSE)

    #Remove the boxes from the scene:
    remove_box(scene, BOX_NAME_1)
    remove_box(scene, BOX_NAME_2)
    remove_box(scene, BOX_FLOOR_NAME)

    print("END OF THE SIMULATION")
