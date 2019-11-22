#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import Grasp

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "panda_arm"
group_arm = moveit_commander.MoveGroupCommander(group_name)
group_hand = moveit_commander.MoveGroupCommander("hand")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

# We can get the name of the reference frame for this robot:
planning_frame = group_arm.get_planning_frame()
print("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = group_arm.get_end_effector_link()
print("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the
# robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

waypoints = []
scale = 1.0

wpose = group_arm.get_current_pose().pose

# wpose.position.z -= scale * 0.1  # First move up (z)
p = wpose.position
print("x: %s" % p.x)
print("y: %s" % p.y)
print("z: %s" % p.z)

def setInitialPosition(wpose):
    wpose.position.x = 0.307019568623
    wpose.position.y = -1.3734595264e-08
    wpose.position.z = 0.590269549206

# initial position
# x: 0.307019568623
# y: -1.3734595264e-08
# z: 0.590269549206

setInitialPosition(wpose)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y += scale * 0.2  # and sideways (y)
waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.2  # Third move sideways (y)
wpose.position.x -= scale * 0.1  # Second move forward/backwards in (x)
waypoints.append(copy.deepcopy(wpose))

# initial position
wpose.position.x = 0.30701956862
wpose.position.y = 0.2
wpose.position.z = 0.2
waypoints.append(copy.deepcopy(wpose))

# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
(plan, fraction) = group_arm.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

group_arm.execute(plan, wait=True)

group_hand.set_named_target('open')
group_hand.go()
group_hand.set_named_target('close')
group_hand.go()
