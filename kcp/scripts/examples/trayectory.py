#! /usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_python', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("kuka6")
display_trayectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

group_varialbe_values = group.get_current_joint_values()

group_varialbe_values[0] = 0
group_varialbe_values[1] = 1.5
group_varialbe_values[2] = 0
group_varialbe_values[3] = -1.5
group_varialbe_values[4] = 0
group_varialbe_values[5] = 0
group.set_joint_value_target(group_varialbe_values)


plan2 = group.plan()
group.go(wait=True)

moveit_commander.roscpp_shutdown()