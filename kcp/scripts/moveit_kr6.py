#!/usr/bin/env python3

""" from __future__ import print_function
from six.moves import input """

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from math import tau, dist, fabs, cos

def all_close(goal, actual, tolerance):
    
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveGroupPythonInterface(object):    

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface", anonymous=True)        
        robot = moveit_commander.RobotCommander()        
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "kuka6"
        move_group = moveit_commander.MoveGroupCommander(group_name)       
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
       
        planning_frame = move_group.get_planning_frame()       
        eef_link = move_group.get_end_effector_link()        
        group_names = robot.get_group_names()       
        current_state = robot.get_current_state()    

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.scale_ROT = tau/16
        self.scale_LIN = 0.1

        N_points = 100
        self.Joint_points = [None] * N_points
        self.Cartesian_points = [None] * N_points
        self.mov_type = [None] *N_points
        
        #self.add_table_fun()

    def go_to_draw(self):
        move_group = self.move_group       
        joint_goal = move_group.get_current_joint_values()        

        joint_goal[0] = 0
        joint_goal[1] = -tau/4
        joint_goal[2] = tau/4
        joint_goal[3] = 0
        joint_goal[4] = tau/4
        joint_goal[5] = 0

        move_group.go(joint_goal, wait=True)
        
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_home(self):
        move_group = self.move_group       
        joint_goal = move_group.get_current_joint_values()        

        joint_goal[0] = 0
        joint_goal[1] = -tau/4
        joint_goal[2] = tau/4
        joint_goal[3] = 0
        joint_goal[4] = 0 #tau/4
        joint_goal[5] = 0

        move_group.go(joint_goal, wait=True)
        
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def joints_move(self, joint, direction):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        scale_ROT = self.scale_ROT

        if joint == "j1":
            if direction == "forward":
                joint_goal[0] += scale_ROT 
            elif direction == "reverse":
                joint_goal[0] -= scale_ROT
        elif joint == "j2":
            if direction == "forward":
                joint_goal[1] += scale_ROT 
            elif direction == "reverse":
                joint_goal[1] -= scale_ROT
        elif joint == "j3":
            if direction == "forward":
                joint_goal[2] += scale_ROT
            elif direction == "reverse":
                joint_goal[2] -= scale_ROT
        elif joint == "j4":
            if direction == "forward":
                joint_goal[3] += scale_ROT 
            elif direction == "reverse":
                joint_goal[3] -= scale_ROT
        elif joint == "j5":
            if direction == "forward":
                joint_goal[4] += scale_ROT 
            elif direction == "reverse":
                joint_goal[4] -= scale_ROT
        elif joint == "j6":
            if direction == "forward":
                joint_goal[5] += scale_ROT 
            elif direction == "reverse":
                joint_goal[5] -= scale_ROT

        move_group.go(joint_goal, wait=True)
        
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
   
    def cartesian_pos(self, axis, direction):
        move_group = self.move_group
        waypoints = []
        scale_LIN = self.scale_LIN
        scale_ROT = self.scale_ROT
        wpose = move_group.get_current_pose().pose

        if axis == "x":
            if direction == "forward":
                wpose.position.x += scale_LIN
            elif direction == "reverse":
                wpose.position.x -= scale_LIN
        elif axis == "y":
            if direction == "forward":
                wpose.position.y += scale_LIN
            elif direction == "reverse":
                wpose.position.y -= scale_LIN
        elif axis == "z":
            if direction == "forward":
                wpose.position.z += scale_LIN
            elif direction == "reverse":
                wpose.position.z -= scale_LIN

        elif axis == "R":
            print("roll")
            if direction == "forward":
                wpose.orientation.x += scale_ROT
            elif direction == "reverse":
                wpose.orientation.x -= scale_ROT
        elif axis == "P":
            print("pitch")
            if direction == "forward":
                wpose.orientation.y += scale_ROT
            elif direction == "reverse":
                wpose.orientation.y -= scale_ROT
        elif axis == "Y":
            print("yaw")
            if direction == "forward":
                wpose.orientation.z += scale_ROT
            elif direction == "reverse":
                wpose.orientation.z -= scale_ROT


        waypoints.append(copy.deepcopy(wpose))  
        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan, wait=True)

        return plan, fraction

    def joints_move_sequence(self, n):

        move_group = self.move_group       
        joint_goal = self.Joint_points[n][0]
        move_group.go(joint_goal, wait=True)        
        move_group.stop()
        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def cartesian_sequence(self, n):
        move_group = self.move_group
        waypoints = []
        wpose = self.Cartesian_points[n][0]
        waypoints.append(copy.deepcopy(wpose))  
        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan, wait=True)

        return plan, fraction
    
    def change_vel(self, operation):
        if operation == "upscale":
            self.scale_LIN = self.scale_LIN*2
            self.scale_ROT = self.scale_ROT*2

        elif operation == "downscale":
            self.scale_LIN = self.scale_LIN/2
            self.scale_ROT = self.scale_ROT/2

    def add_table_fun(self):        
        scene = self.scene
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.6
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.1
        self.box_name = "box"
        scene.add_box(self.box_name, box_pose, size=(0.75, 1, 0.05))
          
    def remove_table_fun(self):        
        box_name = self.box_name
        scene = self.scene        
        scene.remove_world_object(box_name)        
        
    #END CLASS

