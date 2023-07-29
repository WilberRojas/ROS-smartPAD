#!/usr/bin/env python3

#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from math import tau, dist, fabs, cos
import time
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class MoveGroupPythonInterface(object):    

    def __init__(self, group):
        self.debug_prints = True

        super(MoveGroupPythonInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ros_smartpad", anonymous=True)
        robot = moveit_commander.RobotCommander()        
        scene = moveit_commander.PlanningSceneInterface()
       
        self.group_name = group
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)       
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )        
       
        planning_frame = self.move_group.get_planning_frame()       
        eef_link = self.move_group.get_end_effector_link()        
        group_names = robot.get_group_names()       
        current_state = robot.get_current_state()    

        # Misc variables
        self.box_name = ""
        self.tool_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = self.move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.scale_ROT = tau/16
        self.scale_LIN = 0.25

        N_points = 100
        self.Joint_points = [None] * N_points
        self.Cartesian_points = [None] * N_points
        self.mov_type = [None] *N_points
        
        # Main positions
        self.template_position = [10,10,10,10,10,10]
        self.main_positions_list = []
        
        i=1
        number_main_positions = 6
        while i<=number_main_positions: 
            self.main_positions_list.append(self.template_position)
            i += 1
        
        self.main_positions_list[0] = self.template_position
        self.main_positions_list[1] = self.template_position
        self.main_positions_list[2] = self.template_position
        self.main_positions_list[3] = self.template_position
        self.main_positions_list[4] = self.template_position
        self.main_positions_list[5] = self.template_position

        # Default Table dimensions and position
        self.table_dimensions = [1.005, 1.505, 0.05]
        self.table_position = [1.0, 0.0, 0.26]

        # Default Tool Dimensions
        self.tool_dimensions = [0.5, 0.2, 0.2]
        self.tool_position = [0.25, 0.0, 0.0]

        #self.tool_axis = 0 # x:0 ; y:1 ; z:2
        self.tool_spawned = False


    def get_position(self):
        #if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" get_position ")
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values() #joints
        wpose = move_group.get_current_pose().pose #cartesian
        joint_list = []
        for index, data in enumerate(joint_goal):            
            joint_list.append(round(data,4))

        cartesian_position = [wpose.position.x,  wpose.position.y,  wpose.position.z]
        cartesian_orientation = [wpose.orientation.x,  wpose.orientation.y,  wpose.orientation.z, wpose.orientation.w]

        return joint_list, cartesian_position, cartesian_orientation

    def go_to_pos(self, pos):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" go_to_pos ")
        move_group = self.move_group       
        joint_goal = move_group.get_current_joint_values()
        joins_pos = []
        if pos == 1: joins_pos = self.main_positions_list[1]
        elif pos == 2: joins_pos = self.main_positions_list[2]
        elif pos == 3: joins_pos = self.main_positions_list[3]
        elif pos == 4: joins_pos = self.main_positions_list[4]
        elif pos == 5: joins_pos = self.main_positions_list[5]

        if joins_pos == self.template_position:
            print(bcolors.WARNING+"No hay un punto guardado")
        else: 

            joint_goal = joins_pos
            move_group.go(joint_goal, wait=True)
            move_group.stop()
   
    def go_to_home(self):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" go_to_home ")
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()   

        if self.main_positions_list[0] == self.template_position:
            print(bcolors.WARNING+"No hay un punto HOME guardado")     
        else:
            joint_goal = self.main_positions_list[0]
            
            move_group.go(joint_goal, wait=True)
            
            move_group.stop()

    def joints_move(self, joint, direction):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" joints_move ")
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
   
    def cartesian_pos(self, axis, direction):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" cartesian_pos ")
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
            if direction == "forward":
                wpose.orientation.x += scale_ROT
            elif direction == "reverse":
                wpose.orientation.x -= scale_ROT
        elif axis == "P":            
            if direction == "forward":
                wpose.orientation.y += scale_ROT
            elif direction == "reverse":
                wpose.orientation.y -= scale_ROT
        elif axis == "Y":            
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

    def joints_move_sequence(self, index = 0, joints = None):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" joints_move_sequence ")
        move_group = self.move_group
        print("J: ",joints)
        if joints == None: 
            joint_goal = self.Joint_points[index][0]
            print(joint_goal)
        else: 
            joint_goal = joints
            print(joint_goal)

        
        move_group.go(joint_goal, wait=True)
        move_group.stop()

    def cartesian_sequence(self, index = 0, cartesian = None):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" cartesian_sequence ")
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        if cartesian == None:
            wpose = self.Cartesian_points[index][0]
            print(wpose.position.x)
        else:
            wpose.position.x = cartesian[0]
            wpose.position.y = cartesian[1]

        waypoints.append(copy.deepcopy(wpose))
        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        move_group.execute(plan, wait=True)

        return plan, fraction

    def change_stepsize(self, operation):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" change_stepsize ")
        if operation == "upscale":
            self.scale_LIN = self.scale_LIN*2
            self.scale_ROT = self.scale_ROT*2

        elif operation == "downscale":
            self.scale_LIN = self.scale_LIN/2
            self.scale_ROT = self.scale_ROT/2
    
    def change_vel(self, operation, percentage):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" change_vel ")
        if operation == "upscale":
            self.move_group.set_max_velocity_scaling_factor(percentage/100)

        elif operation == "downscale":
            self.move_group.set_max_velocity_scaling_factor(percentage/100)

    def add_table_fun(self):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" add_table_fun ")
        scene = self.scene
  
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = self.table_position[0] + self.table_dimensions[0]/2 # For centering
        box_pose.pose.position.y = self.table_position[1]
        box_pose.pose.position.z = self.table_position[2]
        self.box_name = "Table"
        scene.add_box(self.box_name, box_pose, size=(self.table_dimensions[0], self.table_dimensions[1], self.table_dimensions[2]))
          
    def remove_table_fun(self):    
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" remove_table_fun ")    
        box_name = self.box_name
        scene = self.scene        
        scene.remove_world_object(box_name)

    #--------------------TOOL
    def add_tool_fun(self):        
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" add_tool_fun ")
        self.tool_spawned = True
        scene = self.scene
  
        tool_pose = geometry_msgs.msg.PoseStamped()
        tool_pose.header.frame_id = "base_link"
        

        end_efector_position = self.get_position()

        tool_pose.pose.position.x = end_efector_position[1][0] + self.tool_position[0]
        tool_pose.pose.position.y = end_efector_position[1][1] + self.tool_position[1]
        tool_pose.pose.position.z = end_efector_position[1][2] + self.tool_position[2]
        #tool_pose.pose.orientation.x = end_efector_position[2][0]
        #tool_pose.pose.orientation.y = end_efector_position[2][1]
        #tool_pose.pose.orientation.z = end_efector_position[2][2]
        #tool_pose.pose.orientation.w = end_efector_position[2][3]
        self.tool_name = "Tool"
        scene.add_box(self.tool_name, tool_pose, size=(self.tool_dimensions[0], self.tool_dimensions[1], self.tool_dimensions[2]))

        touch_links = self.robot.get_link_names(group=self.group_name)
        time.sleep(0.1)
        scene.attach_box("tool0", self.tool_name, touch_links=touch_links)
          
    def remove_tool_fun(self):
        if self.debug_prints: print(bcolors.HEADER+"connection_moveit:"+bcolors.ENDC+" remove_tool_fun ")
        self.tool_spawned = False
        scene = self.scene
        scene.remove_attached_object("tool0", name=self.tool_name)
        time.sleep(0.1)
        scene.remove_world_object(self.tool_name)
        
        
    #END CLASS

robot = MoveGroupPythonInterface("irb4400")

joint_list, cartesian_position, cartesian_orientation = robot.get_position()

print("Ángulos:", joint_list)

print("Posición (X,Y,Z):", cartesian_position)

print("Orientación (R, P, Y):", cartesian_orientation)

""" robot.scale_LIN = 0.3 # metros
robot.cartesian_pos("x", "reverse")

robot.scale_LIN = 1.5
robot.cartesian_pos("y", "forward")

robot.scale_LIN = 1.0
robot.cartesian_pos("x", "forward")

robot.scale_LIN = 1.5
robot.cartesian_pos("y", "reverse") """

robot.scale_ROT = 0.77
robot.joints_move("j1", "forward")