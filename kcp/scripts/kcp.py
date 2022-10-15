#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from math import pi, tau, dist, fabs, cos
import tkinter
from tkinter import ttk
import re
from std_msgs.msg import Int32


Color_Background= 'gray12' #'dark slate gray'
Font_Color='snow'
ventana = tkinter.Tk()
ventana.configure(background=Color_Background)
ventana.title("KUKA KCP")

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
        self.pub = rospy.Publisher('clear_whiteboard', Int32, queue_size=1)
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
        print("============ Planning frame: %s" % planning_frame)
       
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")        

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
        self.mov_type = [""] *N_points
        
        self.add_table_fun()

    

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

robot1 = MoveGroupPythonInterface()
velcounter = 5

Options1 = ["PTP","LIN"]
Combo_modo = ttk.Combobox(ventana, value=Options1)
Combo_modo.current(0)

Options2 = ["P1","P2","P3","P4","P5","P6","P7","P8","P9","P10","P11","P12","P13","P14","P15","P16","P17","P18","P19","P20"]
Combo_puntos = ttk.Combobox(ventana, value=Options2)
Combo_puntos.current(0)
points_counter = 0

point_Label = tkinter.Label(ventana, text = "P: 1", width = 30, height = 2, fg= Font_Color, bg= Color_Background)
VelR_Label = tkinter.Label(ventana, text = "Vel.: 25", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
VelR_percentage = 25

def windows_events(event): #------------------------------------------------------------------------------------windows_events
    global points_counter, point_Label, VelR_Label, VelR_percentage
    #--------------Cartesian event
    if event == 'x_mas':        
        robot1.cartesian_pos("x", "forward")
    elif event == 'x_menos':
        robot1.cartesian_pos("x", "reverse")

    elif event == 'y_mas':
        robot1.cartesian_pos("y", "forward")
    elif event == 'y_menos':
        robot1.cartesian_pos("y", "reverse")

    elif event == 'z_mas':
        robot1.cartesian_pos("z", "forward")
    elif event == 'z_menos':
        robot1.cartesian_pos("z", "reverse")

    elif event == 'R_mas':
        robot1.cartesian_pos("R", "forward")
    elif event == 'R_menos':
        robot1.cartesian_pos("R", "reverse")

    elif event == 'P_mas':
        robot1.cartesian_pos("P", "forward")
    elif event == 'P_menos':
        robot1.cartesian_pos("P", "reverse")

    elif event == 'Y_mas':
        robot1.cartesian_pos("Y", "forward")
    elif event == 'Y_menos':
        robot1.cartesian_pos("Y", "reverse")

    #--------------Joints event
    elif event == 'j1_mas':
        robot1.joints_move("j1", "forward")
    elif event == 'j1_menos':
        robot1.joints_move("j1", "reverse")

    elif event == 'j2_mas':
        robot1.joints_move("j2", "forward")
    elif event == 'j2_menos':
        robot1.joints_move("j2", "reverse")

    elif event == 'j3_mas':
        robot1.joints_move("j3", "forward")
    elif event == 'j3_menos':
        robot1.joints_move("j3", "reverse")
    
    elif event == 'j4_mas':
        robot1.joints_move("j4", "forward")
    elif event == 'j4_menos':
        robot1.joints_move("j4", "reverse")
    
    elif event == 'j5_mas':
        robot1.joints_move("j5", "forward")
    elif event == 'j5_menos':
        robot1.joints_move("j5", "reverse")
    
    elif event == 'j6_mas':
        robot1.joints_move("j6", "forward")
    elif event == 'j6_menos':
        robot1.joints_move("j6", "reverse")
    
    #VELOCIDADES
    elif event == 'VR_mas':
        if(VelR_percentage == 100):
            print("maxima velocidad")
        else:
            robot1.change_vel("upscale")
            VelR_percentage = VelR_percentage*2
            text_label_point = "Vel.: "+str(int(VelR_percentage))
            VelR_Label.config(text=text_label_point)

    elif event == 'VR_menos':
        if(int(VelR_percentage) == 1):
            print("minima velocidad")
        else:
            robot1.change_vel("downscale")    
            VelR_percentage = VelR_percentage/2
            text_label_point = "Vel.: "+str(int(VelR_percentage))
            VelR_Label.config(text=text_label_point)
    
    elif event == 'HOME':
        robot1.go_to_home()

    # ---------------- Historial

    elif event == 'guardar':               

        move_group = robot1.move_group       
        joint_goal = move_group.get_current_joint_values()
        wpose = move_group.get_current_pose().pose  
        index_point = int(Combo_puntos.get().replace("P",""))-1

        
        robot1.Joint_points[index_point] = [joint_goal]
        robot1.Cartesian_points[index_point] = [wpose]
        robot1.mov_type[index_point] = Combo_modo.get()
        print("---------------JOINT STATES------------")
        print(robot1.Joint_points)
        print("---------------CARTESIAN DATA------------")
        print(robot1.Cartesian_points)
        print("---------------MOVE TYPE------------")
        print(robot1.mov_type)

        text_label_point = "P: "+str(index_point+1)
        point_Label.config(text=text_label_point)

        
    elif event == 'siguiente':
        print("Punto: ", points_counter + 1)
        points_counter +=1

        if(robot1.mov_type[points_counter] == "PTP"):            
            robot1.joints_move_sequence(points_counter)
        
        elif(robot1.mov_type[points_counter] == "LIN"):            
            robot1.cartesian_sequence(points_counter)

        elif(robot1.mov_type[points_counter] == ""):
            print("No hay un punto guardado")

        text_label_point = "P: "+str(points_counter+1)
        point_Label.config(text=text_label_point)
        
        if points_counter > 20:
            print("No hay mas puntos")
            points_counter -=1
        
    elif event == 'anterior':
        print("Punto: ", points_counter)
        points_counter -=1
        if points_counter < 0:
            print("No hay mas puntos")
            points_counter +=1
        
        if(robot1.mov_type[points_counter] == "PTP"):            
            robot1.joints_move_sequence(points_counter)
        
        elif(robot1.mov_type[points_counter] == "LIN"):            
            robot1.cartesian_sequence(points_counter)

        elif(robot1.mov_type[points_counter] == ""):
            print("No hay un punto guardado")

        text_label_point = "P: "+str(points_counter+1)
        point_Label.config(text=text_label_point)

    elif event == 'start':
        index = 0
        while robot1.mov_type[index] != "":
            
            if(robot1.mov_type[index] == "PTP"):                
                robot1.joints_move_sequence(index)
            
            elif(robot1.mov_type[index] == "LIN"):                
                robot1.cartesian_sequence(index)

            text_label_point = "P: "+str(index+1)        
            point_Label.config(text=text_label_point)
            index += 1

        print("Secuencia Terminada")

    elif event == 'goto':        
        index_point = int(Combo_puntos.get().replace("P",""))-1
        if(robot1.mov_type[index_point] == "PTP"):            
            robot1.joints_move_sequence(index_point)
        
        elif(robot1.mov_type[index_point] == "LIN"):            
            robot1.cartesian_sequence(index_point)

        elif(robot1.mov_type[index_point] == ""):
            print("No hay un punto guardado")

        points_counter = index_point        
        text_label_point = "P: "+str(index_point+1)        
        point_Label.config(text=text_label_point)
    
    elif event == 'delete':        
        index_point = int(Combo_puntos.get().replace("P",""))-1

        robot1.Joint_points[index_point] = None
        robot1.Cartesian_points[index_point] = None
        robot1.mov_type[index_point] = ""

        print("---------------JOINT STATES------------")
        print(robot1.Joint_points)
        print("---------------CARTESIAN DATA------------")
        print(robot1.Cartesian_points)
        print("---------------MOVE TYPE------------")
        print(robot1.mov_type)        

    elif event == 'add_table':
        robot1.add_table_fun()        

    elif event == 'del_table':
        robot1.remove_table_fun()
        robot1.pub.publish(1)        

def main():    
    global point_Label, VelR_Label
    padx_value = 6
    pady_value = 13
    try:

        home_button = tkinter.Button(ventana, text= "HOME", command=lambda: windows_events("HOME"))
        home_button.grid(row=0,column=0, columnspan=1, padx=padx_value, pady=pady_value)

        add_table_button = tkinter.Button(ventana, text= "Add Table", command=lambda: windows_events("add_table"))
        add_table_button.grid(row=13,column=0, columnspan=2, padx=padx_value, pady=pady_value)

        del_table_button = tkinter.Button(ventana, text= "Del. Table", command=lambda: windows_events("del_table"))
        del_table_button.grid(row=13,column=4, columnspan=2, padx=padx_value, pady=pady_value)

        

        Label_x = tkinter.Label(ventana, text = "x", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_xmas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("x_mas"))
        Button_xmenos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("x_menos"))

        Label_y = tkinter.Label(ventana, text = "y", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_ymas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("y_mas"))
        Button_ymenos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("y_menos"))

        Label_z = tkinter.Label(ventana, text = "z", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_zmas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("z_mas"))
        Button_zmenos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("z_menos"))

        # roll pich yaw

        Label_Roll = tkinter.Label(ventana, text = "R", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_Rmas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("R_mas"))
        Button_Rmenos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("R_menos"))

        Label_Pitch = tkinter.Label(ventana, text = "P", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_Pmas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("P_mas"))
        Button_Pmenos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("P_menos"))

        Label_Yaw = tkinter.Label(ventana, text = "Y", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_Ymas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("Y_mas"))
        Button_Ymenos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("Y_menos"))

        # joints

        Label_joint1 = tkinter.Label(ventana, text = "J1", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_joint1_mas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("j1_mas"))
        Button_joint1_menos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("j1_menos"))

        Label_joint2 = tkinter.Label(ventana, text = "J2", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_joint2_mas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("j2_mas"))
        Button_joint2_menos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("j2_menos"))

        Label_joint3 = tkinter.Label(ventana, text = "J3", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_joint3_mas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("j3_mas"))
        Button_joint3_menos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("j3_menos"))

        Label_joint4 = tkinter.Label(ventana, text = "J4", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_joint4_mas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("j4_mas"))
        Button_joint4_menos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("j4_menos"))

        Label_joint5 = tkinter.Label(ventana, text = "J5", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_joint5_mas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("j5_mas"))
        Button_joint5_menos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("j5_menos"))

        Label_joint6 = tkinter.Label(ventana, text = "J6", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
        Button_joint6_mas = tkinter.Button(ventana, text= "+", command=lambda: windows_events("j6_mas"))
        Button_joint6_menos = tkinter.Button(ventana, text= "-", command=lambda: windows_events("j6_menos"))

        # velocidades        

        
        VelR_mas_Button = tkinter.Button(ventana, text= "+", command=lambda: windows_events("VR_mas"))
        VelR_menos_Button = tkinter.Button(ventana, text= "-", command=lambda: windows_events("VR_menos"))

        VelR_Label.grid(row=0,column=3, columnspan=1, padx=padx_value, pady=pady_value)
        VelR_mas_Button.grid(row=0,column=4, columnspan=1, padx=padx_value, pady=pady_value)
        VelR_menos_Button.grid(row=0,column=5, columnspan=1, padx=padx_value, pady=pady_value)

        # movimiento

        historial_Label = tkinter.Label(ventana, text = "HISTORIAL", width = 30, height = 2, fg= Font_Color, bg= Color_Background)
        historial_Label.grid(row=7,column=0, columnspan=6, padx=padx_value, pady=pady_value)

        Combo_modo.bind("<<ComboboxSelected>>",windows_events)
        Combo_puntos.bind("<<ComboboxSelected>>",windows_events)

        Combo_modo.grid(row=8,column=3, columnspan=3, padx=padx_value, pady=pady_value)        
        Combo_puntos.grid(row=8,column=0, columnspan=3, padx=padx_value, pady=pady_value)

        Guardar_Button = tkinter.Button(ventana, text= "SAVE POINT", command=lambda: windows_events("guardar"))
        Guardar_Button.grid(row=9,column=0, columnspan=2, padx=padx_value, pady=pady_value)

        goto_Button = tkinter.Button(ventana, text= "GO TO POINT", command=lambda: windows_events("goto"))
        goto_Button.grid(row=9,column=2, columnspan=2, padx=padx_value, pady=pady_value)

        delete_Button = tkinter.Button(ventana, text= "DELETE POINT", command=lambda: windows_events("delete"))
        delete_Button.grid(row=9,column=4, columnspan=2, padx=padx_value, pady=pady_value)

        secuencia_Label = tkinter.Label(ventana, text = "SECUENCIA", width = 30, height = 2, fg= Font_Color, bg= Color_Background)
        secuencia_Label.grid(row=10,column=0, columnspan=6, padx=padx_value, pady=pady_value)

        siguiente_Button = tkinter.Button(ventana, text= "NEXT", command=lambda: windows_events("siguiente"))
        siguiente_Button.grid(row=11,column=4, columnspan=2, padx=padx_value, pady=pady_value)

        anterior_Button = tkinter.Button(ventana, text= "PREV", command=lambda: windows_events("anterior"))
        anterior_Button.grid(row=11,column=0, columnspan=2, padx=padx_value, pady=pady_value)

        
        point_Label.grid(row=11,column=2, columnspan=2, padx=padx_value, pady=pady_value)

        start_Button = tkinter.Button(ventana, text= "START", command=lambda: windows_events("start"))
        start_Button.grid(row=12,column=2, columnspan=2, padx=padx_value, pady=pady_value)

        """ export_Button = tkinter.Button(ventana, text= "EXPORT", command=lambda: windows_events("export"))
        export_Button.grid(row=12,column=2, columnspan=2, padx=padx_value, pady=pady_value)

        import_Button = tkinter.Button(ventana, text= "CLEAR", command=lambda: windows_events("clear_whiteboard"))
        import_Button.grid(row=12,column=4, columnspan=2, padx=padx_value, pady=pady_value) """

        # positions        

        
        # Combo_modo.grid(row=0,column=2, columnspan=3, padx=padx_value, pady=pady_value)

        Label_x.grid(row=1,column=0, columnspan=1, padx=padx_value, pady=pady_value)
        Button_xmas.grid(row=1,column=1, columnspan=1, padx=padx_value, pady=pady_value)
        Button_xmenos.grid(row=1,column=2, columnspan=1, padx=padx_value, pady=pady_value)

        Label_y.grid(row=2,column=0, columnspan=1, padx=padx_value, pady=pady_value)
        Button_ymas.grid(row=2,column=1, columnspan=1, padx=padx_value, pady=pady_value)
        Button_ymenos.grid(row=2,column=2, columnspan=1, padx=padx_value, pady=pady_value)

        Label_z.grid(row=3,column=0, columnspan=1, padx=padx_value, pady=pady_value)
        Button_zmas.grid(row=3,column=1, columnspan=1, padx=padx_value, pady=pady_value)
        Button_zmenos.grid(row=3,column=2, columnspan=1, padx=padx_value, pady=pady_value)

        Label_Roll.grid(row=4,column=0, columnspan=1, padx=padx_value, pady=pady_value)
        Button_Rmas.grid(row=4,column=1, columnspan=1, padx=padx_value, pady=pady_value)
        Button_Rmenos.grid(row=4,column=2, columnspan=1, padx=padx_value, pady=pady_value)

        Label_Pitch.grid(row=5,column=0, columnspan=1, padx=padx_value, pady=pady_value)
        Button_Pmas.grid(row=5,column=1, columnspan=1, padx=padx_value, pady=pady_value)
        Button_Pmenos.grid(row=5,column=2, columnspan=1, padx=padx_value, pady=pady_value)

        Label_Yaw.grid(row=6,column=0, columnspan=1, padx=padx_value, pady=pady_value)
        Button_Ymas.grid(row=6,column=1, columnspan=1, padx=padx_value, pady=pady_value)
        Button_Ymenos.grid(row=6,column=2, columnspan=1, padx=padx_value, pady=pady_value)

        Label_joint1.grid(row=1,column=3, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint1_mas.grid(row=1,column=4, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint1_menos.grid(row=1,column=5, columnspan=1, padx=padx_value, pady=pady_value)

        Label_joint2.grid(row=2,column=3, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint2_mas.grid(row=2,column=4, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint2_menos.grid(row=2,column=5, columnspan=1, padx=padx_value, pady=pady_value)

        Label_joint3.grid(row=3,column=3, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint3_mas.grid(row=3,column=4, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint3_menos.grid(row=3,column=5, columnspan=1, padx=padx_value, pady=pady_value)

        Label_joint4.grid(row=4,column=3, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint4_mas.grid(row=4,column=4, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint4_menos.grid(row=4,column=5, columnspan=1, padx=padx_value, pady=pady_value)

        Label_joint5.grid(row=5,column=3, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint5_mas.grid(row=5,column=4, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint5_menos.grid(row=5,column=5, columnspan=1, padx=padx_value, pady=pady_value)

        Label_joint6.grid(row=6,column=3, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint6_mas.grid(row=6,column=4, columnspan=1, padx=padx_value, pady=pady_value)
        Button_joint6_menos.grid(row=6,column=5, columnspan=1, padx=padx_value, pady=pady_value)       

        #loop
        ventana.mainloop()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()