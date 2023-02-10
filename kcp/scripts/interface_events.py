#!/usr/bin/env python3

from geometry_msgs.msg import Point
import rospy
import connection_moveit
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import pickle
import os
from pathlib import Path
import curve

#rospy.init_node('interfaz', anonymous=True) # Al importar moveit_kr6 se comparte su nodo
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

class Events1:
    def __init__(self, group):

        self.debug_prints = True

        self.pub = rospy.Publisher('position_list', Point, queue_size=1)
        self.pub_cleaner = rospy.Publisher('custom_joint_states', JointState, queue_size=1)
        self.robot1 = connection_moveit.MoveGroupPythonInterface(group)
        self.move_type = "Ejes"
        self.set_home = True
        
        pointmessaje = Point(-1, 0, 0)
        self.pub.publish(pointmessaje)
        absolute_path = os.path.dirname(__file__)
        folder_path = "/sequences/"
        
        self.total_path = absolute_path + folder_path
        print(bcolors.WARNING+"SEQUENCES PATH: "+self.total_path)

    def movement(self, event):  #-----------------------------------------movement
        if self.debug_prints: print(bcolors.HEADER+"INTERFACE EVENTS,"+bcolors.ENDC+" movement:", event)
        #--------------Cartesian event-------------
        if self.move_type == "Mundo":
            if event == 'j1_mas':        
                self.robot1.cartesian_pos("x", "forward")
            elif event == 'j1_menos':
                self.robot1.cartesian_pos("x", "reverse")

            elif event == 'j2_mas':
                self.robot1.cartesian_pos("y", "forward")
            elif event == 'j2_menos':
                self.robot1.cartesian_pos("y", "reverse")

            elif event == 'j3_mas':
                self.robot1.cartesian_pos("z", "forward")
            elif event == 'j3_menos':
                self.robot1.cartesian_pos("z", "reverse")

            elif event == 'j4_mas':
                self.robot1.cartesian_pos("R", "forward")
            elif event == 'j4_menos':
                self.robot1.cartesian_pos("R", "reverse")

            elif event == 'j5_mas':
                self.robot1.cartesian_pos("P", "forward")
            elif event == 'j5_menos':
                self.robot1.cartesian_pos("P", "reverse")

            elif event == 'j6_mas':
                self.robot1.cartesian_pos("Y", "forward")
            elif event == 'j6_menos':
                self.robot1.cartesian_pos("Y", "reverse")
        #--------------Joints event-------------
        elif self.move_type == "Ejes":
            if event == 'j1_mas':
                self.robot1.joints_move("j1", "forward")
            elif event == 'j1_menos':
                self.robot1.joints_move("j1", "reverse")

            elif event == 'j2_mas':
                self.robot1.joints_move("j2", "forward")
            elif event == 'j2_menos':
                self.robot1.joints_move("j2", "reverse")

            elif event == 'j3_mas':
                self.robot1.joints_move("j3", "forward")
            elif event == 'j3_menos':
                self.robot1.joints_move("j3", "reverse")
            
            elif event == 'j4_mas':
                self.robot1.joints_move("j4", "forward")
            elif event == 'j4_menos':
                self.robot1.joints_move("j4", "reverse")
            
            elif event == 'j5_mas':
                self.robot1.joints_move("j5", "forward")
            elif event == 'j5_menos':
                self.robot1.joints_move("j5", "reverse")
            
            elif event == 'j6_mas':
                self.robot1.joints_move("j6", "forward")
            elif event == 'j6_menos':
                self.robot1.joints_move("j6", "reverse")

    def velocity(self, event, percentage): #-----------------------------------------velocity
        if self.debug_prints: print(bcolors.HEADER+"INTERFACE EVENTS,"+bcolors.ENDC+" velocity:", event)
        if event == 'VR_mas':
            self.robot1.change_vel("upscale", percentage)

        elif event == 'VR_menos':
            self.robot1.change_vel("downscale", percentage)

    def step_size(self, event): #-----------------------------------------step_size
        if self.debug_prints: print(bcolors.HEADER+"INTERFACE EVENTS,"+bcolors.ENDC+" step_size:", event)
        if event == 'step_mas':
            self.robot1.change_stepsize("upscale")

        elif event == 'step_menos':
            self.robot1.change_stepsize("downscale")

    def history(self, event, index_point, move_type="", plano = "", positions=[],joints_circ1 = None):  #-----------------------------------------history
        if self.debug_prints: print(bcolors.HEADER+"INTERFACE EVENTS,"+bcolors.ENDC+" history:", event)
        if event == 'guardar':
            if move_type == "LIN" or move_type == "PTP":
                move_group = self.robot1.move_group
                joint_goal = move_group.get_current_joint_values()
                wpose = move_group.get_current_pose().pose

                self.robot1.Joint_points[index_point] = [joint_goal]
                self.robot1.Cartesian_points[index_point] = [wpose]
                self.robot1.mov_type[index_point] = move_type

                xvalor= 1 # 1 añadir | 0 eliminar
                yvalor= index_point # Posicion

                if self.robot1.mov_type[index_point] == 'PTP':
                    zvalor = 1
                else:
                    zvalor = 2

                pointmessaje = Point(xvalor, yvalor, zvalor)
                self.pub.publish(pointmessaje)
            else:
                self.robot1.mov_type[index_point] = move_type #circ or shape 
                self.robot1.Joint_points[index_point] = joints_circ1 # first position
                self.robot1.Cartesian_points[index_point] = [positions, plano] # 3 or 2 points | xy, xz or yz
                xvalor= 1 # 1 añadir | 0 eliminar
                yvalor= index_point # Posicion                
                if self.robot1.mov_type[index_point] == 'CIRC':
                    zvalor = 3
                else:
                    zvalor = 4

                pointmessaje = Point(xvalor, yvalor, zvalor)
                self.pub.publish(pointmessaje)

        elif event == 'goto':
            if(self.robot1.mov_type[index_point] == "PTP"):
                self.robot1.joints_move_sequence(index_point)
            
            elif(self.robot1.mov_type[index_point] == "LIN"):
                self.robot1.cartesian_sequence(index_point)
                #self.robot1.joints_move_sequence(index_point)
                # los dos van a move joints porque cartesian no siempre funciona.
                # los dejo separados por si en alguna actualizacion se arregla y remplazarlo.

            elif(self.robot1.mov_type[index_point] == "CIRC"):                                
                move_type = self.robot1.mov_type[index_point]#circ or shape 
                joints_circ1 = self.robot1.Joint_points[index_point]# first position
                positions, plano = self.robot1.Cartesian_points[index_point] # 3 or 2 points | xy, xz or yz
                """ print("joints_circ1",joints_circ1)"""
                print("positions", positions) 
                
                """ if plano == "XY": other_list = [(pos.pop(2)) for pos in positions]
                elif plano == "XZ": other_list = [(pos.pop(1)) for pos in positions]
                elif plano == "YZ": other_list = [(pos.pop(0)) for pos in positions]
                print(positions[0],positions[1],positions[2]) """
                
                list_circ_positions = curve.curve(positions[0],positions[1],positions[2])
                print(list_circ_positions)
                """ self.robot1.joints_move_sequence(0,joints_circ1)
                for pos in list_circ_positions:
                    self.robot1.cartesian_sequence(0, pos) """
                
            elif(self.robot1.mov_type[index_point] == "SHAPE"):              
                print("SHAPE")

            elif(self.robot1.mov_type[index_point] == None):
                print(bcolors.WARNING+"No hay un punto guardado")
        
        elif event == 'delete':

            self.robot1.Joint_points[index_point] = None
            self.robot1.Cartesian_points[index_point] = None
            self.robot1.mov_type[index_point] = None

            xvalor= 0 # 1 añadir | 0 eliminar
            yvalor= index_point # Posicion
            zvalor = 0
        
            pointmessaje = Point(xvalor, yvalor, zvalor)
            self.pub.publish(pointmessaje)

    def running(self, event, index_point=0): #-----------------------------------------running
        if self.debug_prints: print(bcolors.HEADER+"INTERFACE EVENTS,"+bcolors.ENDC+" running:", event)
        if event == 'start':        
            index = 0
            self.robot1.go_to_home()            
            rate = rospy.Rate(10)
            for save_point in self.robot1.mov_type:
                
                if save_point != None:
                    # Se muestra el actual en plot points
                    xvalor= 2 # 1 añadir | 0 eliminar | 2 running
                    yvalor= index # Posicion
                    zvalor=0
                    pointmessaje = Point(xvalor, yvalor, zvalor)
                    self.pub.publish(pointmessaje)
                    rate.sleep()

                    # Se mueve el robot
                    if(save_point == "PTP"):
                        self.robot1.joints_move_sequence(index)
                    
                    elif(save_point == "LIN"):
                        self.robot1.cartesian_sequence(index)
                    
                    xvalor= 1 # 1 añadir | 0 eliminar | 2 running
                    yvalor= index # Posicion
                    if(save_point == "PTP"): zvalor = 1
                    elif(save_point == "LIN"): zvalor = 2
                    pointmessaje = Point(xvalor, yvalor, zvalor)
                    self.pub.publish(pointmessaje)
                    rate.sleep()
                
                index += 1

            self.robot1.go_to_home()
            print(bcolors.OKGREEN+"Secuencia Terminada")
        
        elif event == 'siguiente':
            if(self.robot1.mov_type[index_point] == "PTP"):
                self.robot1.joints_move_sequence(index_point)
            elif(self.robot1.mov_type[index_point] == "LIN"):
                self.robot1.cartesian_sequence(index_point)
            elif(self.robot1.mov_type[index_point] == None):
                print(bcolors.WARNING+"No hay un punto guardado")
            
        elif event == 'anterior':
            if(self.robot1.mov_type[index_point] == "PTP"):
                self.robot1.joints_move_sequence(index_point)
            
            elif(self.robot1.mov_type[index_point] == "LIN"):
                self.robot1.cartesian_sequence(index_point)

            elif(self.robot1.mov_type[index_point] == None):
                print(bcolors.WARNING+"No hay un punto guardado")  

    def general(self, event): #-----------------------------------------general
        if self.debug_prints: print(bcolors.HEADER+"INTERFACE EVENTS,"+bcolors.ENDC+" general:", event)

        if event == 'get_pos_mute':
            joint_list, cartesian_position, cartesian_orientation = self.robot1.get_position()
            return joint_list, cartesian_position, cartesian_orientation

        if event == 'get_pos':            
            joint_list, cartesian_position, cartesian_orientation = self.robot1.get_position()

            #For Tool
            if self.robot1.tool_spawned:
                value_max = max(self.robot1.tool_dimensions)
                index_max = self.robot1.tool_dimensions.index(value_max)
                cartesian_position[index_max] += value_max

            print(bcolors.OKBLUE+"ROBOT CURRENT POSITION:")
            print(bcolors.OKBLUE+"Joints list:", joint_list)
            cartesian_position = [round(x,4) for x in cartesian_position]
            print(bcolors.OKBLUE+"Cartesian (x,y,z):", cartesian_position)
            cartesian_orientation = [round(x,4) for x in cartesian_orientation]
            print(bcolors.OKBLUE+"Cartesian (R,P,Y):", cartesian_orientation)
            print(bcolors.ENDC+"")

        if event == "set_position_on": self.set_home = True
        
        if event == "move_position_on": self.set_home = False
            

        if self.set_home:            
            if event == 'HOME':
                joint_list, cartesian_position, cartesian_orientation = self.robot1.get_position()
                self.robot1.main_positions_list[0] = joint_list
            if event == 'pos1': 
                joint_list, cartesian_position, cartesian_orientation = self.robot1.get_position()
                self.robot1.main_positions_list[1] = joint_list
            if event == 'pos2': 
                joint_list, cartesian_position, cartesian_orientation = self.robot1.get_position()
                self.robot1.main_positions_list[2] = joint_list
            if event == 'pos3': 
                joint_list, cartesian_position, cartesian_orientation = self.robot1.get_position()
                self.robot1.main_positions_list[3] = joint_list
            if event == 'pos4': 
                joint_list, cartesian_position, cartesian_orientation = self.robot1.get_position()
                self.robot1.main_positions_list[4] = joint_list
            if event == 'pos5': 
                joint_list, cartesian_position, cartesian_orientation = self.robot1.get_position()
                self.robot1.main_positions_list[5] = joint_list
            
        else:
            if event == 'HOME': self.robot1.go_to_home()
            if event == 'pos1': self.robot1.go_to_pos(1)
            if event == 'pos2': self.robot1.go_to_pos(2)
            if event == 'pos3': self.robot1.go_to_pos(3)
            if event == 'pos4': self.robot1.go_to_pos(4)
            if event == 'pos5': self.robot1.go_to_pos(5)
        
        if event == "clear_whiteboard":
            hello_str = JointState()
            hello_str.header = Header()
            hello_str.header.stamp = rospy.Time.now()
            hello_str.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
            hello_str.position = [11, 0, 0, 0, 0]
            hello_str.velocity = []
            hello_str.effort = []
            self.pub_cleaner.publish(hello_str)

        elif event == 'add_table':
            self.robot1.add_table_fun()

        elif event == 'del_table':
            self.robot1.remove_table_fun()

        # ----
        elif event == 'add_tool':
            self.robot1.add_tool_fun()

        elif event == 'del_tool':
            self.robot1.remove_tool_fun()
    
    def advanced_functions(self, event, data = None):
        if self.debug_prints: print(bcolors.HEADER+"INTERFACE EVENTS, "+bcolors.ENDC+"advanced_functions:", event)

        if event == "update_table":
            self.robot1.table_position = data[0]
            self.robot1.table_dimensions = data[1]
            self.robot1.add_table_fun()
        
        if event =="get_table_dim":
            return [self.robot1.table_position, self.robot1.table_dimensions]

        if event == "update_tool":
            self.robot1.tool_position = data[0]
            self.robot1.tool_dimensions = data[1]
            self.robot1.add_tool_fun()
        
        if event =="get_tool_dim":
            return [self.robot1.tool_position ,self.robot1.tool_dimensions]
    
    def offline_data(self, event, filename): # Import, Export
        if self.debug_prints: print(bcolors.HEADER+"INTERFACE EVENTS, "+bcolors.ENDC+"offline_data:", event)

        if event == "import":
            pik_file = self.total_path + filename + ".dat"
            with open(pik_file, "rb") as f:
                try:
                    rate = rospy.Rate(10)
                    pointmessaje = Point(-1, 0, 0)
                    self.pub.publish(pointmessaje)
                    rate.sleep()
                    
                    list_data = pickle.load(f)
                    self.robot1.Joint_points = list_data[0][0]
                    self.robot1.Cartesian_points = list_data[0][1]
                    self.robot1.mov_type = list_data[0][2]
                    self.robot1.main_positions_list = list_data[1]
                    [self.robot1.table_position, self.robot1.table_dimensions] = list_data[2]
                    [self.robot1.tool_position ,self.robot1.tool_dimensions] = list_data[3]

                    for index, pos in enumerate(self.robot1.mov_type):
                        if pos != None:
                            xvalor= 1 # 1 añadir | 0 eliminar
                            yvalor= index # Posicion

                            if pos == 'PTP':
                                zvalor = 1
                            else:
                                zvalor = 2

                            pointmessaje = Point(xvalor, yvalor, zvalor)
                            self.pub.publish(pointmessaje)
                            rate.sleep()
                    
                    print(bcolors.OKGREEN+"SUCCESSFULLY IMPORTED")
                except EOFError:
                    pass

        elif event == "export":
            pik_file = self.total_path + filename + ".dat"
            my_file = Path(pik_file)
            i=0
            if my_file.is_file():
                i = 1
                while my_file.is_file():

                    pik_file = self.total_path + filename + "(" +str(i) + ")" + ".dat"
                    my_file = Path(pik_file)
                    i+=1

      
            with open(pik_file, "wb") as f:
                history_data= [self.robot1.Joint_points, self.robot1.Cartesian_points, self.robot1.mov_type]
                table_data = [self.robot1.table_position, self.robot1.table_dimensions]
                tool_data = [self.robot1.tool_position ,self.robot1.tool_dimensions]
                list_data = [history_data, self.robot1.main_positions_list, table_data, tool_data]
                pickle.dump(list_data, f)
                if i == 0:
                    print(bcolors.OKGREEN+"SUCCESSFULLY EXPORTED AS: " +bcolors.ENDC+ filename+ ".dat")
                else:
                    print(bcolors.OKGREEN+"SUCCESSFULLY EXPORTED AS: " +bcolors.ENDC+ filename+ "(" +str(i-1) + ")" + ".dat")
    
    