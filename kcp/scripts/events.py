#!/usr/bin/env python3


from geometry_msgs.msg import Point
import rospy
import moveit_kr6
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import pickle
import os

#rospy.init_node('interfaz', anonymous=True) # Al importar moveit_kr6 se comparte su nodo



class Events1:
    def __init__(self):

        self.pub = rospy.Publisher('position_list', Point, queue_size=1)
        self.pub_cleaner = rospy.Publisher('custom_joint_states', JointState, queue_size=1)
        self.robot1 = moveit_kr6.MoveGroupPythonInterface()
        self.move_type = "Ejes"
        
        pointmessaje = Point(-1, 0, 0)      
        self.pub.publish(pointmessaje)
        absolute_path = os.path.dirname(__file__)
        folder_path = "/sequences/"
        self.file_name = "sequence.dat"
        self.total_path = absolute_path + folder_path
        print("total_path: "+self.total_path)
        self.movements_file = os.path.join(self.total_path+self.file_name)




    def windows(self, event):
        print("falta configurar:", event)

    def movement(self, event):  #-----------------------------------------movement
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

    def velocity(self, event): #-----------------------------------------velocity
        if event == 'VR_mas':
            self.robot1.change_vel("upscale")

        elif event == 'VR_menos': 
            self.robot1.change_vel("downscale")

    def history(self, event, index_point, combo=""):  #-----------------------------------------history
        if event == 'guardar':
            move_group = self.robot1.move_group
            joint_goal = move_group.get_current_joint_values()
            wpose = move_group.get_current_pose().pose

            self.robot1.Joint_points[index_point] = [joint_goal]
            self.robot1.Cartesian_points[index_point] = [wpose]
            self.robot1.mov_type[index_point] = combo

            with open(self.movements_file, "wb") as f:
                list_data = [self.robot1.Joint_points, self.robot1.Cartesian_points, self.robot1.mov_type]
                #print(list_data[2])
                pickle.dump(list_data, f)

            xvalor= 1 # 1 añadir | 0 eliminar
            yvalor= index_point # Posicion

            if self.robot1.mov_type[index_point] == 'PTP':
                zvalor = 1
            else:
                zvalor = 2

            pointmessaje = Point(xvalor, yvalor, zvalor)
            self.pub.publish(pointmessaje)

        elif event == 'goto':            
            if(self.robot1.mov_type[index_point] == "PTP"):            
                self.robot1.joints_move_sequence(index_point)
            
            elif(self.robot1.mov_type[index_point] == "LIN"):
                #self.robot1.cartesian_sequence(index_point)
                self.robot1.joints_move_sequence(index_point)
                # los dos van a move joints porque cartesian no siempre funciona.
                # los dejo separados por si en alguna actualizacion se arregla y remplazarlo.

            elif(self.robot1.mov_type[index_point] == None):
                print("No hay un punto guardado")
        
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
        if event == 'start':        
            index = 0
            self.robot1.go_to_home()
            self.robot1.go_to_draw()
            for save_point in self.robot1.mov_type:
                
                if(save_point == "PTP"):
                    self.robot1.joints_move_sequence(index)
                
                elif(save_point == "LIN"):
                    self.robot1.cartesian_sequence(index)
                if save_point != None:
                    xvalor= 1 # 1 añadir | 0 eliminar
                    yvalor= index # Posicion 
                    if self.robot1.mov_type[index] == 'PTP':
                        zvalor = 1
                    else:
                        zvalor = 2            
                    pointmessaje = Point(xvalor, yvalor, zvalor)      
                    self.pub.publish(pointmessaje)

                index += 1
            self.robot1.go_to_home()
            print("Secuencia Terminada")
        
        elif event == 'siguiente':
            if(self.robot1.mov_type[index_point] == "PTP"):
                self.robot1.joints_move_sequence(index_point)
            elif(self.robot1.mov_type[index_point] == "LIN"):
                self.robot1.cartesian_sequence(index_point)
            elif(self.robot1.mov_type[index_point] == None):
                print("No hay un punto guardado")
            
        elif event == 'anterior':
            if(self.robot1.mov_type[index_point] == "PTP"):
                self.robot1.joints_move_sequence(index_point)
            
            elif(self.robot1.mov_type[index_point] == "LIN"):
                self.robot1.cartesian_sequence(index_point)

            elif(self.robot1.mov_type[index_point] == None):
                print("No hay un punto guardado")  

    def general(self, event): #-----------------------------------------general

        if event == 'HOME':
            self.robot1.go_to_home()
        
        elif event == "clear_whiteboard":        
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
    
    def offline_data(self, event, filename):
        PIK_sequence = self.total_path + filename + ".dat"

        if event == "import":            
            with open(PIK_sequence, "rb") as f:
                try:
                    rate = rospy.Rate(10)
                    while True:
                        list_data = pickle.load(f)
                        self.robot1.Joint_points = list_data[0]
                        self.robot1.Cartesian_points = list_data[1]
                        self.robot1.mov_type = list_data[2]
                        print("IMPORT SEQUENCE: ", self.robot1.mov_type)
                        for index, pos in enumerate(self.robot1.mov_type):
                            #print(index, pos)
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
                except EOFError:
                    pass

        elif event == "export":
            with open(PIK_sequence, "wb") as f:
                list_data = [self.robot1.Joint_points, self.robot1.Cartesian_points, self.robot1.mov_type]                
                pickle.dump(list_data, f)
                print("EXPORT SEQUENCE: ", list_data[2])

