#!/usr/bin/env python3

from math import pi, tau, dist, fabs, cos
import tkinter
from tkinter import ttk
from geometry_msgs.msg import Point
import rospy
import moveit_kr6
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

#rospy.init_node('interfaz', anonymous=True) # Al importar moveit_kr6 se comparte su nodo
pub = rospy.Publisher('position_list', Point, queue_size=1)
pub_cleaner = rospy.Publisher('custom_joint_states', JointState, queue_size=1)

robot1 = moveit_kr6.MoveGroupPythonInterface()

Color_Background= 'gray12' #'dark slate gray'
Font_Color='snow'
ventana = tkinter.Tk()
ventana.configure(background=Color_Background)
ventana.title("ROS smartPAD")

Options1 = ["PTP","LIN"]
Combo_modo = ttk.Combobox(ventana, value=Options1)
Combo_modo.current(0)

points_list = list(range(1, 101))
Options2 = ["P" + str(Option) for Option in points_list]
Combo_puntos = ttk.Combobox(ventana, value=Options2)
Combo_puntos.current(0)
points_counter = 0

point_Label = tkinter.Label(ventana, text = "P: 1", width = 30, height = 2, fg= Font_Color, bg= Color_Background)
VelR_Label = tkinter.Label(ventana, text = "Vel.: 25", width = 10, height = 2, fg= Font_Color, bg= Color_Background)
VelR_percentage = 25

def windows_events(event): #------------------------------------------------------------------------------------windows_events
    global points_counter, point_Label, VelR_Label, VelR_percentage
    
    print("------------NUEVO EVENTO:", event)
    index_point = int(Combo_puntos.get().replace("P",""))-1    
    #--------------Cartesian event-------------
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

    #--------------Joints event-------------
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
    
    #---------------VELOCIDADES-------------
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

    # ---------------- Historial -------------

    elif event == 'guardar':               

        move_group = robot1.move_group
        joint_goal = move_group.get_current_joint_values()
        wpose = move_group.get_current_pose().pose       
        
        robot1.Joint_points[index_point] = [joint_goal]
        robot1.Cartesian_points[index_point] = [wpose]
        robot1.mov_type[index_point] = Combo_modo.get() 

        text_label_point = "P: "+str(index_point+1)
        point_Label.config(text=text_label_point)       

        xvalor= 1 # 1 añadir | 0 eliminar
        yvalor= index_point # Posicion 

        if robot1.mov_type[index_point] == 'PTP':
            zvalor = 1
        else:
            zvalor = 2
       
        pointmessaje = Point(xvalor, yvalor, zvalor)      
        pub.publish(pointmessaje)

        
    elif event == 'siguiente':
        print("Punto: ", points_counter + 1)
        points_counter +=1

        if(robot1.mov_type[points_counter] == "PTP"):
            robot1.joints_move_sequence(points_counter)
        
        elif(robot1.mov_type[points_counter] == "LIN"):
            robot1.cartesian_sequence(points_counter)

        elif(robot1.mov_type[points_counter] == None):
            print("No hay un punto guardado")

        text_label_point = "P: "+str(points_counter+1)
        point_Label.config(text=text_label_point)
        
        if points_counter > 100:
            print("No hay mas puntos")
            points_counter -=1
        
        Combo_puntos.current(points_counter)
        
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

        elif(robot1.mov_type[points_counter] == None):
            print("No hay un punto guardado")

        text_label_point = "P: "+str(points_counter+1)
        point_Label.config(text=text_label_point)

        Combo_puntos.current(points_counter)

    elif event == 'start':        
        index = 0
        for save_point in robot1.mov_type:
            
            if(save_point == "PTP"):
                robot1.joints_move_sequence(index)
            
            elif(save_point == "LIN"):
                robot1.cartesian_sequence(index)
            if save_point != None:
                text_label_point = "P: "+str(index+1)
                point_Label.config(text=text_label_point)
                Combo_puntos.current(index)
                points_counter = index

                xvalor= 1 # 1 añadir | 0 eliminar
                yvalor= index # Posicion 
                if robot1.mov_type[index] == 'PTP':
                    zvalor = 1
                else:
                    zvalor = 2            
                pointmessaje = Point(xvalor, yvalor, zvalor)      
                pub.publish(pointmessaje)

            index += 1
            
        print("Secuencia Terminada")

    elif event == 'goto':        
        #index_point = int(Combo_puntos.get().replace("P",""))-1
        if(robot1.mov_type[index_point] == "PTP"):            
            robot1.joints_move_sequence(index_point)
        
        elif(robot1.mov_type[index_point] == "LIN"):            
            robot1.cartesian_sequence(index_point)

        elif(robot1.mov_type[index_point] == None):
            print("No hay un punto guardado")

        points_counter = index_point        
        text_label_point = "P: "+str(index_point+1)        
        point_Label.config(text=text_label_point)
    
    elif event == 'delete':        
        #index_point = int(Combo_puntos.get().replace("P",""))-1

        robot1.Joint_points[index_point] = None
        robot1.Cartesian_points[index_point] = None
        robot1.mov_type[index_point] = None

        xvalor= 0 # 1 añadir | 0 eliminar
        yvalor= index_point # Posicion
        zvalor = 0
       
        pointmessaje = Point(xvalor, yvalor, zvalor)      
        pub.publish(pointmessaje)
    
    elif event == "clear_whiteboard":        
        hello_str = JointState()
        hello_str.header = Header()
        hello_str.header.stamp = rospy.Time.now()
        hello_str.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        hello_str.position = [11, 0, 0, 0, 0]
        hello_str.velocity = []
        hello_str.effort = []
        pub_cleaner.publish(hello_str)

    elif event == 'add_table':
        robot1.add_table_fun()        

    elif event == 'del_table':
        robot1.remove_table_fun()
        
    else:
        text_label_point = "P: "+str(index_point+1)
        point_Label.config(text=text_label_point)
        points_counter = index_point

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
        export_Button.grid(row=12,column=2, columnspan=2, padx=padx_value, pady=pady_value)"""

        import_Button = tkinter.Button(ventana, text= "CLEAR", command=lambda: windows_events("clear_whiteboard"))
        import_Button.grid(row=0,column=1, columnspan=2, padx=padx_value, pady=pady_value) 

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