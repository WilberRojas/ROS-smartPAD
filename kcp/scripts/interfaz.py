#!/usr/bin/env python3

import rospy
import tkinter
from tkinter import ttk
import events as ie

#rospy.init_node('interfaz', anonymous=True) # Al importar interfaz_events se comparte su nodo

class Interfaz1:
    def __init__(self):
        self.events = ie.Events1()

        self.Color_Background= 'gray12' #'dark slate gray'
        self.Font_Color='snow'
        self.ventana = tkinter.Tk()
        self.ventana.resizable(False,False)
        self.ventana.configure(background=self.Color_Background)
        self.ventana.title("ROS smartPAD")

        self.matrix_X = 2
        self.matrix_Y = 8
        self.VelR_percentage = 25

        self.start_row_tools = 30
        self.start_column_tools = 0

        self.start_row_menu = 0
        self.start_column_menu = 0

        self.table_status = False
        self.move_status = True
        
        #se añaden todos los elementos        
        self.movement()        
        self.add_cartesian_movements()
        self.add_joints_movements()
        self.add_historial()
        self.add_secuencia()
        # - tools
        self.tools() #table
        # - Menu
        self.menu()

        #se muestran todos los elementos
        self.grafical_events("show_move")
        self.spawn_historial()
        self.spawn_secuencia()
        # - tools
        self.spawn_tools()
        self.spawn_table()
        # - Menu: Move, History, Sequence, Tools
        # - como inicia activado hay que mostrar el boton de desactivar
        #self.spawn_menu_move()
        self.not_show_move.grid(row=self.start_row_menu,column=self.start_column_menu, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.not_show_history.grid(row=self.start_row_menu,column=self.start_column_menu+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.not_show_sequence.grid(row=self.start_row_menu,column=self.start_column_menu+2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.not_show_tools.grid(row=self.start_row_menu,column=self.start_column_menu+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

    def grafical_events(self, event):  #----------------------grafical_events
        move_type = self.Combo_move.get()
        print("----------NUEVO EVENTO:",event)
        # Move Type        
        self.events.move_type = move_type

        # Velocity
        if event == 'VR_mas':
            if(self.VelR_percentage == 100):
                print("maxima velocidad")
            else:
                self.events.velocity(event)
                self.VelR_percentage = self.VelR_percentage*2
                text_label_point = "Vel.: "+str(int(self.VelR_percentage))+"%"
                self.VelR_Label.config(text=text_label_point)

        elif event == 'VR_menos':
            if(int(self.VelR_percentage) == 1):
                print("minima velocidad")
            else:
                self.events.velocity(event)                    
                self.VelR_percentage = self.VelR_percentage/2
                text_label_point = "Vel.: "+str(int(self.VelR_percentage))+"%"
                self.VelR_Label.config(text=text_label_point)
        
        #Historical
        index_point = int(self.Combo_puntos.get().replace("P",""))-1
        if event == 'guardar':
            combo_modo = self.Combo_modo.get()
            self.events.history(event,index_point, combo_modo)
            text_label_point = "P: "+str(index_point+1)
            self.point_Label.config(text=text_label_point)
            
        elif event == 'siguiente_in':            
            next_index_point = index_point+1
            print("next_index_point: ", next_index_point)
            if next_index_point <= 100:
                text_label_point = "P: "+str(next_index_point+1)
                self.point_Label.config(text=text_label_point)
                self.Combo_puntos.current(next_index_point) 
        
        elif event == 'anterior_in':            
            prev_index_point = index_point-1
            print("prev_index_point: ", prev_index_point)
            if prev_index_point >= 0:
                text_label_point = "P: "+str(prev_index_point+1)
                self.point_Label.config(text=text_label_point)
                self.Combo_puntos.current(prev_index_point) 
        
        elif event == 'goto':
            combo_modo = self.Combo_modo.get()
            self.events.history(event,index_point, combo_modo)
            text_label_point = "P: "+str(index_point+1)
            self.point_Label.config(text=text_label_point)
        
        elif event == 'delete':
            self.events.history(event, index_point)

        else: #para actualizar label con combo box
            text_label_point = "P: "+str(index_point+1)
            self.point_Label.config(text=text_label_point)

        # SECUENCIA   
        if event == 'start':  
            self.events.running(event)

        elif event == 'siguiente':
            next_index_point = index_point+1
            print("next_index_point: ", next_index_point)
            if next_index_point <= 100:
                text_label_point = "P: "+str(next_index_point+1)
                self.point_Label.config(text=text_label_point)
                self.Combo_puntos.current(next_index_point)
                self.events.running(event, next_index_point)
        
        elif event == 'anterior':            
            prev_index_point = index_point-1
            print("prev_index_point: ", prev_index_point)
            if prev_index_point >= 0:
                text_label_point = "P: "+str(prev_index_point+1)
                self.point_Label.config(text=text_label_point)
                self.Combo_puntos.current(prev_index_point)
                self.events.running(event, prev_index_point)

        # Table
        if event == "add_table":
            self.hide_table()
            self.events.general("add_table")
            
        elif event == "del_table":
            self.spawn_table()
            self.events.general("del_table")

        # Tools: move
        if event == "show_move":
            self.hide_menu_move()
            self.spawn_movement()
            self.move_status = True

        elif event == "not_show_move":            
            self.spawn_menu_move()
            self.hide_movement()
            self.hide_cartesian_movements()
            self.hide_joints_movements()
            self.move_status = False
        
        if self.move_status:
            if move_type == "Ejes":                
                self.hide_cartesian_movements()
                self.spawn_joints_movements()
            elif move_type == "Mundo":                
                self.spawn_cartesian_movements()
                self.hide_joints_movements()
                
        # Tools: history
        if event == "show_history":
            self.hide_menu_history()
            self.spawn_historial()
        elif event == "not_show_history":
            self.spawn_menu_history()
            self.hide_historial()
        # Tools: history
        if event == "show_sequence":
            self.hide_menu_sequence()
            self.spawn_secuencia()
        elif event == "not_show_sequence":
            self.spawn_menu_sequence()
            self.hide_secuencia()
        # Tools: history
        if event == "show_tools":
            self.hide_menu_tools()
            self.spawn_tools()
            if self.table_status == True:
                self.spawn_table()
            else: self.hide_table()
            
        elif event == "not_show_tools":
            self.spawn_menu_tools()
            self.hide_tools()

        # Export / Import

        if event == "import":
            text_box = self.import_name.get("1.0","end-1c")
            self.events.offline_data(event, text_box)            

        elif event == "export":
            text_box = self.export_name.get("1.0","end-1c")
            self.events.offline_data(event, text_box) 

    #--------------------------------- END grafical_events
    
    #------------------------------------------------------------------------------------- MENU

    def menu(self):
        self.show_move = tkinter.Button(self.ventana, text= "Move", command=lambda: self.grafical_events("show_move"))
        self.not_show_move = tkinter.Button(self.ventana, text= "Move", command=lambda: self.grafical_events("not_show_move"),
                                            bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")

        self.show_history = tkinter.Button(self.ventana, text= "History", command=lambda: self.grafical_events("show_history"))
        self.not_show_history = tkinter.Button(self.ventana, text= "History", command=lambda: self.grafical_events("not_show_history"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")

        self.show_sequence = tkinter.Button(self.ventana, text= "Sequence", command=lambda: self.grafical_events("show_sequence"))
        self.not_show_sequence = tkinter.Button(self.ventana, text= "Sequence", command=lambda: self.grafical_events("not_show_sequence"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")

        self.show_tools = tkinter.Button(self.ventana, text= "Tools", command=lambda: self.grafical_events("show_tools"))
        self.not_show_tools = tkinter.Button(self.ventana, text= "Tools", command=lambda: self.grafical_events("not_show_tools"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")

    def spawn_menu_move(self): #--------menu: move
        self.show_move.grid(row=self.start_row_menu,column=self.start_column_menu, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.not_show_move.grid_forget()
    
    def hide_menu_move(self):
        self.show_move.grid_forget()
        self.not_show_move.grid(row=self.start_row_menu,column=self.start_column_menu, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
    
    def spawn_menu_history(self): #--------menu: history
        self.show_history.grid(row=self.start_row_menu,column=self.start_column_menu+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.not_show_history.grid_forget()
    
    def hide_menu_history(self):
        self.show_history.grid_forget()
        self.not_show_history.grid(row=self.start_row_menu,column=self.start_column_menu+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

    def spawn_menu_sequence(self): #--------menu: sequence
        self.show_sequence.grid(row=self.start_row_menu,column=self.start_column_menu+2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.not_show_sequence.grid_forget()
    
    def hide_menu_sequence(self):
        self.show_sequence.grid_forget()
        self.not_show_sequence.grid(row=self.start_row_menu,column=self.start_column_menu+2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

    def spawn_menu_tools(self): #--------menu: tools
        self.show_tools.grid(row=self.start_row_menu,column=self.start_column_menu+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.not_show_tools.grid_forget()
    
    def hide_menu_tools(self):
        self.show_tools.grid_forget()
        self.not_show_tools.grid(row=self.start_row_menu,column=self.start_column_menu+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

    #------------------------------------------------------------------------------------- TOOLS

    def tools(self):
        self.tools_Label = tkinter.Label(self.ventana, text = "TOOLS", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.clear_button = tkinter.Button(self.ventana, text= "CLEAR", command=lambda: self.events.general("clear_whiteboard"))        
        self.add_table_button = tkinter.Button(self.ventana, text= "Table", command=lambda: self.grafical_events("add_table"))
        self.del_table_button = tkinter.Button(self.ventana, text= "Table", command=lambda: self.grafical_events("del_table"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")
       
    def spawn_tools(self):  #--------tools: Title, Clear
        self.tools_Label.grid(row=self.start_row_tools,column=self.start_column_tools+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.clear_button.grid(row=self.start_row_tools+1,column=self.start_column_tools, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        
    def hide_tools(self): 
        self.tools_Label.grid_forget()
        self.clear_button.grid_forget()
        self.del_table_button.grid_forget()
        self.add_table_button.grid_forget()
    
    def spawn_table(self): #--------tools: table
        self.add_table_button.grid(row=self.start_row_tools+1,column=self.start_column_tools+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.del_table_button.grid_forget()
        self.table_status = True
    
    def hide_table(self):
        self.add_table_button.grid_forget()
        self.del_table_button.grid(row=self.start_row_tools+1,column=self.start_column_tools+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.table_status = False
    
    #------------------------------------------------------------------------------------- MOVEMENT

    def movement(self):
        self.home_button = tkinter.Button(self.ventana, text= "HOME", command=lambda: self.events.general("HOME")) 
        #------------------------ Mundo | Ejes
        Options4 = ["Ejes","Mundo"]
        self.Combo_move = ttk.Combobox(self.ventana, value=Options4)
        self.Combo_move.current(0)
        self.Combo_move.bind("<<ComboboxSelected>>",self.grafical_events)    
        self.Combo_move.config(width=10)   
        
        #------------------------ + | -        
        self.Button_joint1_mas = tkinter.Button(self.ventana, text= "+", command=lambda: self.events.movement("j1_mas"))
        self.Button_joint1_menos = tkinter.Button(self.ventana, text= "-", command=lambda: self.events.movement("j1_menos"))
        self.Button_joint2_mas = tkinter.Button(self.ventana, text= "+", command=lambda: self.events.movement("j2_mas"))
        self.Button_joint2_menos = tkinter.Button(self.ventana, text= "-", command=lambda: self.events.movement("j2_menos"))
        self.Button_joint3_mas = tkinter.Button(self.ventana, text= "+", command=lambda: self.events.movement("j3_mas"))
        self.Button_joint3_menos = tkinter.Button(self.ventana, text= "-", command=lambda: self.events.movement("j3_menos"))
        self.Button_joint4_mas = tkinter.Button(self.ventana, text= "+", command=lambda: self.events.movement("j4_mas"))
        self.Button_joint4_menos = tkinter.Button(self.ventana, text= "-", command=lambda: self.events.movement("j4_menos"))
        self.Button_joint5_mas = tkinter.Button(self.ventana, text= "+", command=lambda: self.events.movement("j5_mas"))
        self.Button_joint5_menos = tkinter.Button(self.ventana, text= "-", command=lambda: self.events.movement("j5_menos"))
        self.Button_joint6_mas = tkinter.Button(self.ventana, text= "+", command=lambda: self.events.movement("j6_mas"))
        self.Button_joint6_menos = tkinter.Button(self.ventana, text= "-", command=lambda: self.events.movement("j6_menos"))
        
        #vel
        self.VelR_mas_Button = tkinter.Button(self.ventana, text= "+", command=lambda: self.grafical_events("VR_mas"))
        self.VelR_menos_Button = tkinter.Button(self.ventana, text= "-", command=lambda: self.grafical_events("VR_menos"))
        self.VelR_Label = tkinter.Label(self.ventana, text = "Vel.: 25%", width = 8, height = 2, fg= self.Font_Color, bg= self.Color_Background)

    def spawn_movement(self):
        start_row = 0
        start_column = 0
        self.home_button.grid(row=start_row+1,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)        
        self.Combo_move.grid(row=start_row+1,column=start_column+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)        
        #------------------------ + | - 
        start_row = 2
        start_column = 1

        self.Button_joint1_mas.grid(row=start_row,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint1_menos.grid(row=start_row,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        
        self.Button_joint2_mas.grid(row=start_row+1,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint2_menos.grid(row=start_row+1,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_joint3_mas.grid(row=start_row+2,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint3_menos.grid(row=start_row+2,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_joint4_mas.grid(row=start_row+3,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint4_menos.grid(row=start_row+3,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_joint5_mas.grid(row=start_row+4,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint5_menos.grid(row=start_row+4,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_joint6_mas.grid(row=start_row+5,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint6_menos.grid(row=start_row+5,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)   

        self.VelR_mas_Button.grid(row=start_row+6,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.VelR_menos_Button.grid(row=start_row+6,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.VelR_Label.grid(row=start_row+6,column=start_column-1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

    def hide_movement(self):
        self.home_button.grid_forget()
        self.Combo_move.grid_forget()            
        #------------------------ + | - 
        self.Button_joint1_mas.grid_forget()
        self.Button_joint1_menos.grid_forget()
        
        self.Button_joint2_mas.grid_forget()
        self.Button_joint2_menos.grid_forget()

        self.Button_joint3_mas.grid_forget()
        self.Button_joint3_menos.grid_forget()

        self.Button_joint4_mas.grid_forget()
        self.Button_joint4_menos.grid_forget()

        self.Button_joint5_mas.grid_forget()
        self.Button_joint5_menos.grid_forget()

        self.Button_joint6_mas.grid_forget()
        self.Button_joint6_menos.grid_forget()

        self.VelR_menos_Button.grid_forget()
        self.VelR_mas_Button.grid_forget()
        self.VelR_Label.grid_forget() 
    
    #------------------------------------------------------------------------------------- JOINTS / CARTESIAN

    def add_cartesian_movements(self): #----------------------CARTESIAN
        self.Label_x = tkinter.Label(self.ventana, text = "x", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_y = tkinter.Label(self.ventana, text = "y", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_z = tkinter.Label(self.ventana, text = "z", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_Roll = tkinter.Label(self.ventana, text = "R", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_Pitch = tkinter.Label(self.ventana, text = "P", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_yaw = tkinter.Label(self.ventana, text = "Y", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
    
    def spawn_cartesian_movements(self):
        start_row = 2
        start_column = 0
        self.Label_x.grid(row=start_row,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_y.grid(row=start_row+1,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_z.grid(row=start_row+2,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_Roll.grid(row=start_row+3,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_Pitch.grid(row=start_row+4,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_yaw.grid(row=start_row+5,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

    def hide_cartesian_movements(self):
        self.Label_x.grid_forget()
        self.Label_y.grid_forget()
        self.Label_z.grid_forget()
        self.Label_Roll.grid_forget()
        self.Label_Pitch.grid_forget()
        self.Label_yaw.grid_forget()

    def add_joints_movements(self): #----------------------JOINTS
        self.Label_joint1 = tkinter.Label(self.ventana, text = "J1", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_joint2 = tkinter.Label(self.ventana, text = "J2", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_joint3 = tkinter.Label(self.ventana, text = "J3", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_joint4 = tkinter.Label(self.ventana, text = "J4", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_joint5 = tkinter.Label(self.ventana, text = "J5", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.Label_joint6 = tkinter.Label(self.ventana, text = "J6", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)

    def spawn_joints_movements(self):
        start_row = 2
        start_column = 0
        self.Label_joint1.grid(row=start_row,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_joint2.grid(row=start_row+1,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_joint3.grid(row=start_row+2,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_joint4.grid(row=start_row+3,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_joint5.grid(row=start_row+4,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Label_joint6.grid(row=start_row+5,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
    
    def hide_joints_movements(self):
        self.Label_joint1.grid_forget()
        self.Label_joint2.grid_forget()
        self.Label_joint3.grid_forget()
        self.Label_joint4.grid_forget()
        self.Label_joint5.grid_forget()
        self.Label_joint6.grid_forget()
    
    #------------------------------------------------------------------------------------- HISTORIAL

    def add_historial(self): 
        self.historial_Label = tkinter.Label(self.ventana, text = "HISTORY", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)

        Options1 = ["PTP","LIN"]
        self.Combo_modo = ttk.Combobox(self.ventana, value=Options1)
        self.Combo_modo.current(0)
        self.Combo_modo.bind("<<ComboboxSelected>>",self.grafical_events)
        self.Combo_modo.config(width=10)

        points_list = list(range(1, 101))
        Options2 = ["P" + str(Option) for Option in points_list]
        self.Combo_puntos = ttk.Combobox(self.ventana, value=Options2)
        self.Combo_puntos.current(0)
        self.points_counter = 0
        self.Combo_puntos.bind("<<ComboboxSelected>>",self.grafical_events)
        self.Combo_puntos.config(width=10)

        self.Guardar_Button = tkinter.Button(self.ventana, text= "SAVE POINT", command=lambda: self.grafical_events("guardar"))
        self.goto_Button = tkinter.Button(self.ventana, text= "GO TO POINT", command=lambda: self.grafical_events("goto"))
        self.delete_Button = tkinter.Button(self.ventana, text= "DELETE POINT", command=lambda: self.grafical_events("delete"))

        self.anterior_in = tkinter.Button(self.ventana, text= "PREV", command=lambda: self.grafical_events("anterior_in"))
        self.siguiente_in = tkinter.Button(self.ventana, text= "NEXT", command=lambda: self.grafical_events("siguiente_in"))        

    def spawn_historial(self):
        start_row = 10
        start_column = 0

        self.anterior_in.grid(row=start_row + 1, column=start_column + 1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.siguiente_in.grid(row=start_row + 1, column=start_column + 2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.historial_Label.grid(row=start_row,column=start_column+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)        
        self.Combo_puntos.grid(row=start_row+2,column=start_column, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.Combo_modo.grid(row=start_row+2,column=start_column+2, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)        

        self.Guardar_Button.grid(row=start_row+3,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.goto_Button.grid(row=start_row+3,column=start_column+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.delete_Button.grid(row=start_row+3,column=start_column+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)        

    def hide_historial(self):
        self.anterior_in.grid_forget()
        self.siguiente_in.grid_forget()

        self.historial_Label.grid_forget()
        self.Combo_modo.grid_forget()
        self.Combo_puntos.grid_forget()

        self.Guardar_Button.grid_forget()
        self.goto_Button.grid_forget()
        self.delete_Button.grid_forget()        

    #------------------------------------------------------------------------------------- SECUENCIA

    def add_secuencia(self): 
        self.secuencia_Label = tkinter.Label(self.ventana, text = "SEQUENCE", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.point_Label = tkinter.Label(self.ventana, text = "P: 1", width = 5, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.anterior_Button = tkinter.Button(self.ventana, text= "PREV", command=lambda: self.grafical_events("anterior"))
        self.siguiente_Button = tkinter.Button(self.ventana, text= "NEXT", command=lambda: self.grafical_events("siguiente"))
        self.start_Button = tkinter.Button(self.ventana, text= "START", command=lambda: self.grafical_events("start"))

        self.import_name = tkinter.Text(self.ventana, height=1, width=15)      
        self.import_name.insert(tkinter.INSERT, "sequence")

        self.export_name = tkinter.Text(self.ventana, height=1, width=15)
  
        self.import_data = tkinter.Button(self.ventana, text="IMPORT", command=lambda: self.grafical_events("import"))
        self.export_data = tkinter.Button(self.ventana, text="EXPORT", command=lambda: self.grafical_events("export"))

    def spawn_secuencia(self):
        start_row = 20
        start_column = 0
        self.secuencia_Label.grid(row=start_row,column=start_column+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.point_Label.grid(row=start_row+1,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.anterior_Button.grid(row=start_row+1,column=start_column+2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)    
        self.siguiente_Button.grid(row=start_row+1,column=start_column+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)        
        self.start_Button.grid(row=start_row+1,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.import_name.grid(row=start_row+2,column=start_column, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.export_name.grid(row=start_row+2,column=start_column+2, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)

        self.import_data.grid(row=start_row+3,column=start_column, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.export_data.grid(row=start_row+3,column=start_column+2, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)

    def hide_secuencia(self):        
        self.secuencia_Label.grid_forget()
        self.point_Label.grid_forget()
        self.siguiente_Button.grid_forget()
        self.anterior_Button.grid_forget()
        self.start_Button.grid_forget()

        self.import_name.grid_forget()
        self.export_name.grid_forget()

        self.import_data.grid_forget()
        self.export_data.grid_forget()
    
    #------------------------------------------------------------------------------------- RUN

    def run(self):
        try:
            #loop
            self.ventana.mainloop()

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return


if __name__ == "__main__":
    interfaz1 = Interfaz1()
    interfaz1.run() 