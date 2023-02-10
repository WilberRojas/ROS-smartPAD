#!/usr/bin/env python3

import rospy
import tkinter
from tkinter import ttk
import interface_events as ie


#rospy.init_node('interfaz', anonymous=True) # Al importar interfaz_events se comparte su nodo
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

class Interfaz1:
    def __init__(self):
        self.debug_prints = True
        self.contador_comando = 1

        self.group_name = rospy.get_param('/ros_smartpad/Planning_Group') #-------------GROUP NAME
        #self.group_name = "kuka6"
        self.events = ie.Events1(self.group_name)

        self.Color_Background= 'gray12' #'dark slate gray'
        self.Font_Color='snow'
        self.ventana = tkinter.Tk()
        self.ventana.resizable(False,False)
        self.ventana.configure(background=self.Color_Background)
        self.ventana.title("ROS smartPAD")

        self.matrix_X = 2
        self.matrix_Y = 8
        self.VelR_percentage = 25 #Default
        self.step_value = 0.25 #Default

        self.start_row_tools = 30
        self.start_column_tools = 0

        self.start_row_menu = 0
        self.start_column_menu = 0

        self.table_status = False
        self.move_status = True
        self.set_position_button = True
        self.tool_status = False
        self.on_history = False

        self.circ_points = [None, None, None]
        self.joints_circ1 = []
        
        #se añaden todos los elementos        
        self.movement()        
        self.add_cartesian_movements()
        self.add_joints_movements()
        self.add_historial()
        self.add_secuencia()       
        self.tools() #table, tool     
        self.menu()

        #se muestra el menu de show_move
        self.grafical_events("show_move")        
        self.not_show_move.grid(row=self.start_row_menu,column=self.start_column_menu, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        # Se guarda la primera posicion del robot como HOME:
        self.grafical_events("HOME")

        # Para saber que ya esta listo:
        print(bcolors.OKGREEN+"Ready for your commands" + bcolors.ENDC)       

    def grafical_events(self, event):  #----------------------grafical_events

        if self.debug_prints: 
            print(bcolors.HEADER+"Comando "+str(self.contador_comando)+":"+bcolors.ENDC, event)
            self.contador_comando += 1

        move_type = self.Combo_move.get()        
        # Move Type        
        self.events.move_type = move_type

        # Velocity
        if event == 'VR_mas':
            if(self.VelR_percentage == 100):
                print(bcolors.WARNING+"maxima velocidad"+ bcolors.ENDC)
            else:
                self.VelR_percentage = self.VelR_percentage*2
                self.events.velocity(event, self.VelR_percentage)
                
                if self.VelR_percentage < 1:
                    text_label_point = "Vel.: "+str(round(self.VelR_percentage,2))+"%"
                else:
                    text_label_point = "Vel.: "+str(int(self.VelR_percentage))+"%"
                self.VelR_Label.config(text=text_label_point)


        elif event == 'VR_menos':
            if(self.VelR_percentage < 0.1):
                print(bcolors.WARNING+"minima velocidad"+ bcolors.ENDC)
            else:
                self.VelR_percentage = self.VelR_percentage/2
                self.events.velocity(event, self.VelR_percentage)
                
                if self.VelR_percentage < 1:
                    text_label_point = "Vel.: "+str(round(self.VelR_percentage,2))+"%"
                else:
                    text_label_point = "Vel.: "+str(int(self.VelR_percentage))+"%"
                self.VelR_Label.config(text=text_label_point)

        # --------------STEP SIZE
        if event == 'step_mas':
            
            self.step_value = self.step_value*2
            self.events.step_size(event)
            
            if self.step_value*100 < 1:
                text_label_point = "Step: "+str(round(self.step_value*100,2))+"cm\n      " + str(round(90*self.step_value,2)) +"º"
            else:
                text_label_point = "Step: "+str(round(self.step_value*100,1))+"cm\n      " + str(round(90*self.step_value,1)) +"º"
            self.step_Label.config(text=text_label_point)

        elif event == 'step_menos':
            
            self.step_value = self.step_value/2
            self.events.step_size(event)
            
            if self.step_value*100 < 1:
                text_label_point = "Step: "+str(round(self.step_value*100,2))+"cm\n      " + str(round(90*self.step_value,2)) +"º"
            else:
                text_label_point = "Step: "+str(round(self.step_value*100,1))+"cm\n      " + str(round(90*self.step_value,1)) +"º"
            self.step_Label.config(text=text_label_point)
        
        #Historical
        index_point = int(self.Combo_puntos.get().replace("P",""))-1        
        combo_modo = self.Combo_modo.get()
        if self.on_history:
            if combo_modo == "CIRC" or combo_modo == "SHAPE":
                self.Combo_plano.grid(row=13,column=0, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
                self.h_p1.grid(row=13,column=1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
                self.h_p2.grid(row=13,column=2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
                if combo_modo == "CIRC": self.h_p3.grid(row=13,column=3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
                else: self.h_p3.grid_forget()
                if combo_modo == "SHAPE": self.Combo_esquinas.grid(row=13,column=3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
                else: self.Combo_esquinas.grid_forget()
            else:
                self.Combo_plano.grid_forget()
                self.h_p1.grid_forget()
                self.h_p2.grid_forget()
                self.h_p3.grid_forget()
                self.Combo_esquinas.grid_forget()

            if event == "p1" or event == "p2" or event == "p3": 
                joint_list, cartesian_position, cartesian_orientation = self.events.general("get_pos_mute")
                if event == "p1": 
                    self.circ_points = [None, None, None]
                    self.circ_points[0] = cartesian_position
                    self.joints_circ1 = joint_list
                if event == "p2": 
                    self.circ_points[1] = cartesian_position
                if event == "p3": 
                    self.circ_points[2] = cartesian_position
                
                print(self.circ_points)
            


        if event == 'guardar':
            index_ComboPlano = self.Combo_plano.get()
            if combo_modo == "CIRC" or combo_modo == "SHAPE":
                #positions=[],joints_circ1 = None)
                self.events.history(event,index_point, combo_modo, index_ComboPlano, self.circ_points, self.joints_circ1)
            else:
                self.events.history(event,index_point, combo_modo, index_ComboPlano)
            
            
        elif event == 'siguiente_in':
            next_index_point = index_point+1
            if next_index_point <= 100:
                text_label_point = "P: "+str(next_index_point+1)
                self.point_Label.config(text=text_label_point)
                self.Combo_puntos.current(next_index_point) 
        
        elif event == 'anterior_in':            
            prev_index_point = index_point-1
            if prev_index_point >= 0:
                text_label_point = "P: "+str(prev_index_point+1)
                self.point_Label.config(text=text_label_point)
                self.Combo_puntos.current(prev_index_point) 
        
        elif event == 'goto':            
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
            if next_index_point <= 100:                
                text_label_point = "P: "+str(next_index_point+1)
                print(bcolors.OKBLUE+ text_label_point+ bcolors.ENDC)
                self.point_Label.config(text=text_label_point)
                self.Combo_puntos.current(next_index_point)
                self.events.running(event, next_index_point)
        
        elif event == 'anterior':            
            prev_index_point = index_point-1
            if prev_index_point >= 0:
                text_label_point = "P: "+str(prev_index_point+1)
                print(bcolors.OKBLUE+ text_label_point+ bcolors.ENDC)
                self.point_Label.config(text=text_label_point)
                self.Combo_puntos.current(prev_index_point)
                self.events.running(event, prev_index_point)

        # Table
        if event == "add_table":
            self.table_status = True
            self.hide_table()
            self.events.general("add_table")
            
            
        elif event == "del_table":
            self.table_status = False
            self.spawn_table()
            self.events.general("del_table")

        # Table
        if event == "add_tool":
            self.tool_status = True
            self.hide_advanced_tool()
            self.events.general("add_tool")
            
        elif event == "del_tool":
            self.tool_status = False
            self.spawn_advanced_tool()
            self.events.general("del_tool")
        
        
        # ----------TOOLS
        # Tools: move
        if event == "show_move":
            self.on_history = False
            #self.menu_function_not_show_move()
            self.menu_function_not_show_sequence()
            self.menu_function_not_show_history()
            self.menu_function_not_show_tools() 

            self.hide_menu_move()
            self.spawn_movement()
            self.move_status = True

            if self.set_position_button == True:
                self.movement_set_position()
            else:  self.movement_move_position()

        #elif event == "not_show_move": self.menu_function_not_show_move()
        
        if self.move_status:
            if move_type == "Ejes":                
                self.hide_cartesian_movements()
                self.spawn_joints_movements()
            elif move_type == "Mundo":                
                self.spawn_cartesian_movements()
                self.hide_joints_movements()
        
        if event == "set_position_on":
            self.movement_set_position()
            self.events.general(event)
        elif event == "move_position_on":
            self.movement_move_position()
            self.events.general(event)
                
        # Tools: history
        if event == "show_history":
            self.on_history = True

            self.menu_function_not_show_move()
            self.menu_function_not_show_sequence()
            #self.menu_function_not_show_history()
            self.menu_function_not_show_tools()            

            self.hide_menu_history()
            self.spawn_historial()
        #elif event == "not_show_history": self.menu_function_not_show_history()
        # Tools: sequence
        if event == "show_sequence":
            self.on_history = False
            self.menu_function_not_show_move()
            #self.menu_function_not_show_sequence()
            self.menu_function_not_show_history()
            self.menu_function_not_show_tools() 

            self.hide_menu_sequence()
            self.spawn_secuencia()
        #elif event == "not_show_sequence": self.menu_function_not_show_sequence()
        # Tools: advanced
        if event == "show_tools":
            self.on_history = False
            self.menu_function_not_show_move()
            self.menu_function_not_show_sequence()
            self.menu_function_not_show_history()
            #self.menu_function_not_show_tools()
            self.hide_menu_tools()
            self.spawn_tools()
            

            if self.table_status == False:
                self.spawn_table()
            else: self.hide_table()

            if self.tool_status == False:
                self.spawn_advanced_tool()
            else: self.hide_advanced_tool()        
        
        #elif event == "not_show_tools": self.menu_function_not_show_tools()

        # Export / Import

        if event == "import":
            text_box = self.import_name.get("1.0","end-1c")
            self.events.offline_data(event, text_box)
            self.update_table_function()
            self.update_tool_function()

        elif event == "export":
            text_box = self.export_name.get("1.0","end-1c")
            self.events.offline_data(event, text_box)

        # Move Table
        if event == "update_table":
            position_table=[]
            dimension_table=[]
            try:
                posX = float(self.Button_Table_posX.get("1.0","end-1c"))
                posY = float(self.Button_Table_posY.get("1.0","end-1c"))
                posZ = float(self.Button_Table_posZ.get("1.0","end-1c"))

                dimX = float(self.Button_Table_dimXmas.get("1.0","end-1c"))
                dimY = float(self.Button_Table_dimYmas.get("1.0","end-1c"))
                dimZ = float(self.Button_Table_dimZmas.get("1.0","end-1c"))
                position_table = [posX, posY, posZ]
                dimension_table = [dimX, dimY, dimZ]
                self.events.advanced_functions(event, [position_table, dimension_table])
            except: print(bcolors.WARNING+"DATOS ERRONEOS: La posicion y dimension de la MESA deben ser numericos"+ bcolors.ENDC)
        
        if event == "update_tool":            
            dimension_tool=[]
            try:
                dimX = float(self.Button_Tool_dimXmas.get("1.0","end-1c"))
                dimY = float(self.Button_Tool_dimYmas.get("1.0","end-1c"))
                dimZ = float(self.Button_Tool_dimZmas.get("1.0","end-1c"))

                posX = float(self.TextBlock_Tool_posX.get("1.0","end-1c"))
                posY = float(self.TextBlock_Tool_posY.get("1.0","end-1c"))
                posZ = float(self.TextBlock_Tool_posZ.get("1.0","end-1c"))
                
                position_tool = [posX, posY, posZ]
                dimension_tool = [dimX, dimY, dimZ]
                self.events.advanced_functions(event, [position_tool ,dimension_tool])
            except: 
                print(bcolors.WARNING+"DATOS ERRONEOS: Las dimensiones de la HERRAMIENTA deben ser numericos"+ bcolors.ENDC)           
        
        if event == "HOME":
            self.events.general(event)
            self.grafical_events("move_position_on")

        elif event == "pos1":
            self.events.general(event)
            self.grafical_events("move_position_on")

        elif event == "pos2":
            self.events.general(event)
            self.grafical_events("move_position_on")

        elif event == "pos3":
            self.events.general(event)
            self.grafical_events("move_position_on")

        elif event == "pos4":
            self.events.general(event)
            self.grafical_events("move_position_on")

        elif event == "pos5":
            self.events.general(event)
            self.grafical_events("move_position_on")

        

    #--------------------------------- END grafical_events

    
    def menu_function_not_show_move(self):
        self.spawn_menu_move()
        self.hide_movement()
        self.hide_cartesian_movements()
        self.hide_joints_movements()
        self.move_status = False
    
    def menu_function_not_show_history(self):
        self.spawn_menu_history()
        self.hide_historial()
    
    def menu_function_not_show_sequence(self):
        self.spawn_menu_sequence()
        self.hide_secuencia()
    
    def menu_function_not_show_tools(self):
        self.spawn_menu_tools()
        self.hide_tools()
        
    
    #------------------------------------------------------------------------------------- MENU

    def menu(self):
        self.show_move = tkinter.Button(self.ventana, text= "Move", command=lambda: self.grafical_events("show_move"))
        self.not_show_move = tkinter.Button(self.ventana, text= "Move", command=lambda: self.grafical_events("not_show_move"),
                                            bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")

        self.show_history = tkinter.Button(self.ventana, text= "History", command=lambda: self.grafical_events("show_history"))
        self.not_show_history = tkinter.Button(self.ventana, text= "History", command=lambda: self.grafical_events("not_show_history"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")

        self.show_sequence = tkinter.Button(self.ventana, text= "Sequence", command=lambda: self.grafical_events("show_sequence"))
        self.not_show_sequence = tkinter.Button(self.ventana, text= "Sequence", command=lambda: self.grafical_events("not_show_sequence"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")

        self.show_tools = tkinter.Button(self.ventana, text= "Advanced", command=lambda: self.grafical_events("show_tools"))
        self.not_show_tools = tkinter.Button(self.ventana, text= "Advanced", command=lambda: self.grafical_events("not_show_tools"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")

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
        self.show_tools.grid(row=self.start_row_menu,column=self.start_column_menu+3, columnspan=3, padx=self.matrix_X, pady=self.matrix_Y)
        self.not_show_tools.grid_forget()
    
    def hide_menu_tools(self):
        self.show_tools.grid_forget()
        self.not_show_tools.grid(row=self.start_row_menu,column=self.start_column_menu+3, columnspan=3, padx=self.matrix_X, pady=self.matrix_Y)

    #------------------------------------------------------------------------------------- Advanced_a

    def update_table_function(self):
        self.Button_Table_posX.delete('1.0',tkinter.END)
        self.Button_Table_posY.delete('1.0',tkinter.END)
        self.Button_Table_posZ.delete('1.0',tkinter.END)
        self.Button_Table_dimXmas.delete('1.0',tkinter.END)
        self.Button_Table_dimYmas.delete('1.0',tkinter.END)
        self.Button_Table_dimZmas.delete('1.0',tkinter.END)

        table_dim = self.events.advanced_functions("get_table_dim")
        
        self.Button_Table_posX.insert(tkinter.INSERT, table_dim[0][0])
        self.Button_Table_posY.insert(tkinter.INSERT, table_dim[0][1])
        self.Button_Table_posZ.insert(tkinter.INSERT, table_dim[0][2])
        self.Button_Table_dimXmas.insert(tkinter.INSERT, table_dim[1][0])
        self.Button_Table_dimYmas.insert(tkinter.INSERT, table_dim[1][1])
        self.Button_Table_dimZmas.insert(tkinter.INSERT, table_dim[1][2])

    def update_tool_function(self):
        self.Button_Tool_dimXmas.delete('1.0',tkinter.END)
        self.Button_Tool_dimYmas.delete('1.0',tkinter.END)
        self.Button_Tool_dimZmas.delete('1.0',tkinter.END)

        self.TextBlock_Tool_posX.delete('1.0',tkinter.END)
        self.TextBlock_Tool_posY.delete('1.0',tkinter.END)
        self.TextBlock_Tool_posZ.delete('1.0',tkinter.END)

        tool_pos, tool_dim = self.events.advanced_functions("get_tool_dim")        

        self.Button_Tool_dimXmas.insert(tkinter.INSERT, tool_dim[0])
        self.Button_Tool_dimYmas.insert(tkinter.INSERT, tool_dim[1])
        self.Button_Tool_dimZmas.insert(tkinter.INSERT, tool_dim[2])

        self.TextBlock_Tool_posX.insert(tkinter.INSERT, tool_pos[0])
        self.TextBlock_Tool_posY.insert(tkinter.INSERT, tool_pos[1])
        self.TextBlock_Tool_posZ.insert(tkinter.INSERT, tool_pos[2])

    def tools(self):
        #self.advanced_Label = tkinter.Label(self.ventana, text = "ADVANCED", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.button_get_pos = tkinter.Button(self.ventana, text= "Get Pos", command=lambda: self.events.general("get_pos"))
        #-----------------------Table
        self.clear_button = tkinter.Button(self.ventana, text= "CLEAR", command=lambda: self.events.general("clear_whiteboard"))
        self.table_update = tkinter.Button(self.ventana, text= "UPDATE: Table", command=lambda: self.grafical_events("update_table"))
        self.add_table_button = tkinter.Button(self.ventana, text= "Table", command=lambda: self.grafical_events("add_table"))
        self.del_table_button = tkinter.Button(self.ventana, text= "Table", command=lambda: self.grafical_events("del_table"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")
        # Position
        self.Label_Table_Positions = tkinter.Label(self.ventana, text = "Pos (x,y,z):", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)

        self.Button_Table_posX = tkinter.Text(self.ventana, height=1, width=8)        
        self.Button_Table_posY = tkinter.Text(self.ventana, height=1, width=8)        
        self.Button_Table_posZ = tkinter.Text(self.ventana, height=1, width=8)
        
        # Dimensions

        self.Label_Table_Dimensions = tkinter.Label(self.ventana, text = "Dim (x,y,z):", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)

        self.Button_Table_dimXmas = tkinter.Text(self.ventana, height=1, width=8)
        self.Button_Table_dimYmas = tkinter.Text(self.ventana, height=1, width=8)
        self.Button_Table_dimZmas = tkinter.Text(self.ventana, height=1, width=8)

        #---------------Tool
        self.add_tool_button = tkinter.Button(self.ventana, text= "Tool", command=lambda: self.grafical_events("add_tool"))
        self.del_tool_button = tkinter.Button(self.ventana, text= "Tool", command=lambda: self.grafical_events("del_tool"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")
        self.tool_update = tkinter.Button(self.ventana, text= "UPDATE: Tool", command=lambda: self.grafical_events("update_tool"))

        # Dimensions tool

        self.Label_Tool_Dimensions = tkinter.Label(self.ventana, text = "Dim (x,y,z):", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)

        self.Button_Tool_dimXmas = tkinter.Text(self.ventana, height=1, width=8)
        self.Button_Tool_dimYmas = tkinter.Text(self.ventana, height=1, width=8)
        self.Button_Tool_dimZmas = tkinter.Text(self.ventana, height=1, width=8)

        # Position tool

        self.Label_Tool_Positions = tkinter.Label(self.ventana, text = "Pos (x,y,z):", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)

        self.TextBlock_Tool_posX = tkinter.Text(self.ventana, height=1, width=8)
        self.TextBlock_Tool_posY = tkinter.Text(self.ventana, height=1, width=8)
        self.TextBlock_Tool_posZ = tkinter.Text(self.ventana, height=1, width=8)

        self.update_table_function()
        self.update_tool_function()

    def spawn_tools(self):  #--------tools: Title, Clear, get_pos.
        #self.advanced_Label.grid(row=self.start_row_tools,column=self.start_column_tools+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)        
        self.button_get_pos.grid(row=self.start_row_tools+1,column=self.start_column_tools+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        
    def hide_tools(self):
        self.spawn_table()
        self.spawn_advanced_tool()
        #self.advanced_Label.grid_forget()
        self.clear_button.grid_forget()
        self.button_get_pos.grid_forget()
        self.del_table_button.grid_forget()
        self.add_table_button.grid_forget()
        self.del_tool_button.grid_forget()
        self.add_tool_button.grid_forget()
    
    def spawn_advanced_tool(self): #--------tools: table
        self.add_tool_button.grid(row=self.start_row_tools+1,column=self.start_column_tools+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)        
        self.del_tool_button.grid_forget()
        self.tool_update.grid_forget()

        self.Label_Tool_Dimensions.grid_forget()

        self.Button_Tool_dimXmas.grid_forget()
        self.Button_Tool_dimYmas.grid_forget()
        self.Button_Tool_dimZmas.grid_forget()

        self.Label_Tool_Positions.grid_forget()

        self.TextBlock_Tool_posX.grid_forget()
        self.TextBlock_Tool_posY.grid_forget()
        self.TextBlock_Tool_posZ.grid_forget()
        

    def hide_advanced_tool(self): 
        self.add_tool_button.grid_forget()
        self.del_tool_button.grid(row=self.start_row_tools+1,column=self.start_column_tools+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)        
        
        self.Label_Tool_Dimensions.grid(row=self.start_row_tools+6,column=self.start_column_tools+0, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_Tool_dimXmas.grid(row=self.start_row_tools+6,column=self.start_column_tools+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_Tool_dimYmas.grid(row=self.start_row_tools+6,column=self.start_column_tools+2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_Tool_dimZmas.grid(row=self.start_row_tools+6,column=self.start_column_tools+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Label_Tool_Positions.grid(row=self.start_row_tools+7,column=self.start_column_tools+0, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.TextBlock_Tool_posX.grid(row=self.start_row_tools+7,column=self.start_column_tools+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.TextBlock_Tool_posY.grid(row=self.start_row_tools+7,column=self.start_column_tools+2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.TextBlock_Tool_posZ.grid(row=self.start_row_tools+7,column=self.start_column_tools+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.tool_update.grid(row=self.start_row_tools+8,column=self.start_column_tools+2, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)

    
    def spawn_table(self): #--------tools: table
        self.add_table_button.grid(row=self.start_row_tools+1,column=self.start_column_tools, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)        
        self.del_table_button.grid_forget()
        self.table_update.grid_forget()
        
        self.clear_button.grid_forget()
        #---------------------------------
        # Position
        self.Label_Table_Positions.grid_forget()

        self.Button_Table_posX.grid_forget()
        self.Button_Table_posY.grid_forget()
        self.Button_Table_posZ.grid_forget()

        # Dimensions

        self.Label_Table_Dimensions.grid_forget()

        self.Button_Table_dimXmas.grid_forget()
        self.Button_Table_dimYmas.grid_forget()
        self.Button_Table_dimZmas.grid_forget()

    def hide_table(self):
        self.add_table_button.grid_forget()
        self.del_table_button.grid(row=self.start_row_tools+1,column=self.start_column_tools, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        
        self.clear_button.grid(row=self.start_row_tools+5,column=self.start_column_tools, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.table_update.grid(row=self.start_row_tools+5,column=self.start_column_tools+2, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)

        #---------------------------------
        # Position
        self.Label_Table_Positions.grid(row=self.start_row_tools+3,column=self.start_column_tools, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_Table_posX.grid(row=self.start_row_tools+3,column=self.start_column_tools+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_Table_posY.grid(row=self.start_row_tools+3,column=self.start_column_tools+2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_Table_posZ.grid(row=self.start_row_tools+3,column=self.start_column_tools+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        # Dimensions

        self.Label_Table_Dimensions.grid(row=self.start_row_tools+4,column=self.start_column_tools, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_Table_dimXmas.grid(row=self.start_row_tools+4,column=self.start_column_tools+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_Table_dimYmas.grid(row=self.start_row_tools+4,column=self.start_column_tools+2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_Table_dimZmas.grid(row=self.start_row_tools+4,column=self.start_column_tools+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
    
    #------------------------------------------------------------------------------------- MOVEMENT

    def movement(self):
        self.set_position_on = tkinter.Button(self.ventana, text= "SET", command=lambda: self.grafical_events("set_position_off"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")
        self.set_position_off = tkinter.Button(self.ventana, text= "SET", command=lambda: self.grafical_events("set_position_on"))
        self.set_position_on.config(width=2) 
        self.set_position_off.config(width=2) 

        self.move_position_on = tkinter.Button(self.ventana, text= "MOVE", command=lambda: self.grafical_events("move_position_off"), bg = "green", fg = "white", activebackground="lime green", activeforeground = "white")
        self.move_position_off = tkinter.Button(self.ventana, text= "MOVE", command=lambda: self.grafical_events("move_position_on"))
        self.move_position_on.config(width=2)
        self.move_position_off.config(width=2)

        self.home_button = tkinter.Button(self.ventana, text= "HOME", command=lambda: self.grafical_events("HOME"))
        self.button_pos1 = tkinter.Button(self.ventana, text= "POS 1", command=lambda: self.grafical_events("pos1")) 
        self.button_pos2 = tkinter.Button(self.ventana, text= "POS 2", command=lambda: self.grafical_events("pos2")) 
        self.button_pos3 = tkinter.Button(self.ventana, text= "POS 3", command=lambda: self.grafical_events("pos3")) 
        self.button_pos4 = tkinter.Button(self.ventana, text= "POS 4", command=lambda: self.grafical_events("pos4")) 
        self.button_pos5 = tkinter.Button(self.ventana, text= "POS 5", command=lambda: self.grafical_events("pos5")) 
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


        self.step_Label = tkinter.Label(self.ventana, text = "Step: 0.25m \n      22.5º", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)

        self.step_mas_Button = tkinter.Button(self.ventana, text= "+", command=lambda: self.grafical_events("step_mas"))
        self.step_menos_Button = tkinter.Button(self.ventana, text= "-", command=lambda: self.grafical_events("step_menos"))

    
    def movement_set_position(self): #--------tools: table
        start_row = 0
        start_column = 0

        self.set_position_on.grid(row=start_row+1,column=start_column+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.set_position_off.grid_forget()

        self.move_position_off.grid(row=start_row+1,column=start_column+4, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.move_position_on.grid_forget()
        self.set_position_button = True
    
    def movement_move_position(self):
        start_row = 0
        start_column = 0

        self.set_position_off.grid(row=start_row+1,column=start_column+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.set_position_on.grid_forget()

        self.move_position_on.grid(row=start_row+1,column=start_column+4, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.move_position_off.grid_forget()
        self.set_position_button = False

    def spawn_movement(self):
        start_row = 0
        start_column = 0
        self.Combo_move.grid(row=start_row+1,column=start_column+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.home_button.grid(row=start_row+2,column=start_column+3, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.button_pos1.grid(row=start_row+3,column=start_column+3, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.button_pos2.grid(row=start_row+4,column=start_column+3, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.button_pos3.grid(row=start_row+5,column=start_column+3, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.button_pos4.grid(row=start_row+6,column=start_column+3, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.button_pos5.grid(row=start_row+7,column=start_column+3, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        #------------------------ + | - 
        start_row = 2
        start_column = 1

        self.Button_joint1_mas.grid(row=start_row,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint1_menos.grid(row=start_row,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        
        self.Button_joint2_mas.grid(row=start_row+1,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint2_menos.grid(row=start_row+1,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_joint3_mas.grid(row=start_row+2,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint3_menos.grid(row=start_row+2,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_joint4_mas.grid(row=start_row+3,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint4_menos.grid(row=start_row+3,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_joint5_mas.grid(row=start_row+4,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint5_menos.grid(row=start_row+4,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.Button_joint6_mas.grid(row=start_row+5,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.Button_joint6_menos.grid(row=start_row+5,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)   

        self.VelR_mas_Button.grid(row=start_row+6,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.VelR_menos_Button.grid(row=start_row+6,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.VelR_Label.grid(row=start_row+6,column=start_column-1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.step_Label.grid(row=start_row+7,column=start_column-1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.step_mas_Button.grid(row=start_row+7,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.step_menos_Button.grid(row=start_row+7,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

    def hide_movement(self):        
        self.Combo_move.grid_forget()

        self.set_position_off.grid_forget()
        self.set_position_on.grid_forget()

        self.move_position_on.grid_forget()
        self.move_position_off.grid_forget()

        self.home_button.grid_forget()
        self.button_pos1.grid_forget()
        self.button_pos2.grid_forget()
        self.button_pos3.grid_forget()
        self.button_pos4.grid_forget()
        self.button_pos5.grid_forget()
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

        self.step_Label.grid_forget() 
        self.step_mas_Button.grid_forget() 
        self.step_menos_Button.grid_forget() 
    
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
        #self.historial_Label = tkinter.Label(self.ventana, text = "HISTORY", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)

        Options1 = ["PTP","LIN","CIRC", "SHAPE"]
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

        Options3 = ["XY","XZ","YZ"]
        self.Combo_plano = ttk.Combobox(self.ventana, value=Options3)
        self.Combo_plano.current(0)
        self.Combo_plano.bind("<<ComboboxSelected>>",self.grafical_events)
        self.Combo_plano.config(width=5)

        points_list2 = list(range(3, 21))
        Options4 = [str(Option) + " Puntos" for Option in points_list2]
        self.Combo_esquinas = ttk.Combobox(self.ventana, value=Options4)
        self.Combo_esquinas.current(0)        
        self.Combo_esquinas.bind("<<ComboboxSelected>>",self.grafical_events)
        self.Combo_esquinas.config(width=10)

        self.h_p1 = tkinter.Button(self.ventana, text= "P1", command=lambda: self.grafical_events("p1"))
        self.h_p2 = tkinter.Button(self.ventana, text= "P2", command=lambda: self.grafical_events("p2"))
        self.h_p3 = tkinter.Button(self.ventana, text= "P3", command=lambda: self.grafical_events("p3"))

        self.Guardar_Button = tkinter.Button(self.ventana, text= "SAVE", command=lambda: self.grafical_events("guardar"))
        self.goto_Button = tkinter.Button(self.ventana, text= "GO TO POINT", command=lambda: self.grafical_events("goto"))
        self.delete_Button = tkinter.Button(self.ventana, text= "DELETE", command=lambda: self.grafical_events("delete"))

        self.anterior_in = tkinter.Button(self.ventana, text= "PREV", command=lambda: self.grafical_events("anterior_in"))
        self.siguiente_in = tkinter.Button(self.ventana, text= "NEXT", command=lambda: self.grafical_events("siguiente_in"))        

    def spawn_historial(self):
        start_row = 10
        start_column = 0

        self.anterior_in.grid(row=start_row + 1, column=start_column + 1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.siguiente_in.grid(row=start_row + 1, column=start_column + 2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        #self.historial_Label.grid(row=start_row,column=start_column+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)        
        self.Combo_puntos.grid(row=start_row+2,column=start_column, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.Combo_modo.grid(row=start_row+2,column=start_column+2, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        

        self.Guardar_Button.grid(row=start_row+4,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.goto_Button.grid(row=start_row+4,column=start_column+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.delete_Button.grid(row=start_row+4,column=start_column+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)        

    def hide_historial(self):
        self.anterior_in.grid_forget()
        self.siguiente_in.grid_forget()

        #self.historial_Label.grid_forget()
        self.Combo_modo.grid_forget()
        self.Combo_puntos.grid_forget()

        self.Guardar_Button.grid_forget()
        self.goto_Button.grid_forget()
        self.delete_Button.grid_forget() 

        self.Combo_plano.grid_forget()
        self.h_p1.grid_forget()
        self.h_p2.grid_forget()
        self.h_p3.grid_forget()
        self.Combo_esquinas.grid_forget()

    #------------------------------------------------------------------------------------- SECUENCIA

    def add_secuencia(self): 
        #self.secuencia_Label = tkinter.Label(self.ventana, text = "SEQUENCE", width = 10, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.point_Label = tkinter.Label(self.ventana, text = "P: 1", width = 5, height = 2, fg= self.Font_Color, bg= self.Color_Background)
        self.anterior_Button = tkinter.Button(self.ventana, text= "PREV", command=lambda: self.grafical_events("anterior"))
        self.siguiente_Button = tkinter.Button(self.ventana, text= "NEXT", command=lambda: self.grafical_events("siguiente"))
        self.start_Button = tkinter.Button(self.ventana, text= "START", command=lambda: self.grafical_events("start"))

        self.import_name = tkinter.Text(self.ventana, height=1, width=15)      
        self.import_name.insert(tkinter.INSERT, self.group_name)

        self.export_name = tkinter.Text(self.ventana, height=1, width=15)        
        self.export_name.insert(tkinter.INSERT, self.group_name)
  
        self.import_data = tkinter.Button(self.ventana, text="IMPORT", command=lambda: self.grafical_events("import"))
        self.export_data = tkinter.Button(self.ventana, text="EXPORT", command=lambda: self.grafical_events("export"))

    def spawn_secuencia(self):
        start_row = 20
        start_column = 0
        #self.set_position_on.config(width=2) 
        #self.secuencia_Label.grid(row=start_row,column=start_column+1, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.point_Label.grid(row=start_row+1,column=start_column+1, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.anterior_Button.grid(row=start_row+1,column=start_column+2, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.anterior_Button.config(width=2) 
        self.siguiente_Button.grid(row=start_row+1,column=start_column+3, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)
        self.siguiente_Button.config(width=2) 
        self.start_Button.grid(row=start_row+1,column=start_column, columnspan=1, padx=self.matrix_X, pady=self.matrix_Y)

        self.import_name.grid(row=start_row+2,column=start_column, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.export_name.grid(row=start_row+2,column=start_column+2, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)

        self.import_data.grid(row=start_row+3,column=start_column, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)
        self.export_data.grid(row=start_row+3,column=start_column+2, columnspan=2, padx=self.matrix_X, pady=self.matrix_Y)

    def hide_secuencia(self):        
        #self.secuencia_Label.grid_forget()
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