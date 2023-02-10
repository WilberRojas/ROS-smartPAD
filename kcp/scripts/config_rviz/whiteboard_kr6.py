#!/usr/bin/env python3
import rospy
from sympy import*
import numpy as np 
from sensor_msgs.msg import JointState
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('whiteboard', anonymous=True)

j1=Symbol('j1')
j2=Symbol('j2')
j3=Symbol('j3')
j4=Symbol('j4')
j5=Symbol('j5')

def dh_matrix (t,d,a,aph):
	T=Matrix([[cos(t), -sin(t)*cos(aph),sin(t)*sin(aph), a*cos(t)],
            [sin(t), cos(t)*cos(aph), -cos(t)*sin(aph), a*sin(t)],
            [0,sin(aph), cos(aph), d],
            [0, 0, 0, 1]])
	return T

T01 = dh_matrix(0,0.4,0.0,pi)
T12 = dh_matrix(j1, 0.0, 0.025, pi/2)
T23 = dh_matrix(j2, 0.0, 0.455, 0)
T34 = dh_matrix(j3,0.0,0.0,pi/2)
T45 = dh_matrix(pi/2,0.035,0.0,-pi/2)
T56 = dh_matrix(j4-pi/2,-0.42,0.0,-pi/2)
T67 = dh_matrix(j5,0.0,0.0,pi/2)
T78 = dh_matrix(0,-0.08,0.0,0)

T02 = T01*T12
T03 = T02*T23
T04 = T03*T34
T05 = T04*T45
T06 = T05*T56
T07 = T06*T67
T08 = T07*T78

class Main:
    def __init__(self, T0N):
    
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("/whiteboard",Image, queue_size=1)
        rospy.Subscriber("custom_joint_states", JointState, self.joint_callback)

        self.T0N = T0N
        self.size_whiteboard = 500
        self.color_text = (10, 10, 10)
        #template
        self.multiplier = 0.714
        self.template = np.zeros((int(self.size_whiteboard*self.multiplier), self.size_whiteboard, 3), dtype=np.uint8)
        self.template.fill(255) #Fondo blanco
        
        #cv2.putText(self.template, 'Pizarra', (220, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color_text, 2, cv2.LINE_AA)
        
        hoja_p1 = (2, 2)
        hoja_p2 = (498, 355)
        cv2.rectangle(self.template, hoja_p1, hoja_p2, self.color_text, 3)
        #frame
        self.whiteboard = self.template.copy()
        #template 2
        self.template_number = np.zeros((int(self.size_whiteboard*0.1), self.size_whiteboard, 3), dtype=np.uint8)
        self.template_number.fill(255) #Fondo blanco

        #corrections
        self.previus_x_pix = 0
        self.previus_y_pix = 0
        self.previus_z = 0

    def joint_callback(self, msg):
        #print("Running")

        # se recibe las pocisiones en un mensaje JointState
        angle1=msg.position[0]
        angle2=msg.position[1] 
        angle3=msg.position[2]
        angle4=msg.position[3]
        angle5=msg.position[4]

        # se aplica la multiplicacion de matrices
        T03n=self.T0N.subs([(j1,angle1),
                        (j2,angle2),
                        (j3,angle3),
                        (j4,angle4),
                        (j5,angle5)])

        # se obtiene X,Y,Z
        x_point = T03n[0,3]
        y_point = T03n[1,3]
        z_point = T03n[2,3]

        
        # Se borra la imagen si:
        if angle1 > 10:
            self.whiteboard = self.template.copy()

        # Rango de Z en que puede rayar:
        max_z = 45.0+10 #cm
        min_z = 44.8+10 #cm

        max_z = max_z/100 #metros, se puede ver el dato actual en RVIZ
        min_z = min_z/100 #metros

        #-----------Numero en otro frame
        distance_frame = self.template_number.copy()
        if z_point >= min_z:
            if z_point <= max_z:
                self.color_text = (10, 200, 10)
            else:
                self.color_text = (200, 10, 10)
            diff_z_cm = (z_point - min_z)*100
            algo = "%.2f" % diff_z_cm
            #print (algo, type(algo))
            cv2.putText(distance_frame, 'Tool: +' + algo + "cm", (40, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.color_text, 1, cv2.LINE_AA)
        else: 
            self.color_text = (10, 10, 200)
            cv2.putText(distance_frame, "Tool: COLLISION!", (50, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.color_text, 1, cv2.LINE_AA)
        
        

        if(z_point > min_z and z_point < max_z):
            zoom=1
            # ------- va de [-y, +y]
            y1 = 0.14 # extremo positivo
            x_pix = ((y_point + y1+0.015)/(2*y1)) * self.size_whiteboard * zoom

            # ------- va de [+x1, +x2]
            x1 = 0.42 # punto inicio
            x2 = 0.63 # punto fin
            y_pix = ((x_point - x1)/(x2-x1)) * self.size_whiteboard * zoom *self.multiplier #MULTIPLIER

            
            #cv2.circle(self.whiteboard, (int(x_pix), int(y_pix)), 5, (255, 20, 20), -1)
            
            if(self.previus_z > min_z and self.previus_z < max_z):
                #print("distance OK:", round(self.previus_z*100,2), round(z_point*100,2))
                if self.previus_x_pix!=0:
                    if self.previus_x_pix != x_pix and self.previus_y_pix != y_pix:
                        #print("Line OK", (self.previus_x_pix, self.previus_y_pix), (int(x_pix), int(y_pix)))
                        cv2.line(self.whiteboard, (int(x_pix), int(y_pix)), (self.previus_x_pix, self.previus_y_pix), (0, 255, 0), thickness=5)
                        print("rviz(x,y)",[round(x_point,2), round(y_point,2)], "pixel(y,x)", [int(y_pix),int(x_pix)])
                
            self.previus_z = z_point
            self.previus_x_pix = int(x_pix)
            self.previus_y_pix = int(y_pix)
        else:
            self.previus_z = 0
            self.previus_x_pix = 0
            self.previus_y_pix = 0

        output_frame = im_v = cv2.vconcat([distance_frame, self.whiteboard])        
        image_msg = self.bridge.cv2_to_imgmsg(output_frame,"bgr8")
        self.image_pub.publish(image_msg)
    
    def run(self):
        rospy.spin()        

app = Main(T08)
app.run()