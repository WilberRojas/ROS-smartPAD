#!/usr/bin/env python3
import rospy
from sympy import*
import numpy as np 
from sensor_msgs.msg import JointState
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading

rospy.init_node('kinematics', anonymous=True)
bridge=CvBridge()
image_pub=rospy.Publisher("/whiteboard",Image, queue_size=1) 

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

size_whiteboard = 500
color = (10, 10, 10)

def new_whiteboard():    
    template = np.zeros((int(size_whiteboard*0.75), size_whiteboard, 3), dtype=np.uint8)
    template.fill(255) #Fondo blanco
    cv2.putText(template, 'Pizarra', (220, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
    hoja_p1 = (50, 55)
    hoja_p2 = (447, 338)
    cv2.rectangle(template, hoja_p1, hoja_p2, color, 3)
    return template

whiteboard = new_whiteboard()

def joint_callback(msg):

    # se recibe las pocisiones en un mensaje JointState
    angle1=msg.position[0]
    angle2=msg.position[1] 
    angle3=msg.position[2]
    angle4=msg.position[3]
    angle5=msg.position[4]

    # se aplica la multiplicacion de matrice
    T03n=T08.subs([(j1,angle1),
                    (j2,angle2),
                    (j3,angle3),
                    (j4,angle4),
                    (j5,angle5)])

    # se obtiene X,Y,Z
    
    x_point = T03n[0,3]
    y_point = T03n[1,3]
    z_point = T03n[2,3]

    # Rango de Z en que puede rallar:

    max_z = 0.155 #metros, se puede ver el dato actual en RVIZ
    min_z = 0.15 #metros

    if(z_point > min_z and z_point < max_z):
        zoom=1
        # ------- va de [-y, +y]
        y1 = 0.3 # extremo positivo
        x_pix = ((y_point + y1)/(2*y1)) * size_whiteboard * zoom

        # ------- va de [+x1, +x2] 
        x1 = 0.4 # punto inicio
        x2 = 0.8 # punto fin
        y_pix = ((x_point - x1)/(x2-x1)) * size_whiteboard*0.75 * zoom

        cv2.circle(whiteboard, (int(x_pix), int(y_pix)), 5, (255, 20, 20), -1)
    
    image_msg = bridge.cv2_to_imgmsg(whiteboard,"bgr8")
    image_pub.publish(image_msg)

def init_whiteboard():
    rospy.Subscriber("joint_states", JointState, joint_callback) 
    rospy.spin()

def Cleaner_callback(msg):
    int_value=msg.data
    print("CLEAR WHITEBOARD",int_value)
    new_whiteboard()


def init_cleaner():
    rospy.Subscriber("clear_whiteboard", Int32, Cleaner_callback) 
    print("Whiteboard_Cleaned")
    rate = rospy.Rate(1) # 1hz --> 1/1hz=1s
    while not rospy.is_shutdown():
        rate.sleep() # delay de 1 segundo

threads = [
    threading.Thread(target=init_whiteboard()),
    threading.Thread(target=init_cleaner())
]
for thread in threads:
    thread.start()

for thread in threads:
    thread.join()