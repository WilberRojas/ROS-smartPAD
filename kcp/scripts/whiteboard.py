#!/usr/bin/env python3
import rospy
from sympy import*
import numpy as np 
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('kinematics', anonymous=True)
bridge=CvBridge()
image_pub=rospy.Publisher("/whiteboard",Image, queue_size=1) 
j1=Symbol('j1')
j2=Symbol('j2')
j3=Symbol('j3')
j4=Symbol('j4')
j5=Symbol('j5')

pub = rospy.Publisher('dh_position', Point, queue_size=10)

def dh_matrix (t,d,a,aph):
	T=Matrix([[cos(t), -sin(t)*cos(aph),sin(t)*sin(aph), a*cos(t)],
            [sin(t), cos(t)*cos(aph), -cos(t)*sin(aph), a*sin(t)],
            [0,sin(aph), cos(aph), d],
            [0, 0, 0, 1]])
	return T

T01 = dh_matrix(0,0.4,0.0,pi)
T12 = dh_matrix(j1, 0.0, 0.025, pi/2)
T23 = dh_matrix(j2, 0.0, 0.0, pi/2)
T34 = dh_matrix(0, 0.0, 0.455, -pi/2)
T45 = dh_matrix(j3,0.0,0.0,pi/2)
T56 = dh_matrix(pi/2,0.035,0.0,-pi/2)
T67 = dh_matrix(j4-pi/2,-0.42,0.0,-pi/2)
T78 = dh_matrix(j5,0.0,0.0,pi/2)
T89 = dh_matrix(0,-0.08,0.0,0)

T02 = T01*T12
T03 = T02*T23
T04 = T03*T34
T05 = T04*T45
T06 = T05*T56
T07 = T06*T67
T08 = T07*T78
T09 = T08*T89

print("px= ", T09[0,3])
print("py= ", T09[1,3])
print("pz= ", T09[2,3])
print("---------------------------------")

size_whiteboard = 500
color = (10, 10, 10)
whiteboard = np.zeros((int(size_whiteboard*0.75), size_whiteboard, 3), dtype=np.uint8)
whiteboard.fill(255) #Fondo blanco

def clear_whiteboard(data):
    global whiteboard
    int_value=data.data
    print("CLEAR WHITEBOARD",int_value) 
    whiteboard = np.zeros((int(size_whiteboard*0.75), size_whiteboard, 3), dtype=np.uint8)   

def joint_callback(msg):    
    cv2.putText(whiteboard, 'Pizarra', (220, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
    angle1=msg.position[0]
    angle2=msg.position[1] 
    angle3=msg.position[2]
    angle4=msg.position[3]
    angle5=msg.position[4]
    T03n=T09.subs([(j1,angle1),
                    (j2,angle2),
                    (j3,angle3),
                    (j4,angle4),
                    (j5,angle5)])
    """  print("x= ", T03n[0,3])
    print("y= ", T03n[1,3])
    print("z= ", T03n[2,3])
    print("---------------------------------") """
    
    x_point = T03n[0,3]
    y_point = T03n[1,3]
    z_point = T03n[2,3]

    if(z_point > 0.15 and z_point < 0.18):
        zoom=1
        # ------- va de [-y, +y]
        y1 = 0.3 # extremo positivo
        x_pix = ((y_point + y1)/(2*y1)) * size_whiteboard * zoom      

        # ------- va de [+x1, +x2] 
        x1 = 0.4 # punto inicio
        x2 = 0.8 # punto fin
        y_pix = ((x_point - x1)/(x2-x1)) * size_whiteboard*0.75 * zoom        

        print(x_pix, y_pix, whiteboard.shape)
        #Hoja 
        

        
        cv2.circle(whiteboard, (int(x_pix), int(y_pix)), 5, (255, 20, 20), -1)        

    """ cv2.imshow("Whiteboard", whiteboard)    
    cv2.waitKey(1) """
    hoja_p1 = (50, 55)
    hoja_p2 = (447, 338)
    cv2.rectangle(whiteboard, hoja_p1, hoja_p2, color, 3)
    image_msg = bridge.cv2_to_imgmsg(whiteboard,"bgr8")
    image_pub.publish(image_msg)    



def init_whiteboard():
    rospy.Subscriber("joint_states", JointState, joint_callback) 
    rospy.spin()

def destroy_whiteboard():
    cv2.destroyAllWindows()

init_whiteboard()
    