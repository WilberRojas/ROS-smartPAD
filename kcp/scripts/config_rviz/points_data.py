#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

rospy.init_node('plot_points', anonymous=True)	


bridge=CvBridge()
image_pub=rospy.Publisher("/points_data",Image, queue_size=1) 

size_layout = 500
layout_template = np.zeros((int(size_layout*0.72), size_layout, 3), dtype=np.uint8)
layout_template.fill(255) #Fondo blanco
cv2.putText(layout_template, 'Puntos Guardados', (120, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (10, 10, 10), 2, cv2.LINE_AA)


#array con posicion x,y para cada punto
point_x = np.arange(40, 500, 46).tolist()
point_y = np.arange(50, 350, 30).tolist()

color = (0, 0, 200) #rojo
tickness_ball = 10
for y in point_y:
    for x in point_x:
        im_point = (x, y)
        cv2.circle(layout_template, im_point, tickness_ball, color, -1)

layout = layout_template.copy()
layout_filled = layout_template.copy()

def callback(data):
    global layout, color, layout_filled

    if data.x == -1:
        layout = layout_template.copy()
    else:
        #----------------------------------BORRAR | PTP o LIN
        if data.x == 1:
            if data.z == 2:
                color = (0, 200, 0) #verde LIN
            elif data.z == 1:
                color = (200, 0, 0) #azul PTP
        elif data.x == 0:        
            color = (0, 0, 200) #rojo
        elif data.x == 2:
            layout = layout_filled.copy()
            color =(0,255,255) #amarillo
        
        #----------------------------------Circle Position
        index = list(str(int(data.y))) #transformamos float to list: 12.0 --> [1,2]

        if len(index) == 1:
            index1 = int(index[0])
            im_point = (point_x[index1], point_y[0])
            cv2.circle(layout, im_point, tickness_ball, color, -1)

        elif len(index) == 2:
            index2 = int(index[0])
            index1 = int(index[1])
            im_point = (point_x[index1], point_y[index2])
            cv2.circle(layout, im_point, tickness_ball, color, -1)
        
        if data.x != 2:
            layout_filled = layout.copy()

    image_msg = bridge.cv2_to_imgmsg(layout,"bgr8")
    image_pub.publish(image_msg)

sub = rospy.Subscriber("position_list", Point, callback)
rospy.spin()