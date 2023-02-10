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

rospy.init_node("dvfdxv", anonymous=True)
filename="kr6_lucho"
pik_file = "/home/wilber/robotica/dip_ws/src/ROS-smartPAD/kcp/scripts/sequences/kr6_lucho.dat"
with open(pik_file, "rb") as f:
    try:
        rate = rospy.Rate(10)
        pointmessaje = Point(-1, 0, 0)        
        rate.sleep()
        
        list_data = pickle.load(f)
        Joint_points = list_data[0][0]
        Cartesian_points = list_data[0][1]
        mov_type = list_data[0][2]
        main_positions_list = list_data[1]
        [table_position, table_dimensions] = list_data[2]
        [tool_position ,tool_dimensions] = list_data[3]

        print("Cartesian_points", Cartesian_points[4])
     
        
        
        
    except EOFError:
        pass