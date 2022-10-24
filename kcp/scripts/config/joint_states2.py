#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('joint_states2', anonymous=True)
pub_cleaner = rospy.Publisher('custom_joint_states', JointState, queue_size=1)

def callback(data):
    pub_cleaner.publish(data)
    
sub = rospy.Subscriber("joint_states", JointState, callback)
rospy.spin()