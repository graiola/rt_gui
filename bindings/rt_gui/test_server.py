#!/usr/bin/env python
import rospy
import rosparam
from rt_gui import py_server
  
if __name__ == '__main__':
    rospy.init_node('test_server', anonymous=True)
    server = py_server.RtGuiServer
    server.run('test')
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
