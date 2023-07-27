#!/usr/bin/env python
import rospy
import rosparam
from rt_gui_ros.py_client import RtGuiClient

def set_int(v):
    print(v)
    
def set_double(v):
    print(v)

def set_bool(v):
    print(v)

def set_list(v):
    print(v)
  
if __name__ == '__main__':
    rospy.init_node('test_client')
    client = RtGuiClient
    
    rosparam.set_param_raw('test/int',2)
    rosparam.set_param_raw('test/double',3.0)
    rosparam.set_param_raw('test/bool',True)
    rosparam.set_param_raw('test/list','a')
    
    client.init()
    client.addInt('test','int',1,5,set_int,True)
    client.addDouble('test','double',1,5,set_double,True)
    client.addBool('test','bool',set_bool,True)
    client.addList('test','list',['a', 'b', 'c', 'd', 'e'],set_list,True)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        client.sync()
        rate.sleep()
