#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rt_gui_ros2.py_client import RtGuiClient

class TestParams(Node):
    def __init__(self):
        super().__init__('test_params_rclpy')

    def set_int(v):
        print(v)

    def set_double(v):
        print(v)

    def set_bool(v):
        print(v)

    def set_list(v):
        print(v)
  
def main(args=None):
    rclpy.init(args=args)
    node = TestParams()

    client = RtGuiClient
    
    client.init()
    client.addInt('test','int',1,5,0,node.set_int,False)
    client.addDouble('test','double',1,5,0,node.set_double,False)
    client.addBool('test','bool',True,node.set_bool,False)
    client.addList('test','list',['a', 'b', 'c', 'd', 'e'],'a',node.set_list,False)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
