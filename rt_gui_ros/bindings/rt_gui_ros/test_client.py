#!/usr/bin/env python
import os

# Check ROS version from the environment
ros_version = os.getenv('ROS_VERSION', '1')

if ros_version == '1':
    # ROS 1 imports and setup
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

    def set_checklist(v):
        print(v)

    if __name__ == '__main__':
        rospy.init_node('test_client')
        client = RtGuiClient()

        rosparam.set_param_raw('test/int', 2)
        rosparam.set_param_raw('test/double', 3.0)
        rosparam.set_param_raw('test/bool', True)
        rosparam.set_param_raw('test/list', 'a')

        client.init()
        client.addInt('test', 'int', 1, 5, set_int, True)
        client.addDouble('test', 'double', 1, 5, set_double, True)
        client.addBool('test', 'bool', set_bool, True)
        client.addList('test', 'list', ['a', 'b', 'c', 'd', 'e'], set_list, True)
        client.addCheckList('test', 'checklist', ['a', 'b', 'c', 'd', 'e'], [True, False, False, False, False], set_checklist, True)

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            client.sync()
            rate.sleep()

elif ros_version == '2':
    # ROS 2 imports and setup
    import rclpy
    from rclpy.node import Node
    from rt_gui_ros2.py_client import RtGuiClient

    class TestParams(Node):
        def __init__(self):
            super().__init__('test_params_rclpy')

        def set_int(self, v):
            print(v)

        def set_double(self, v):
            print(v)

        def set_bool(self, v):
            print(v)

        def set_list(self, v):
            print(v)

    def main(args=None):
        rclpy.init(args=args)
        node = TestParams()

        client = RtGuiClient()
        client.init()
        client.addInt('test', 'int', 1, 5, 0, node.set_int, False)
        client.addDouble('test', 'double', 1, 5, 0, node.set_double, False)
        client.addBool('test', 'bool', True, node.set_bool, False)
        client.addList('test', 'list', ['a', 'b', 'c', 'd', 'e'], 'a', node.set_list, False)

        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == "__main__":
        main()
else:
    print("Unsupported ROS version. Set the ROS_VERSION environment variable to 1 or 2.")
