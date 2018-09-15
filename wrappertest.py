#! /usr/bin/env python3.5

from nodecontrol import NodeControl     # Node controller
from example_nodeclass import NodeClass
from std_msgs.msg import *

if __name__ == '__main__':
    # Sample usage
    nc = NodeControl()
    nc.addnode(NodeClass(name='a',
                         pub_data_type=String,
                         pub_chan='/testInheritance',
                         pub_rate=2,
                         pub_data='test'))

    nc.addnode(NodeClass(name='b',
                         sub_data_type=String,
                         sub_chan='/testInheritance'))

    # pts = [[0.0, 0.5, 1.0], [3.0, 2.0, 1.0]]
    # pts = zip(pts[0], pts[1])

    # nc.addnode(NodeClass(name='c',
    #                      pub_data_type=Float32MultiArray,
    #                      pub_chan='/testFloatMulti',
    #                      pub_rate=5,
    #                      pub_data=pts))

    nc.run()
