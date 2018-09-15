#! /usr/bin/env python3.5

from rosnode import RosNode
from collections import Iterable
from std_msgs.msg import *


class MyPub(RosNode):
    def __init__(self,
                 name='node',
                 sub_data_type=None,
                 sub_chan='sub_chan',
                 sub_func=None,
                 pub_data_type=None,
                 pub_chan='pub_chan',
                 pub_func=None,
                 pub_rate=0.0,
                 pub_data=None,):
        super(MyPub, self).__init__(name,
                                    sub_data_type,
                                    sub_chan,
                                    sub_func,
                                    pub_data_type,
                                    pub_chan,
                                    pub_func,
                                    pub_rate,
                                    pub_data)
        print('Sent to super')

    def publish(self):
        if (not isinstance(self.pub_data, Iterable))\
                or self.pub_data_type is String:
                    self.pub_msg.data =\
                        '\"All your publishes are belong to us\"'
                    self.publisher.publish(self.pub_msg)
        else:
            # Have to test for StopIteration unfortunately
            try:
                self.pub_data_last = next(self.pub_data)
            except StopIteration:
                pass
            self.pub_msg.data = (0.0, 0.0)
            self.publisher.publish(self.pub_msg)
        print('Overriden to: ', self.pub_msg.data)

    def subscribe(self, msg):
        print('We got a message: ', msg.data)
