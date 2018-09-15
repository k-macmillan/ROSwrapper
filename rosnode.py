#! /usr/bin/env python3.5

from collections import Iterable        # Determine if data is iterable
from std_msgs.msg import *              # Import all message types
from rclpy import create_node           # Only need to create a node


class RosNode(object):
    """ Simplifies creating ROS nodes """
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
        self.node = create_node(name)
        self.pub_data = pub_data
        self.pub_data_last = pub_data
        self.sub_data_type = sub_data_type
        self.pub_data_type = pub_data_type

        if sub_data_type is not None:
            self.subscriber = self.node.\
                create_subscription(sub_data_type,
                                    sub_chan,
                                    self.__subscribe
                                    if sub_func is None
                                    else sub_func)

        if pub_data_type is not None:
            self.pub_msg = pub_data_type()
            try:
                self.timer_period = 1.0 / pub_rate
                try:
                    if (not isinstance(self.pub_data,
                                       Iterable))\
                            or pub_data_type is String:
                        self.pub_msg.data = self.pub_data
                        self.timer = self.node.\
                            create_timer(self.timer_period,
                                         self.__publish)
                    else:
                        self.timer = self.node.\
                            create_timer(self.timer_period,
                                         self.__publish)
                except TypeError:
                    print('Type error!')
                    pass
            except AssertionError:
                print('Node: \"{}\" requires data to publish\n'.
                      format(self.node.get_name()))
                exit()
            except ZeroDivisionError:
                print('Node: \"{}\" cannot publish at a rate '
                      'of 0.0 times per second\n'.
                      format(self.node.get_name()))
                exit()

            self.publisher = self.node.create_publisher(pub_data_type,
                                                        pub_chan)
        print('Created node: ', self.node.get_name())

    def __subscribe(self, msg):
        """ Callback for subscriptions. Has to be done this way because ROS
            will not call the overriden inherited method so we call a base
            class method which then calls a possibly overriden method.
        """
        try:
            self.subscribe(msg)
        except BaseException as e:
            # Lets just catch them all and display the issue.
            print('Failed to subscribe due to a(n) {}!'
                  .format(type(e).__name__))

    def subscribe(self, msg):
        """ Posts msg to get_logger().info """
        self.node.get_logger().info('"%s"' % msg.data)

    def __sub_pub(self, msg):
        """ Callback for subs that pub. Has to be done this way because ROS
            will not call the overriden inherited method so we call a base
            class method which then calls a possibly overriden method.
        """
        self.sub_pub()

    def sub_pub():
        """ Handles nodes that both publish and subscribe. """
        print('TODO')

    def __publish(self):
        """ Callback for timed publishes. Has to be done this way because ROS
            will not call the overriden inherited method so we call a base
            class method which then calls a possibly overriden method.
        """
        try:
            self.publish()
        except BaseException as e:
            # Lets just catch them all and display the issue.
            print('Failed to publish due to a\
                  {} exception!'.format(type(e).__name__))

    def publish(self):
        """ Publishes what is currently in pub_msg.data. If pub_msg.data is
            iteritable it will instead call next on it.
        """
        if (not isinstance(self.pub_data, Iterable))\
                or self.pub_data_type is String:
                    self.publisher.publish(self.pub_msg)
        else:
            # Have to test for StopIteration unfortunately
            try:
                self.pub_data_last = next(self.pub_data)
            except StopIteration:
                pass
            self.pub_msg.data = self.pub_data_last
            self.publisher.publish(self.pub_msg)

    def cleanup(self):
        """ Destroys the node and lets the user know it was destroyed. """
        name = self.node.get_name()
        try:
            self.node.destroy_node()
            print('Destroyed node: ', name)
        except BaseException as e:
            print('Failed to destroy node due to a\
                  {} exception!'.format(type(e).__name__))
