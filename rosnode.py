from collections.abc import Sequence    # Determine if data is iterable
from types import GeneratorType
from rclpy import create_node           # ROS2 for python
from collections import Iterable
import std_msgs as msg


class RosNode(object):
    """ Simplifies creating ROS nodes """

    # instance variables declared as class variable set for performance
    attribs = {'name',
               'obj',
               'sub_data_type',
               'sub_chan',
               'pub_data_type',
               'pub_chan',
               'pub_rate',
               'pub_data', }

    @staticmethod
    def __is_msg_data_type(obj):
        for arg in dir(msg):
            if type(obj) is type(arg):
                return True
        return False

    @staticmethod
    def __seq_but_not_str(obj):
        return isinstance(obj, Sequence) and\
            not isinstance(obj, (str, bytes, bytearray))

    def __init__(self, **kwargs):
        """ Initializes the class """
        # keyword arguments
        self.name = 'node'
        self.obj = None
        self.sub_data_type = None
        self.sub_chan = 'sub_chan'
        self.pub_data_type = None
        self.pub_chan = 'pub_chan'
        self.pub_rate = 0.0
        self.pub_data = None
        self.valid = True       # Used to identify invalid state

        for key, value in kwargs.items():
            if key in self.attribs:
                setattr(self, key, value)
            else:
                print('\n\nWARNING:\
                      \nInvalid keyword argument in node constructor: {}\n\n'
                      .format(key))

        self.node = create_node(self.name)
        self.timer = None

        if self.sub_data_type is not None:
            try:
                self.__createsubs()
            except BaseException:
                self.cleanup()
                self.valid = False
                return

        if self.pub_data_type is not None:
            self.pub_msg = self.pub_data_type()
            try:
                if self.pub_rate != 0.0:
                    self.__maketimer()
            except AssertionError:
                print('Node: \"{}\" requires data to publish\n'.
                      format(self.node.get_name()))
                exit()

            self.publisher = self.node.create_publisher(self.pub_data_type,
                                                        self.pub_chan)
        print('Created node: ', self.node.get_name())

    def __createsubs(self):
        chan_len = 1
        type_len = 1

        # Check if sub_chan is a sequence
        if self.__seq_but_not_str(self.sub_chan):
            chan_len = len(self.sub_chan)
        # Now check if sub data types are equal to the number of channels
        if self.__seq_but_not_str(self.sub_data_type):
            type_len = len(self.sub_data_type)
            for d_type in self.sub_data_type:
                if self.__is_msg_data_type(d_type):
                    print('Unsuported message type: ', d_type)
                    raise

        if self.pub_data is None:
            sub_func = self.__subscribe
        else:
            sub_func = self.__sub_pub

        if chan_len == type_len and chan_len == 1:
            # Handle single case, make a tuple
            self.subscriber = (self.node
                               .create_subscription(self.sub_data_type,
                                                    self.sub_chan,
                                                    sub_func), )
        elif type_len == 1:
            # Multiple channels, single type
            for i in range(len(self.sub_chan)):
                self.subscriber = (self.node
                                   .create_subscription(self.sub_data_type,
                                                        self.sub_chan[i],
                                                        self.__subscribe, (self.sub_chan[i],)))
        elif chan_len == type_len:
            # Multiple channels, multiple types
            for i in range(len(self.sub_data_type)):
                self.subscriber = (self.node
                                   .create_subscription(self.sub_data_type[i],
                                                        self.sub_chan[i],
                                                        sub_func))
        else:
            print('sub_data_type count must equal sub_chan or be 1')
            raise

    def __maketimer(self):
        """ Generate a timer based on the pub_rate """
        timer_period = 1.0 / self.pub_rate
        try:
            self.timer = self.node.create_timer(timer_period, self.__publish)
        except TypeError:
            print('TypeError when setting timer!')
            pass

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
        if self.pub_data_type is not None:
            self.__sub_pub(msg)

    def subscribe(self, msg):
        """ Prints message """
        print(msg.data)

    def __sub_pub(self, msg):
        """ Callback for subs that pub. Has to be done this way because ROS
            will not call the overriden inherited method so we call a base
            class method which then calls a possibly overriden method.
        """
        self.sub_pub(msg)

    def sub_pub(self, msg):
        """ When a subscription is heard it is published to the publish channel
            if the pub_rate is 0.0. If the pub_rate is not 0.0 then it updates
            the message for the next time the publish timer goes off. Messages
            must be of the same type to pull off the latter type.
        """
        if self.timer is None:
            self.publisher.publish(msg)
        elif type(self.pub_data_type) == type(msg):
            self.pub_msg = msg
        else:
            print('Node: {} received an incompatible message type'.format(self.name))

    def __publish(self):
        """ Callback for timed publishes. Has to be done this way because ROS
            will not call the overriden inherited method so we call a base
            class method which then calls a possibly overriden method.
        """
        try:
            self.publish()
        except BaseException as e:
            # Lets just catch them all and display the issue.
            print('Node: {} failed to publish due to a(n) {}'.format(self.name, type(e).__name__))

    def publish(self):
        """ Publishes what is currently in pub_msg.data. If pub_msg.data is
            iteritable it will instead call next on it.
        """
        isIter = False
        try:
            if self.pub_data.__class__ is zip().__class__:
                isIter = True
            if self.__seq_but_not_str(self.pub_data):
                isIter = True
        except:
            pass

        if isIter:
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
            if self.timer is not None:
                self.node.destroy_timer(self.timer)
                print('Timer destroyed for: ', name)
            self.node.destroy_node()
            print('Destroyed node: ', name)
        except BaseException as e:
            print('Failed to destroy node due to a\
                  {} exception!'.format(type(e).__name__))
