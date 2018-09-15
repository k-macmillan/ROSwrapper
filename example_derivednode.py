from rosnode import RosNode             # Base class
from collections import Iterable        # Determine if data is iterable
from std_msgs.msg import String         # Message type
from nodecontrol import NodeControl     # Controller class


class NodeClass(RosNode):
    """ This is an example of what a class would look like if you wanted to
        override RosNode so you can have custom publish/subscribe functions.
    """
    def __init__(self, **kwargs):
        super(NodeClass, self).__init__(**kwargs)

    def publish(self):
        """ Example overriden publish method """
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
        """ Example overriden subscribe method """
        print('We got a message: ', msg.data)


if __name__ == '__main__':
    # Sample usage
    nc = NodeControl()
    nc.addnode(NodeClass(name='a',
                         pub_data_type=String,
                         pub_chan='/testInheritance',
                         pub_rate=5,
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
