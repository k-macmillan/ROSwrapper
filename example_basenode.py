from std_msgs.msg import String       # For sample
from rosnode import RosNode           # Base node class
from nodecontrol import NodeControl   # Controller class

if __name__ == '__main__':
    # Sample usage

    # NOTE: This will not output to the console
    nc = NodeControl()
    # nc.addnode(RosNode(name='b', sub_chan='test', sub_data_type=String))
    # nc.addnode(RosNode(name='a', pub_chan='test', pub_data_type=String,
    #                    pub_rate=2, pub_data='asdf'))
    # nc.addnode(RosNode(name='c'))
    # nc.removenode('c')
    # nc.printnodes()

    nc.addnode(RosNode(name='publisher',
                       pub_chan='topicasdf',
                       pub_data_type=String,
                       pub_data='Hello World',
                       pub_rate=2,
                       asdf='asdf'))    # asdf is a sample false keyword
    nc.run()
