from time import sleep                              # Cleanup safety
import rclpy                                        # ROS for python
from std_msgs.msg import String                     # For sample
from rclpy.executors import SingleThreadedExecutor  # To spins all nodes
from rosnode import RosNode                         # Base node class


class NodeControl():
    """ Primary controller for RosNodes and derivatives of RosNode. """
    def __init__(self):
        rclpy.init()
        self.nodes = []
        self.executor = SingleThreadedExecutor()

    def addnode(self, node):
        """ Adds a node to the nodes list to be ran by the executor. """
        if not self.executor._guard_condition:
            print('\nCannot add nodes while running!\n')
            exit()
        self.nodes.append(node)
        self.executor.add_node(self.nodes[-1].node)

    def removenode(self, name):
        """ Removes a node by name from the nodes list.
            Will remove all nodes of this name!
        """
        destroy_list = [nod for nod in self.nodes
                        if nod.node.get_name() == name]
        self.nodes = [nod for nod in self.nodes if nod.node.get_name() != name]
        for nod in destroy_list:
            nod.cleanup()

    def printnodes(self):
        """ Prints all nodes in the nodes list by name. """
        for nod in self.nodes:
            print(nod.node.get_name())

    def run(self):
        """ Fires up the executor and handles keyboard interrupts. """
        print('Running...\n')
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            self.__cleanup()
            exit(0)

    def __cleanup(self):
        """ Cleans up the ROS nodes and properly shuts down rclpy. """
        try:
            self.executor.shutdown()
            print('\nCleaning up...\n')
            for node in self.nodes:
                node.cleanup()
            # Added 1s delay to let nodes close
            sleep(1)
            rclpy.shutdown()
            print('rclpy shutdown successful\n')
        except RuntimeError:
            print('Already cleaned up')
        except AttributeError:
            pass  # rclpy is poorly written?


if __name__ == '__main__':
    # Sample usage

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
                       pub_rate=2))
    nc.run()
