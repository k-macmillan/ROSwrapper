from time import sleep                              # Cleanup safety
import rclpy                                        # ROS2 for python
from rclpy.executors import SingleThreadedExecutor  # To spins all nodes


class NodeControl():
    """ Primary controller for RosNodes and derivatives of RosNode. """
    def __init__(self):
        rclpy.init()
        self.nodes = []
        self.executor = SingleThreadedExecutor()

    def addnode(self, node):
        """ Adds a node to the nodes list to be ran by the executor. """

        if node.valid is False:
            print('Node created incorrectly, did not add a node')
            return None

        if not self.executor._guard_condition:
            print('\nCannot add nodes while running!\n')
            exit()

        self.nodes.append(node)
        self.executor.add_node(self.nodes[-1].node)

        return self.nodes[-1]

    def removenode(self, name):
        """ Removes all nodes with a given name from the nodes list.
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
        if len(self.nodes) == 0:
            print('No nodes to run')
            self.__cleanup()

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
            pass  # rclpy issue?
