from time import sleep
import rclpy
from std_msgs.msg import *
from rclpy.executors import SingleThreadedExecutor


class NodeControl():
    def __init__(self):
        rclpy.init()
        self.nodes = []
        self.executor = SingleThreadedExecutor()

    def addnode(self, node):
        if not self.executor._guard_condition:
            print('\nCannot add nodes while running!\n')
            exit()
        self.nodes.append(node)
        self.executor.add_node(self.nodes[-1].node)

    def removenode(self, name):
        destroy_list = [nod for nod in self.nodes
                        if nod.node.get_name() == name]
        self.nodes = [nod for nod in self.nodes if nod.node.get_name() != name]
        for nod in destroy_list:
            nod.cleanup()

    def printnodes(self):
        for nod in self.nodes:
            print(nod.node.get_name())

    def run(self):
        print('Running...\n')
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            self.__cleanup()

    def __cleanup(self):
        try:
            self.executor.shutdown()
            print()
            for node in self.nodes:
                node.cleanup()
            sleep(1)
            rclpy.shutdown()
            print('rclpy shutdown successful\n')
        except RuntimeError:
            print('Already cleaned up')
        except AttributeError:
            pass  # rclpy is poorly written?


if __name__ == '__main__':
    n = NodeControl()
    n.addnode(name='b', sub_chan='test', sub_data_type=String)
    n.addnode(name='a', pub_chan='test', pub_data_type=String,
              pub_rate=.5, pub_data='asdf')
    n.addnode(name='c')
    n.removenode('c')
    n.printnodes()
    n.run()
