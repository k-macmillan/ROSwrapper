# ROSwrapper
ROSwrapper is a wrapper for ROS2's rclpy. ROSwrapper uses object oriented programming to turn nodes into clean, self-contained objects that can be added or removed from system.

## rclpy
To create a publish node with a timer in rclpy:
```python
def timer_callback():
    msg.data = 'Hello World'
    publisher.publish(msg)


rclpy.init()
node = rclpy.create_node('publisher')
publisher = node.create_publisher(String, 'topic')

msg = String()
timer_period = 0.5  # seconds
timer = node.create_timer(timer_period, timer_callback)

rclpy.spin(node)

node.destroy_timer(timer)
node.destroy_node()
rclpy.shutdown()
```
You can spin up a publisher in 13 lines of code using just rclpy. Additional publishers/subscribers are several lines each.

## ROSwrapper
To publish the same thing with ROSwrapper:
```python
nc = NodeControl()
nc.addnode(RosNode(name='publisher',
                   pub_data_type=String,
                   pub_chan='topic',
                   pub_rate=2,  # in Hz
                   pub_data='Hello World'))
nc.run
```
You can spin up a publisher in 3 lines of code using ROSwrapper. Additional publishers/subscribers are 1 line of code each. Nodes are added to the [NodeControl](nodecontrol.py) so everything can be cleanly executed and cleaned up.

## A RosNode
The core nodes are defined in the base class [RosNode](rosnode.py). RosNode was intended to be derived in the event the user wishes to see specific behavior.

- An example of it running with the base node type can be found in [example_basenode](example_basenode.py). 
- An example of a derived node class can be found in the [example_derivednode](example_derivednode.py) file. 

If a user has multiple subscribers you can use a switch statement to interpret the message. 

For more advanced usage you coud derive multiple classes from RosNode to handle each case. For example a robot may have IR sensors and wheels, each of which could have a custom class to deal with the information.

## Looking Forward
This still has quite a bit of work to be done before it's ready.


## Why this exists
I was tasked with coding a robot to navigate a course while utilizing ROS2. Unhappy with the examples and messy structure I created this wrapper.
