import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    # This function is automatically executed when we create a new instance 
    # of SimpleSubscriber class.
    def __init__(self): 
        # initilize Node class by calling its constructor (init function of base class)
        super().__init__("simple_subscriber")

        # create new subscriber object which uses String interface 
        # String messages are defined within standard messages library (std_msgs.msg)
        self.sub = self.create_subscription(String, "chatter", self.msgCallback, 10)
    
    # message callback function is executed upon receiving a new message within chatter topic
    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)

def main():
    rclpy.init() #initialize ROS2 interface
    simple_subscriber = SimpleSubscriber() #create new instance of SimpleSubcriber class
    rclpy.spin(simple_subscriber) #keep subscriber node active so it can continue to receive new messages
    simple_subscriber.destroy_node()
    rclpy.shutdown() #shutdown ROS2 interface

if __name__ == '__main__':
    main()