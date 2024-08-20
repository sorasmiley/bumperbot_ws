import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
  def __init__(self): #constructor
    super().__init__("simple_publisher") #call constructor of base class 
    
    # publisher will publish string messages
    # specify type of message (String class), string message(name of topic), size of queue works as a buffer
    self.pub_ = self.create_publisher(String, "chatter", 10)
    
    # count number of messages we publish within chatter topic
    self.counter_ = 0 
    # frequency at which we publish messages
    # set to 1 hertz, so publish 1 message per second in chatter topic
    self.frequency_ = 1.0
    # print informative message
    self.get_logger().info("Publishing at %d Hz" % self.frequency_)
    
    # create new ROS2 timer using create_timer function that executes a specific function repeatedly
    # frequency we want to execute function, name of function we execute at each timer expiration
    self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
  
  # publish a new message within chatter topic every time this callback function is executed
  # so every timer expires, we call this callback function
  def timerCallback(self):
    msg = String()
    msg.data = "Hello ROS 2 - counter: %d" % self.counter_
    self.pub_.publish(msg) #use publisher object to publish new message
    self.counter_ += 1 #increment by one

def main():
  rclpy.init()
  simple_publisher = SimplePublisher()
  rclpy.spin(simple_publisher)
  simple_publisher.destroy_node() #ensure publisher node is destroyed
  rclpy.shutdown() #shutdown ROS2 interface

if __name__ == "__main__":
  main()