#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
class ListenerNodesher(Node):
    def __init__(self):
        
        super().__init__('ultrasonic_listener')
        ultrasonic_addr= [0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27]
        self.subscriptions = []
        for addr in ultrasonic_addr:
            subcription = self.create_subscription(Int32,"Ultra{}".format(chr(65+addr)),self.listener_callback,20)
            self.subscriptions.append(subcription)
        
    def listener_callback(self,msg):
        self.get_logger().info("Publishing '{}'".format(msg.data))

def main(args= None):
    
    rclpy.init(args=args)
    my_publisher = ListenerNodesher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_nod()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

