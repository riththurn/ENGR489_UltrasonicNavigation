#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range
from std_msgs.msg import Int32
from std_msgs.msg import Header
import sys
sys.path.append('../')
sys.path.insert(1,'/home/ubuntu/Documents/engr489/testenv/eeen489_robot/src/ultrasonics/Lib_UltrasonicSource/DFRobot_URM13-master/python/raspberrypi/example')
from DFRobot_URM13 import *
from csv import writer 
from datetime import datetime

class MyPublisher(Node):
    def __init__(self):
        self.failedAttmptsRetry = 0
        
        super().__init__('ultrasonic_publisher')
        self.declare_parameter('addr',0x20)
        #self.get_logger().info('Please check that sensor {} is properly connected'.format(self.get_parameter('addr')))
        self.publisher_ = self.create_publisher(Range,"Ultra{}".format(chr(65+self.get_parameter('addr').value)),10)
        self.sensor = DFRobot_URM13_I2C(i2c_addr = self.get_parameter('addr').value, bus = 1)
        
        timer_period = 0.1

        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.sensor_running = False
        self.sensor_measure_mode_set = False
    def timer_callback(self):
        try:
         #   while (self.sensor.begin() == False and self.sensor_running == False):
         #       self.get_logger().warn('Please check that sensor {} is properly connected'.format(self.get_parameter('addr')))
         #   if(self.sensor_running == False):
         #       self.sensor_running = True
                
            if(self.sensor_measure_mode_set ==False):
                self.sensor_measure_mode_set = True
                self.sensor.set_measure_mode(self.sensor.E_INTERNAL_TEMP | 
                            self.sensor.E_TEMP_COMP_MODE_EN | 
                            self.sensor.E_AUTO_MEASURE_MODE_DIS | 
                            self.sensor.E_MEASURE_RANGE_MODE_SHORT)
                self.sensor.set_external_tempreture_C(20.2)
                self.sensor.set_measure_sensitivity(0x10)


            self.sensor.passive_measurement_TRIG()
            distance_cm = self.sensor.get_distance_cm()
            msg = Range()
            msg.field_of_view = float(0.698132) #60 deg cone documented cone (1.0472) too wide, attempting 0.698132 (40deg) 
            msg.radiation_type = 0

            msg.min_range=0.15
            msg.max_range=9.0
            msg.header=Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "Ultra{}".format(chr(65+self.get_parameter('addr').value))
            msg.range = (distance_cm/100)
            self.publisher_.publish(msg)
            #self.get_logger().info("Publishing '{}'".format(msg.range))
        except:
            if(self.failedAttmptsRetry == 100):
                self.get_logger().info("Sensor '{}' is cuurrently not working, check connection.".format(self.get_parameter('addr').value))
                self.failedAttmptsRetry = 0
            else:
                self.failedAttmptsRetry += 1
                
def main(args= None):
    
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_nod()
    
    rclpy.shutdown()
if __name__ == '__main__':
    main()

