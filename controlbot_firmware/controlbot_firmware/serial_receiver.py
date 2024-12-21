#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleReceiver(Node):
    def __init__(self):
        super().__init__("simple_receiver_node")
        self.pub=self.create_publisher(String,"simple_receiver",10)
        self.frequency=0.1
        self.get_logger().info("Publishing at %d hz" %self.frequency)
        
        self.declare_parameter("port","/dev/ttyUSB0")
        self.declare_parameter("baudrate",115200)
        self.port_=self.get_parameter("port").value
        self.baudrate_=self.get_parameter("baudrate").value

        
        self.arduino=serial.Serial(port=self.port_,baudrate=self.baudrate_)
        self.timer=self.create_timer(self.frequency,self.timerCallback)
        
    def timerCallback(self):
        if rclpy.ok() and self.arduino.is_open:
            data=self.arduino.readline()
            try:
                data.decode("utf-8")
            except:
                return
            
            msg=String()
            msg.data=str(data)
            self.pub.publish(msg)
            
def main():
    rclpy.init()
    simple_receiver_node=SimpleReceiver()
    rclpy.spin(simple_receiver_node)
    simple_receiver_node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
                
        
        