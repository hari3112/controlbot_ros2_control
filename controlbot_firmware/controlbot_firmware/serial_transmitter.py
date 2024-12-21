#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

class SimpleTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_node")
        self.declare_parameter("port","/dev/ttyUSB0")
        self.declare_parameter("baudrate",115200)
        
        self.port_= self.get_parameter("port").value
        self.baudrate_=self.get_parameter("baudrate").value
        
        
        self.arduino= serial.Serial(port=self.port_,baudrate=self.baudrate_,timeout=0.1)
        self.sub= self.create_subscription(String,"serial_transmitter",self.msgCallBack,10)
        
    def msgCallBack(self,msg):
        self.arduino.write(msg.data.encode("utf-8"))
        
def main():
    rclpy.init()
    simple_serial_node=SimpleTransmitter()
    rclpy.spin(simple_serial_node)
    simple_serial_node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()