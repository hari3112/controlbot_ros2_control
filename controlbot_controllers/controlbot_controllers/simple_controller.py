#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from rclpy.constants import S_TO_NS
import numpy as np
from rclpy.time import Time
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")
        
        self.prev_pose_of_right_wheel=0.0
        self.prev_pose_of_left_wheel=0.0
        self.prev_time=self.get_clock().now()
        
        self.x_=0.0
        self.y_=0.0
        self.theta_=0.0
        
        
        self.wheel_cmd_pub=self.create_publisher(Float64MultiArray,"simple_velocity_controller/commands",10)
        self.vel_sub=self.create_subscription(TwistStamped,"controlbot_controllers/cmd_vel",self.VelCallback,10)
        self.joint_sub=self.create_subscription(JointState,"joint_states",self.jointCallBack,10)
        self.odom_pub=self.create_publisher(Odometry,"controlbot_controllers/odom",10)
        
        self.declare_parameter("wheel_radius",0.056)
        self.declare_parameter("wheel_separation",0.281)
        
        self.wheel_radius=self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation=self.get_parameter("wheel_separation").get_parameter_value().double_value
        
        self.get_logger().info("Using wheel radius : %f " %self.wheel_radius)
        self.get_logger().info("\n Using wheel separation length : %f " %self.wheel_separation)
        
        self.speed_conversion=np.array([[self.wheel_radius/2,self.wheel_radius/2],
                                  [self.wheel_radius/self.wheel_separation,- self.wheel_radius/self.wheel_separation]] )
        self.get_logger().info("\n The speed conversion is %s" %self.speed_conversion)
        
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id="odom"
        self.odom_msg.child_frame_id="base_link"
        self.odom_msg.pose.pose.orientation.x=0.0
        self.odom_msg.pose.pose.orientation.y=0.0
        self.odom_msg.pose.pose.orientation.z=0.0
        self.odom_msg.pose.pose.orientation.w=1.0
        
        
        
        
    def VelCallback(self,msg):
        
        
        robot_speed=np.array([[msg.twist.linear.x],
                                   [msg.twist.angular.z]])
        wheel_speed=np.matmul(np.linalg.inv(self.speed_conversion),robot_speed)
        
        wheel_speed_msg=Float64MultiArray()
        wheel_speed_msg.data=[wheel_speed[1,0],wheel_speed[0,0]]       
        
        self.wheel_cmd_pub.publish(wheel_speed_msg)
        
    def jointCallBack(self,msg):
        
        change_in_left_wheel_pose=msg.position[1]-self.prev_pose_of_left_wheel
        change_in_right_wheel_pose=msg.position[0]-self.prev_pose_of_right_wheel
        self.get_logger().info ("msg_position_1:   %f    and  msg_position 2:   %f" %(msg.position[1],msg.position[0]))
        
        change_in_time=Time.from_msg(msg.header.stamp) - self.prev_time
      
        self.prev_pose_of_left_wheel=msg.position[1]
        self.prev_pose_of_right_wheel=msg.position[0]
        self.prev_time=Time.from_msg(msg.header.stamp)
        
        fi_left= change_in_left_wheel_pose/(change_in_time.nanoseconds/S_TO_NS)
        fi_right = change_in_right_wheel_pose/(change_in_time.nanoseconds/S_TO_NS)
        
        linear_vel= (self.wheel_radius*fi_right + self.wheel_radius*fi_left)/2
        angular_vel = (self.wheel_radius*fi_right - self.wheel_radius*fi_left)/self.wheel_separation
        
       
        
        d_s= (self.wheel_radius * change_in_right_wheel_pose + self.wheel_radius * change_in_left_wheel_pose)/2
        d_theta= (self.wheel_radius * change_in_right_wheel_pose - self.wheel_radius * change_in_left_wheel_pose)/self.wheel_separation
        self.theta_ += d_theta
        self.x_ += d_s*math.cos(self.theta_)
        self.y_ +=d_s*math.sin(self.theta_)
        
        #self.get_logger().info("Linear : %f  and   Angular : %f" %(linear_vel,angular_vel))
        #self.get_logger().info("x : %f , y : %f , theta : %f" %(self.x,self.y,self.theta))
        
        q=quaternion_from_euler(0 ,0 ,self.theta_)
        self.odom_msg.pose.pose.orientation.x=q[0]
        self.odom_msg.pose.pose.orientation.y=q[1]
        self.odom_msg.pose.pose.orientation.z=q[2]
        self.odom_msg.pose.pose.orientation.w=q[3]
        self.odom_msg.pose.pose.position.x= self.x_
        self.odom_msg.pose.pose.position.y= self.y_
        self.odom_msg.twist.twist.linear.x=linear_vel
        self.odom_msg.twist.twist.angular.z=angular_vel
        
        self.odom_pub.publish(self.odom_msg)
        

        
        
def main():
    rclpy.init()
    simple_controller=SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()