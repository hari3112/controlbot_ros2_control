import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
import math

class SimpleKinematics(Node):
    def __init__(self):
        super().__init__("simple_turtlesim_kinematics")
        self.turtle1_pose_sub=self.create_subscription(Pose,"/turtle1/pose",self.turtle1PoseCallback,10)
        self.turtle2_pose_sub=self.create_subscription(Pose,"/turtle2/pose",self.turtle2PoseCallback,10)
        
        self.turtle1_last_pose=Pose()
        
        self.turtle2_last_pose=Pose()
        
    def turtle1PoseCallback(self,msg):
        self.turtle1_last_pose=msg
        
    def turtle2PoseCallback(self,msg):
        self.turtle2_last_pose=msg
        
        Tx=self.turtle2_last_pose.x - self.turtle1_last_pose.x
        Ty=self.turtle2_last_pose.y - self.turtle1_last_pose.y
        
        theta_rad=self.turtle2_last_pose.theta - self.turtle1_last_pose.theta
        theta_deg=180*theta_rad/3.14
        
        self.get_logger().info(""""/n
                Translation vector of turtle1 -> turtle2
                
                Tx=%f
                Ty=%f
                
                Rotational vector of turtle1  -> turtle2
                theta_rad=%f
                theta_deg=%f
                
                |R11    R12| = |%f    %f|
                |R21    R22| = |%f    %f|""" %(Tx,Ty,theta_rad,theta_deg,math.cos(theta_rad),-math.sin(theta_rad),math.sin(theta_rad),math.cos(theta_rad)))
        
def main():
    rclpy.init()
    simple_turtlesim_kinematics=SimpleKinematics()
    rclpy.spin(simple_turtlesim_kinematics)
    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main()
    
                               
        
        