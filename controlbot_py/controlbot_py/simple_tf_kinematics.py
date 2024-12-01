import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster,TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from controlbot_msgs.srv import GetTransform
from tf_transformations import quaternion_from_euler,quaternion_multiply,quaternion_inverse

class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")
        self.last_transform_x=0.0
        self.x_increment=0.05
        self.rotations_counter=0
        self.last_orientation=quaternion_from_euler(0,0,0)
        self.orientation_increment=quaternion_from_euler(0,0,0.05)
       
        
        
        self.static_tf_broadcaster=StaticTransformBroadcaster(self)
        self.tf_broadcaster=TransformBroadcaster(self)
        
        self.tf_stamped=TransformStamped()
        self.static_tf_stamped=TransformStamped()
        
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        
        self.static_tf_stamped.header.stamp=self.get_clock().now().to_msg()
        self.static_tf_stamped.header.frame_id="controlbot_bottom"
        self.static_tf_stamped.child_frame_id="controlbot_top"
        
        self.static_tf_stamped.transform.translation.x=0.0
        self.static_tf_stamped.transform.translation.y=0.0
        self.static_tf_stamped.transform.translation.z=0.5
        
        self.static_tf_stamped.transform.rotation.x=0.0
        self.static_tf_stamped.transform.rotation.y=0.0
        self.static_tf_stamped.transform.rotation.z=0.0
        self.static_tf_stamped.transform.rotation.w=1.0
        
        self.static_tf_broadcaster.sendTransform(self.static_tf_stamped)
        self.get_logger().info("publishing static TF between %s and %s " %(self.static_tf_stamped.header.frame_id , self.static_tf_stamped.child_frame_id))
        
        self.timer=self.create_timer(0.1,self.timerCallback)
        
        self.get_transform=self.create_service(GetTransform,"get_transform",self.GetTransformserviceCallBack)
        
    def timerCallback(self):
        
        
        self.static_tf_stamped.header.stamp=self.get_clock().now().to_msg()
        self.tf_stamped.header.frame_id="controlbot_bottom"
        self.tf_stamped.child_frame_id="odom"
        self.tf_stamped.transform.translation.x=self.last_transform_x+self.x_increment
        self.tf_stamped.transform.translation.y=0.0
        self.tf_stamped.transform.translation.z=0.0
        q=quaternion_multiply(self.last_orientation,self.orientation_increment)
        self.tf_stamped.transform.rotation.x=q[0]
        self.tf_stamped.transform.rotation.y=q[1]
        self.tf_stamped.transform.rotation.z=q[2]
        self.tf_stamped.transform.rotation.w=q[3]
        self.last_orientation=q
        self.rotations_counter +=1
        
        if self.rotations_counter>=100:
            
            self.orientation_increment=quaternion_inverse(self.orientation_increment)
            self.rotations_counter=0
            
            
        
        self.last_transform_x=self.tf_stamped.transform.translation.x
        
        self.tf_broadcaster.sendTransform(self.tf_stamped)
        
        
    def GetTransformserviceCallBack(self,req,res):
        
        self.get_logger().info("\nThe service is ready for the tranform between %s and %s " %(req.frame_id,req.child_frame_id))
        service_tf_requested=TransformStamped()
        try:
            service_tf_requested=self.tf_buffer.lookup_transform(req.frame_id,req.child_frame_id,rclpy.time.Time())
        except TransformException as e:
            self.get_logger().error("An error occured")
            res.success=False
            return res
        res.transform=service_tf_requested
        res.success=True
        return res
        
def main():
    rclpy.init()
    simple_tf_kinematics=SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()
            
if __name__=='__main__':
    main()
        
        