import rclpy
from rclpy.node import Node
from controlbot_msgs.srv import Addtwoints

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simple_service_server")
        
        self.create_service(Addtwoints,"addtwoints",self.serviceCallBack)
        self.get_logger().info("Add two ints srvice is ready")
        
    def serviceCallBack(self,req,res):
        
        self.get_logger().info("\nTwo inputs are a:%d  b=%d" %(req.a,req.b))
        res.sum = req.a + req.b 
        self.get_logger().info("\n The sum is %d" %res.sum)
        
        return res
    
def main():
    rclpy.init()
    simple_service_server=SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
        
        