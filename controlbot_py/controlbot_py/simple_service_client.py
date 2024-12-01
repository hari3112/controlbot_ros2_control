import rclpy
from rclpy.node import Node
from controlbot_msgs.srv import Addtwoints
import sys

class simpleServiceClient(Node):
    def __init__(self,a,b):
        super().__init__("simple_service_client")
        
        self.client=self.create_client(Addtwoints,"addtwoints")
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for server\n")
            
        self.req=Addtwoints.Request()
        self.req.a=a
        self.req.b=b
        
        self.future=self.client.call_async(self.req)
        self.future.add_done_callback(self.responseCallBack)
        
    def responseCallBack(self,future):
        self.get_logger().info("\nThe response is %d" %future.result().sum)
      
def main():
    rclpy.init()
    if   len(sys.argv)!=3:
        print("Wrong number of arguments")
        return -1
    
    simple_service_client=simpleServiceClient(int(sys.argv[1]),int(sys.argv[2]))
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
        
        