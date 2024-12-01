import rclpy
from rclpy import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.declare_parameter("simple_int_param",32)
        self.declare_parameter("simple_string_param","haris")
        self.add_on_set_parameters_callback(self.ParamCheckCallback)
        
    def ParamCheckCallback(self,paramet):
        result=SetParametersResult()
        for param in paramet:
            if paramet.name=="simple_int_param" and paramet.type==Parameter.Type.INTEGER:
                self.get_logger().info("Param simple_int_param changed! New value is %d" %paramet.value)
                result.successful=True
                
            if paramet.name=="simple_string_param" and paramet.type==Parameter.Type.STRING:
                self.get_logger().info("Param simple_int_param changed! New value is %s" %paramet.value)
                result.successful=True
                
        return result
    
    
def main():
    rclpy.init()
    simple_parameter=SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
    main()
    
    