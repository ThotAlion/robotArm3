import rclpy,os,numpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
import pypot.robot


class Pypot_node(Node):
    """
    This class wraps a pypot robot to communicate with classical control topics
    
    Attributes
    ----------

    Methods
    ----------

    Subscribed topics
    ----------

    Published topics
    ----------


    """

    def __init__(self):
        """
        Constructor
        """
        super().__init__('pypot_node')
        self.jointPublisher = self.create_publisher(JointState, 'joint_states', 10)
        self.jointSubscription = self.create_subscription(JointState,'joint_cmd',self.listener_callback,10)
        
        self.declare_parameter('configFilePath', '')
        # self.add_on_set_parameters_callback(self.parameters_callback)
        self.configFilePath = self.get_parameter('configFilePath').get_parameter_value().string_value
        self.pypotRobot = None
        if os.path.isfile(self.configFilePath):
            print("open robot :",self.configFilePath)
            self.pypotRobot = pypot.robot.from_json(self.configFilePath)
        else:
            print("The config file does not exist :",self.configFilePath)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    # def parameters_callback(self, params):
    #     """
    #     Callback when a parameter is changed
    #     Here, only select configFilePath (type STRING) in which is the path of the pypot config file

    #     Parameters
    #     ----------
    #     params : array of object. Each object has attributes :
    #         - name : name of the ROS2 parameter
    #         - value : value of the ROS2 parameter
    #         - type_ : type of the ROS2 parameter
    #     """
    #     for param in params:
    #         if param.name == "configFilePath":
    #             self.configFilePath = param.value
    #             print("Got the new file path",self.configFilePath)
    #     return SetParametersResult(successful=True)

    def listener_callback(self, msg):
        for i_name in range(len(msg.name)):
            name = msg.name[i_name]
            value = msg.position[i_name]
            for m in self.pypotRobot.motors:
                if m.name == name:
                    if numpy.isnan(value) or m.present_temperature>55.0:
                        m.compliant = True
                    else:
                        m.compliant = False
                        m.goal_position = 180.0*value/numpy.pi

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = super().get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        for m in self.pypotRobot.motors:
            msg.name.append(m.name)
            msg.position.append(m.present_position*numpy.pi/180.0)
            msg.velocity.append(m.present_speed*numpy.pi/180.0)
            msg.effort.append(m.present_current)
        self.jointPublisher.publish(msg)

    def destroy_node(self):
        super().destroy_node()
        self.pypotRobot.compliant = True
        self.pypotRobot.close()



def main(args=None):
    rclpy.init(args=args)

    ppn = Pypot_node()

    rclpy.spin(ppn)

    ppn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
