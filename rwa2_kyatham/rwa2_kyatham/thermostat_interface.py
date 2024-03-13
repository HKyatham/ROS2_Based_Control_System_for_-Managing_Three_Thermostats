from random import randint
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Int32

class ThermostatInterface(Node):
    """
    A ROS2 ThermostatInterface node that sends Int32 messages to the 'temperature'
    topics, at 1-second intervals.

    The messages contain a fixed message, which demonstrates a simple way to publish data continuously in ROS2.

    Attributes:
        _current_temp (Int): A Random integer between 0 and 40 to initialize the current temperature.
        _default_temp (Int): A Int to store the temperature after receiving from target temperature.
        _temp_data (Int): A Int to store the temperature after receiving it in subscriber call back.
        _temp_publisher (Publisher): The ROS2 publisher object for sending messages to 'temperature'.
        _temp_subscriber (Subscriber): The ROS2 subscriber object for receiving messages from 'temperature'.
        _temp_timer (Timer): A timer object that triggers the temp publishing event every 3 second.
        _mode_timer (Timer): A timer object that triggers the mode change event every 10 second.
        _mode (String): A String to store the mode parameter value.
        _thermostat_name (String): A String to store the thermostat_name parameter value.
        _target_temperature (Int32): A Int to store the target_temperature parameter value.

    Args:
        node_name (str): The name of the node.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        
        self._current_temp = randint(0, 40)
        
        self._default_temp = 0
        
        self._temp_data = 0
        
        self._temp_publisher = self.create_publisher(Int32, "temperature", 10)
        
        self._temp_subscriber = self.create_subscription(
            Int32, "temperature", self.receive_temp_message, 10
        )
        
        self._temp_timer = self.create_timer(3, self.temp_publish_message)
        
        self._mode_timer = self.create_timer(10, self.autoMode)
        
        # Declare descriptors
        mode_descriptor = ParameterDescriptor(
            description="Mode of the thermostat(string).",
        )
        thermostat_descriptor = ParameterDescriptor(
            description="Name of the thermostat(string).",
        )
        target_temperature_descriptor = ParameterDescriptor(
            description="The target temperature for the thermostat to reach (int).",
        )
        # Declare parameters
        self.declare_parameter("mode", "away", mode_descriptor)
        self.declare_parameter("thermostat_name", "thermostat", thermostat_descriptor)
        self.declare_parameter("target_temperature", 30, target_temperature_descriptor)
        
        # Get the parameters
        self._mode = (
            self.get_parameter("mode").get_parameter_value().string_value
        )
        self._thermostat_name = (
            self.get_parameter("thermostat_name").get_parameter_value().string_value
        )
        self._target_temperature = (
            self.get_parameter("target_temperature").get_parameter_value().integer_value
        )
        
        self._default_temp = self._target_temperature
        
        # Function call to set the target temperature and log the mode and target temperature
        self.mode()

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    def parameters_callback(self, params):
        """
        Callback function for the parameters
        
        Args:
        params (dictionary): The dict of params in key value pairs.
        """
        success = False
        for param in params:
            if param.name == "thermostat_name":
                if param.type_ == Parameter.Type.STRING:  # validation
                    success = False
                    self.get_logger().error("❌ Not allowed to change the name of the thermostat")  # Log error message
            elif param.name == "target_temperature":
                if param.type_ == Parameter.Type.INTEGER:  # validation
                    success = False
                    self.get_logger().error("❌ Not allowed to change the value of the target temperature")  # Log error message
            elif param.name == "mode":
                if param.type_ == Parameter.Type.STRING and param.value in ["eco","night","away"]:  # validation
                    success = True
                    self._mode = param.value  # modify the attribute
                    self.mode()
                else:
                    success = False
                    self.get_logger().error(f"There is no mode with name {param.value}")
        return SetParametersResult(successful=success)
    
    def mode(self):
        """
        A Funtion to log the mode of the thermostat and target temperature based on mode.
        
        """
        self.get_logger().info(f"Mode set to: {self._mode}")  # Log info message
        if(self._mode=="eco"):
            self._target_temperature = self._default_temp - 3
            self.get_logger().info(f"Target temperature is: {self._target_temperature}")  # Log info message
            # End of if
        elif(self._mode=="night"):
            self._target_temperature = 18
            self.get_logger().info(f"Target temperature is: {self._target_temperature}")  # Log info message
            # End of else if
    # End of mode
        
    def temp_publish_message(self):
        """
        Callback function for the timer event. This function constructs the message to be published,
        and logs the message to the ROS2 logger.

        The message is current temperature values. This function will not publish if the value of mode is 'away'
        """
        # Initialize the message object that will be published.
        self._temp_msg = Int32()
        # Set the message data.
        self._temp_msg.data = self._current_temp
        # Publish the message.
        if(self._target_temperature > self._current_temp):
            self._current_temp += 1
            # End of if
        elif(self._target_temperature < self._current_temp):
            self._current_temp -= 1
            # End of else if
        
        if(self._mode!="away"):
            self._temp_publisher.publish(self._temp_msg)
            # End of if
        #self.get_logger().info(f"Publishing temperature: {self._current_temp.data}")
    # End of publisher call back.
        
    def receive_temp_message(self, msg):
        """Handle incoming messages on the "temperature" topic.

        This function is called when a new message is received on the "temperature" topic. It stores this data 
        and logs it.

        Args:
            msg (std_msgs.msg.Int32): The received message object, containing the Int32 data.
        """
        #self.get_logger().info(f"Receiving: {msg.data}")
        self._temp_data = msg.data
        # Logs the received message data.
        self.get_logger().info(f"Current temperature: {self._temp_data}")
    # End of subscriber call back.
        
    def autoMode(self):
        """
        Callback function for the timer event. This function changes the mode based on current mode and prints a log message.
        """
        if(self._mode == 'away'):
            self._mode = 'night'
            self.mode()
            # End of if
        elif(self._mode == 'night'):
            self._mode = 'eco'
            self.mode()
            # End of else if
        elif(self._mode == 'eco'):
            self._mode = 'away'
            self.mode()
            # End of else if
    # End of call back function.