#!/usr/bin/env python3

"""
This script initializes a ROS2 node using rclpy, which publishes messages using a custom ThermostatInterface.
The ThermostatInterface is defined in the rwa2_kyatham package, and it is responsible for publishing messages
to a topic as defined within its implementation. This script is an entry point for running the ThermostatInterface,
setting it up, spinning it to keep it alive and processing data, and properly shutting it down afterwards.

Usage:
    To run this script, use the following command in a terminal:
    ```
    ros2 run rwa2_kyatham thermostat_demo.py
    ```
"""
# Import ROS Client Library for Python
import rclpy

# Import the custom ThermostatInterface class
from rwa2_kyatham.thermostat_interface import (
    ThermostatInterface,
)


def main(args=None):
    """
    Main function to initialize and run the ROS2 publisher node.

    Args:
        args (list, optional): Command-line arguments passed to the node. Defaults to None.
    """
    rclpy.init(args=args)  # Initialize the ROS client library
    node = ThermostatInterface("thermostat_house")  # Create an instance of the ThermostatInterface
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()


if __name__ == "__main__":
    main()  # Execute the main function when the script is run
