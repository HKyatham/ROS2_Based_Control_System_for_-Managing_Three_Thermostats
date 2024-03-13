# pull in some Python launch modules.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


# This function is needed
def generate_launch_description():
    """
    The function will help with launching the of the nodes at the same time.
    """
    # Launch descriptor object. 
    ld = LaunchDescription()

    # Declare a command-line argument "mode".
    cmd_line_parameter = DeclareLaunchArgument(
        "mode",
        default_value="away",
        description="A parameter from the command line.",
    )

    # Path to the parameters file.
    node_params = PathJoinSubstitution(
        [FindPackageShare("rwa2_kyatham"), "config", "params.yaml"]
    )

    # Python node thermostat_house.
    thermostat_py = Node(
        package="rwa2_kyatham",
        executable="thermostat_demo.py",
        parameters=[
            {"mode": LaunchConfiguration("mode")},
            node_params
        ],
    )
    
    # Python node thermostat_kitchen.
    thermostat_kitchen = Node(
        package="rwa2_kyatham",
        executable="thermostat_demo.py",
        name="thermostat_kitchen",
        parameters=[
            {"mode": LaunchConfiguration("mode")},
            node_params
        ],
        remappings=[("temperature","thermostat_kitchen")]
    )
    
    # Python node thermostat_livingroom.
    thermostat_livingroom = Node(
        package="rwa2_kyatham",
        executable="thermostat_demo.py",
        name="thermostat_livingroom",
        parameters=[
            {"mode": LaunchConfiguration("mode")},
            node_params
        ],
        remappings=[("temperature","thermostat_livingroom")]
    )
    
    # Python node thermostat_bedroom.
    thermostat_bedroom = Node(
        package="rwa2_kyatham",
        executable="thermostat_demo.py",
        name="thermostat_bedroom",
        parameters=[
            {"mode": LaunchConfiguration("mode")},
            node_params
        ],
        remappings=[("temperature","thermostat_bedroom")]
    )

    # Adding the nodes and command line parameters to Launch descriptor object.
    ld.add_action(cmd_line_parameter)
    #ld.add_action(thermostat_py)
    ld.add_action(thermostat_kitchen)
    ld.add_action(thermostat_livingroom)
    ld.add_action(thermostat_bedroom)

    return ld
