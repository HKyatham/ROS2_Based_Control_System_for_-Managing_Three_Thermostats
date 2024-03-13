# ROS2_Based_Control_System_for_ Managing_Three_Thermostats
 
The Requirements for this project and step involved in building one are stated below:

1. Generate a random temperature (between 0°C and 40°C) and store this value in your class as the current temperature. Temperatures should be represented as integer values.
2. Publish the randomly generated temperature data to topic temperature at a frequency of 3 Hz.
3. In the same node, use a subscriber to listen to t temperature and displays the received message as Current temperature: X where X is the current temperature from step 1.
4. Test your node with TERMINAL
```
ros2 run
```
Node thermostat_house has two parameters:
Parameter thermostat_name : Name of the thermostat (string).
+ Default value: "thermostat"
Parameter target_temperature : The target temperature for the thermostat to reach (int).
+ Default value: 30 (representing a target temperature of 30°C)

1. Store these parameters in FILE-ALT config/params.yaml (create FILE-ALT params.yaml )
2. Edit FILE rwa2.launch.py : Get the path to the parameter file. Pass the path to the node object.
3. In your program, declare, retrieve, and store these parameters.
4. Check everything is working with TERMINAL ros2 launch You can either modify your program to display the parameter values or use TERMINAL ros2 param get (recommended)

The CLI argument mode can be used to set the thermostat to different modes:
+ away – When this mode is used, the thermostat is turned off. This is the default mode.
+ eco – When used, it reduces the target temperature by 3 degrees to save energy.
+ night – When used, it automatically adjusts the target temperature to a preset comfortable sleeping temperature at night (18°C)

1. Edit FILE rwa2.launch.py to incorporate the argument mode
2. Modify your program to incorporate this argument:
+ When in away mode, the publisher stops publishing.
+ In eco mode, the target temperature decreases by 3 degrees. Adjust the current temperature up or down by 1 degree and continue publishing until the target temperature is achieved. Do not go below 0°C or above 40°C.
+ While in night mode, the target temperature is established at 18°C. Alter the current temperature by 1 degree, either lowering or raising it, and keep publishing until reaching the target temperature. Do not go below 0°C or above 40°C.
3. Pass this argument to TERMINAL ros2 launch with different values.

Use a parameter callback to handle parameter changes at runtime. Only the parameter mode can be changed at runtime. If the user tries to change parameter thermostat_name or parameter target_temperature , display a message saying this is not allowed.

1. Create a parameter callback in your program to provide the user the ability to change the thermostat mode.
2. Run your node with TERMINAL ros2 launch
3. In a different terminal, try to change the thermostat mode with TERMINAL ros2 param set
4. Ensure the parameter callback properly handles the changes.

Change the thermostat’s mode every 10 seconds by using a timer. The specific mode to be activated next will be determined based on the thermostat’s existing mode.

Use the following logic in the timer:
1. If the current mode is “away”, set it to “night”
2. If the current mode is “night”, set it to “eco”
3. If the current mode is “eco”, set it to “away”

Until now, a single thermostat managed the temperature for the entire house. However, after receiving a raise at work and achieving a more affluent status, I’ve opted to upgrade my home by installing two additional thermostats.
+ The n thermostat_kitchen node is responsible for regulating the kitchen’s temperature, with a set target of 12°C. This thermostat is identified by the name "kitchen" and publishes temperature data to the t kitchen_temperature topic.
+ The n thermostat_livingroom node is responsible for regulating the living room’s temperature, with a set target of 14°C. This thermostat is identified by the name "livingroom" and publishes temperature data to the t livingroom_temperature topic.
+ The n thermostat_bedroom node is responsible for regulating the bedroom’s temperature, with a set target of 15°C. This thermostat is identified by the name "bedroom" and publishes temperature data to the t bedroom_temperature topic.

1. Edit FILE params.yaml and add entries for the new nodes.
2. Edit FILE rwa2.launch.py to only start the new nodes.
3. Remap the node name and topic name for each node object.
4. Test everything with TERMINAL ros2 launch

To run this package, 

clone the project using the command, to your src folder of your ROS workspace.
```
git clone 
```

Build the project using the colcon build.
```
colcon build --packages-select rwa2_kyatham
```

Source the package using the command
```
source install/setup.bash
```

Run the node using the ros2 launch command
```
ros2 launch rwa2_kyatham rwa2.launch.py
```

The output looks something like below:
![Images/Exercise2.png](https://github.com/HKyatham/ROS2_Based_Control_System_for_-Managing_Three_Thermostats/blob/main/Images/output.png)
