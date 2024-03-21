# cartesian_impedance_control
ROS2 catestian_impedance_controller from PdZ

Prerequesites:
ROS2 humble
Libfranka 0.13.0 or newer
franka_ros2 v0.13.1

Clone this repository in the src directory of your franka_ros2_ws
```bash cd franka_ros2_ws/src
 git clone ________
```

Build the package or whole workspace 
```bash colcon build --packages-select cartesian_impedance_control --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release #Builds all the packages in your src folder
```

Source your setup
```bash source install/setup.sh
```

Launch the controller 
```bash ros2 launch cartesian_impeance_control cartesian_impedance_controller.launch.py
```

Launch the client if you want to adjust parameters
``` bash ros2 run cartesian_impedance_control user_input_client
```
