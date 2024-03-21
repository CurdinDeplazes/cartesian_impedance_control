# cartesian_impedance_control
### ROS2 catestian_impedance_controller from PdZ

Prerequesites:
* ROS2 humble <br />
* Libfranka 0.13.0 or newer <br />
* franka_ros2 v0.13.1 <br />

For further informtaion, please refer to the [Franka ROS2 FCI documentation](https://support.franka.de/docs/franka_ros2.html)

Once you have everything setup, follow the steps below to get the controller running.

Clone this repository in the src directory of your franka_ros2_ws: <br />
```bash
cd franka_ros2_ws/src 
git clone https://github.com/CurdinDeplazes/cartesian_impedance_control.git
```

Clone the messages package in the src directory: <br />
```bash
git clone https://github.com/CurdinDeplazes/messages_fr3.git
```

Build the package or whole workspace: <br />
```bash
colcon build --packages-select cartesian_impedance_control --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release #Builds all the packages in your src folder
```

Source your setup: <br />
```bash
source install/setup.sh 
```

Launch the controller: <br />
```bash
ros2 launch cartesian_impeance_control cartesian_impedance_controller.launch.py
```

Launch the client if you want to adjust parameters: <br />
``` bash
ros2 run cartesian_impedance_control user_input_client 
```
