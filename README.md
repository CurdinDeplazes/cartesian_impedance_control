# cartesian_impedance_control
### ROS2 catestian_impedance_controller from Pd|Z

Prerequisites:
* ROS2 humble <br />
* Libfranka 0.13.0 or newer <br />
* franka_ros2 v0.13.1 <br />

For further information, please refer to the [Franka ROS2 FCI documentation](https://support.franka.de/docs/franka_ros2.html)

Once you have everything set up, follow the steps below to get the controller running.

Clone this repository in the src directory of your franka_ros2_ws: <br />
```bash
cd franka_ros2_ws/src 
git clone https://github.com/CurdinDeplazes/cartesian_impedance_control.git
```
For the moment, you need to add the following lines of code, to your controllers.yaml file inside franka_ros2/franka_bringup/config/:
```bash
cartesian_impedance_controller:
      type: cartesian_impedance_control/CartesianImpedanceController
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

If not yet done, ensure your setup is always source by adding the following line to your .bashrc file (to get access to it, you need to execute `nano .bashrc` in your home directory). : <br />
```bash
source /home/<user>/franka_ros2_ws/src/install/setup.sh 
```

Launch the controller: <br />
```bash
ros2 launch cartesian_impedance_control cartesian_impedance_controller.launch.py
```

Launch the client if you want to adjust parameters: <br />
``` bash
ros2 run cartesian_impedance_control user_input_client 
```
