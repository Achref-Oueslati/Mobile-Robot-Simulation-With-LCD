# Mobile Robot Simulation

A ROS 2 differential-drive robot simulation with Gazebo Harmonic, RViz, a simulated lidar, and an optional Arduino LCD that displays joint positions. The workspace includes Python and C++ implementations; the C++ package also provides a `ros2_control` launch mode.

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy
- Gazebo Harmonic and the ROS-Gazebo integration packages
- `xacro`, `robot_state_publisher`, and `rviz2`
- `python3-serial` for the Python LCD node
- `teleop_twist_keyboard` for keyboard control

For the C++ `ros2_control` launch mode, also install `ros2_control`, `ros2_controllers`, `gz_ros2_control`, `diff_drive_controller`, and `joint_state_broadcaster`.

## Build

From the workspace root, with ROS 2 Jazzy installed:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch

Run one mode at a time in a sourced terminal:

```bash
# Python implementation
ros2 launch mobile_robot_py bringup.launch.py

# C++ implementation with the Gazebo differential-drive plugin
ros2 launch mobile_robot_cpp bringup.launch.py

# C++ implementation with ros2_control
ros2 launch mobile_robot_cpp bringup_ros2_control.launch.py
```

Each launch starts the simulator, spawns the robot, and starts the ROS-Gazebo bridge and RViz. The `ros2_control` mode uses `/diff_drive_controller/cmd_vel` with `geometry_msgs/msg/TwistStamped`; the other modes use `/cmd_vel` with `geometry_msgs/msg/Twist`.

## Drive the Robot

For the Python and standard C++ modes, run this in another sourced terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

For the C++ `ros2_control` mode:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel -p stamped:=true
```

## Arduino LCD (Optional)

Upload [`grove_lcd_display.ino`](src/mobile_robot_cpp/arduino/grove_lcd_display/grove_lcd_display.ino) to the Arduino and connect it over USB. The LCD node reads `/joint_states` and sends up to two formatted joint positions to a 16×2 display over serial (`/dev/ttyACM0`, 115200 baud by default). Both implementations retry if the serial device is unavailable, so the simulation can run without the LCD connected.

On Linux, grant your user serial-port access if needed, then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

## Packages

- [`mobile_robot_py`](src/mobile_robot_py/): Python LCD node and standard Gazebo differential-drive simulation.
- [`mobile_robot_cpp`](src/mobile_robot_cpp/): C++ LCD node, standard simulation, and `ros2_control` simulation.

Both packages contain their own robot model, launch files, bridge parameters, RViz configuration, and Arduino sketch. The C++ package's `config/mobile_robot_controllers.yaml` configures the `ros2_control` controllers.
