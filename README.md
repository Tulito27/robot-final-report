# Indoor Mobile Robot with LiDAR

## Main files for final report

1. firmware/esp32_diffdrive_controller.ino
   - Low-level motor and encoder control on ESP32
   - Serial communication with Raspberry Pi

2. ros2/serial_bridge_pkg/diffdrive_serial_bridge.py
   - ROS 2 bridge between Raspberry Pi and ESP32
   - Converts /cmd_vel to motor commands
   - Publishes /wheel_ticks and /odom

3. ros2/robot_bringup/launch/base.launch.py
   - Launches robot description, serial bridge, and LiDAR

4. ros2/robot_navigation/launch/navigation.launch.py
   - Launches autonomous navigation stack

## Project summary
This project implements an indoor mobile robot with LiDAR, wheel odometry, mapping, localization, and autonomous navigation using ROS 2.
