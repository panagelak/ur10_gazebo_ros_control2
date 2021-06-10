# ur10_gazebo_ros_control2
This holds the packages to debug the ur10 with gazebo ros2 control plugin

# Start Gazebo with controllers only
```bash
ros2 launch ur10_gazebo ur10_gazebo.launch.py 
```

# Start Gazebo RViz and Moveitcpp with a demo node
```bash
ros2 launch ur10_moveit_demo_nodes ur10_gazebo_demo.launch.py
```
## Prerequisities

on workspace the commit of gazebo_ros2_control with id **a33ea2**

moveit from source (ros control gets installed with moveit)