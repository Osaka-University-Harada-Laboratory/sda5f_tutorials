# sda5f_stack

ROS metapackage for YASKAWA Motoman SDA5F dual arm robot.

## Usage

### simulation
```bash
roslaunch motoman_sda5f_support test_sda5f.launch         # display robot in rviz
roslaunch motoman_sda5f_moveit_config demo_fake.launch    # use moveit on simulation
roslaunch motoman_sda5f_moveit_config demo_sim.launch     # use moveit on simulation
```

### real robot
1. Set IP address (e.g. ip: 10.0.0.21, subnet mask: 255.255.255.0) for the external computer  
2. Connect robot with an ethernet cable  
3. Launch motoman_driver  

```bash
roslaunch motoman_sda5f_support bringup.launch robot_ip:=10.0.0.2 controller:=fs100
roslaunch motoman_sda5f_support bringup.launch robot_ip:=10.0.0.2 controller:=fs100 fake_execution:=true
```
