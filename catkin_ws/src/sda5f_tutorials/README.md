# sda5f_tutorials

ROS package for YASKAWA MOTOMAN SDA5F tutorial.

## Dependencies

- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [ros-industrial/motoman](https://github.com/ros-industrial/motoman)
- [qqfly/motoman_sda5f_pkg](https://github.com/qqfly/motoman_sda5f_pkg)

## Usage

1. launch motoman driver
```bash  
roslaunch motoman_sda5f_support robot_interface_streaming_sda5f.launch robot_ip:=X.X.X.X controller:=fs100
```
2. enable joint motors  
```bash 
rosservice call robot_enable
```
3. launch moveit  
```bash
roslaunch motoman_sda5f_moveit_config demo.launch
```
4. execute tutorial demo  
```bash
roslaunch sda5f_tutorials [demo_name].launch
```

## Examples

1. wiggle  
```bash
roslaunch sda5f_tutorials wiggle.launch
```

<img src="../../../images/wiggle.gif" height="200">  
