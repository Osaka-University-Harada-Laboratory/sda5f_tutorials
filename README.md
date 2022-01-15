# sda5f_tutorials

ROS package for YASKAWA MOTOMAN SDA5F tutorial.

## Dependencies

- ROS Kinetic
- motoman driver

## Installation

    $ cd catkin_ws
    $ git clone -b kinetic-devel https://github.com/ros-industrial/motoman.git src/motoman
    $ git reset --hard $SHA1
    $ git clone git@github.com:qqfly/motoman_sda5f_pkg.git src/motoman_sda5f_pkg
    $ git clone git@github.com:Osaka-University-Harada-Laboratory/sda5f_tutorials.git src/sda5f_tutorials
    $ rosdep update
    $ rosdep install --from-paths src/ --ignore-src --rosdistro kinetic
    $ cd catkin_ws; catkin build

## Usage

1. launch motoman driver  
    `$ roslaunch motoman_sda5f_support robot_interface_streaming_sda5f.launch robot_ip:=X.X.X.X controller:=fs100`
2. enable joint motors  
    `$ rosservice call robot_enable`
3. launch moveit  
    `$ roslaunch motoman_sda5f_moveit_config demo.launch`
4. execute tutorial demo  
    `$ roslaunch sda5f_tutorials wiggle.launch`

## Author / Contributor

[Takuya Kiyokawa](https://takuya-ki.github.io/)

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
