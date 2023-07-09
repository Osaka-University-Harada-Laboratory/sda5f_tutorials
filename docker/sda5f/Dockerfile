FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04

RUN apt-get update && apt-get install -y --no-install-recommends curl lsb-release wget gnupg
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends tzdata
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-control-toolbox \
    ros-noetic-moveit \
    ros-noetic-effort-controllers \
    ros-noetic-forward-command-controller \
    ros-noetic-joint-state-controller \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-position-controllers \
    ros-noetic-velocity-controllers \
    ros-noetic-tf \
    ros-noetic-tf2-tools \
    ros-noetic-robot-state-publisher \
    ros-noetic-genpy \
    ros-noetic-industrial-core \
    ros-noetic-roslint \
    ros-noetic-trac-ik-kinematics-plugin \
    python3-osrf-pycommon \
    python3-catkin-tools \
    python3-rosdep \
    byobu \
    libudev-dev \
    python3-pip \
    git \
    vim \
    net-tools \
    build-essential \
    libcanberra-gtk* \
    dbus \
    software-properties-common \
    imagemagick

RUN mkdir -p /tmp/
RUN mkdir -p /catkin_ws/src && \
	cd /catkin_ws/src && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace" && \
	cd /catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build" && \
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
	echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/catkin_ws" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=/catkin_ws" >> ~/.bashrc

WORKDIR /catkin_ws/src/
RUN git clone https://github.com/ros-industrial/industrial_core.git -b melodic --depth 1
RUN git clone https://github.com/Osaka-University-Harada-Laboratory/onrobot.git --depth 1
RUN git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git --depth 1
WORKDIR /catkin_ws/
RUN pip install pyassimp==4.1.3 --ignore-installed

# Generates bash commands

## sda5f_rviz.sh
RUN echo "roslaunch motoman_sda5f_support test_sda5f_with_onrobot.launch" >> ~/.bash_history

## sda5f_moveit.sh
RUN echo "roslaunch motoman_sda5f_moveit_config demo_sim.launch" >> ~/.bash_history

## sda5f_pp_fake.sh
RUN echo "roslaunch motoman_sda5f_support bringup.launch robot_ip:=10.0.0.2 controller:=fs100 fake_execution:=true" >> ~/.bash_history
RUN echo "rosrun sda5f_motion_plan object_tf_publisher.py" >> ~/.bash_history
RUN echo "roslaunch sda5f_motion_plan demo_display.launch fake_execution:=true planning_space:=joint" >> ~/.bash_history

## sda5f_pp.sh
RUN echo "roslaunch motoman_sda5f_support bringup.launch robot_ip:=10.0.0.2 controller:=fs100 fake_execution:=false" >> ~/.bash_history
RUN echo "roslaunch sda5f_motion_plan demo_display.launch fake_execution:=false planning_space:=joint" >> ~/.bash_history

WORKDIR /catkin_ws/src/
