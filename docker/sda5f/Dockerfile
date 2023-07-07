FROM nvidia/opengl:1.2-glvnd-devel-ubuntu16.04

RUN apt-get update && apt-get install -y --no-install-recommends curl lsb-release wget gnupg
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-desktop-full \
    ros-kinetic-gazebo-ros-pkgs \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-control-toolbox \
    ros-kinetic-moveit \
    ros-kinetic-effort-controllers \
    ros-kinetic-forward-command-controller \
    ros-kinetic-joint-state-controller \
    ros-kinetic-joint-trajectory-controller \
    ros-kinetic-position-controllers \
    ros-kinetic-velocity-controllers \
    ros-kinetic-tf \
    ros-kinetic-tf2-tools \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-genpy \
    ros-kinetic-industrial-core \
    ros-kinetic-roslint \
    python-catkin-tools \
    byobu \
    libudev-dev \
    python-pip \
    git \
    vim \
    net-tools \
    build-essential \
    dbus

ADD bringup.bash /
ADD wiggle.bash /
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
CMD [ "source /opt/ros/kinetic/setup.bash" ]
