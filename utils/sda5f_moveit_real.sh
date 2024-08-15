#!/bin/bash

byobu new-session -d -s moveit
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0

byobu send-keys -t 0 'xhost + && docker exec -it sda5f_container bash -it -c "roslaunch motoman_sda5f_support bringup.launch robot_ip:=10.0.0.2 controller:=fs100 fake_execution:=false"' 'C-m'
sleep 3.
byobu send-keys -t 0 'xhost + && docker exec -it sda5f_container bash -it -c "roslaunch motoman_sda5f_moveit_config demo.launch"' 'C-m'

byobu attach -t moveit