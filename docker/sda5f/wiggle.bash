#!/bin/bash
byobu new-session -d -s wiggle
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0
byobu split-window -h
byobu select-pane -t 2
byobu split-window -h
byobu select-pane -t 3

byobu send-keys -t 0 'roslaunch motoman_sda5f_support robot_interface_streaming_sda5f.launch robot_ip:=10.0.0.2 controller:=fs100' 'C-m'
sleep 5.
byobu send-keys -t 1 'rosservice call robot_enable' 'C-m'
sleep 5.
byobu send-keys -t 2 'roslaunch motoman_sda5f_moveit_config demo.launch' 'C-m'
sleep 5.
byobu send-keys -t 3 'roslaunch sda5f_tutorials wiggle.launch' 'C-m'
byobu attach -t wiggle