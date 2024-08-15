#!/bin/bash

byobu new-session -d -s rviz
byobu select-pane -t 0
byobu split-window -v
byobu select-pane -t 0

byobu send-keys -t 0 'xhost + && docker exec -it sda5f_container bash -it -c "roslaunch motoman_sda5f_support test_sda5f_with_onrobot.launch"' 'C-m'

byobu attach -t rviz