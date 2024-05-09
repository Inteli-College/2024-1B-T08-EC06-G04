#!/bin/bash

SSH_USER="grupo4"
SSH_HOST="10.128.0.9"

sudo apt-get install -y xterm

# Use xterm to open a new window and run the SSH command to launch ros2
xterm -e "ssh $SSH_USER@$SSH_HOST 'source /opt/ros/foxy/setup.bash && ros2 launch turtlebot3_bringup robot.launch.py'" &
