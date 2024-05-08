#!/bin/bash

# Replace 'username' and 'hostname' with your SSH credentials
SSH_USER="grupo4"
SSH_HOST="10.128.0.9"

# Open a new xterm window and run the SSH command to launch the ROS 2 node
xterm -e "ssh $SSH_USER@$SSH_HOST 'source /opt/ros/foxy/setup.bash && ros2 launch turtlebot3_bringup robot.launch.py'" &
