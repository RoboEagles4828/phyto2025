#!/bin/bash
ORANGE='\033[0;33m'
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

# Change permissions on any joystick devices
sudo chmod a+rw /dev/input/js*
source /opt/ros/jazzy/setup.bash

# echo -e "\n----JOYSTICKS CONNECTED----"
# ros2 run joy joy_enumerate_devices

# echo -e "\n----COMMANDS----"
# echo "launch          : Launch sim teleop code"
# echo "launch isaac    : Launch sim teleop code"
# echo "launch real     : Launch real teleop code"
# echo "launch test_hw  : Launch rviz with teleop code"
# echo "restart-ros2    : restart ros2 daemon"

echo -e "\n${ORANGE}===THIS IS IN DEVELOPMENT. ISSUES MAY ARISE===${NC}"

echo -e "\n----Build Command----"
echo -e "ctrl + shift + b  : Build"

echo -e "\n----ROS DOMAIN ID: ${GREEN}$ROS_DOMAIN_ID${NC}"
echo -e "----ROS NAMESPACE: ${GREEN}$ROS_NAMESPACE${NC}"