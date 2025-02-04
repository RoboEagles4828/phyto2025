#!/bin/bash
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m'
SCRIPT_PATH=$(dirname "$0")
devenv_path="$SCRIPT_PATH/../.devcontainer/.env"

echo -e "${ORANGE}INSTALLING APT PACKAGES${NC}"
sudo apt-get update
sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common \
    git-lfs -y

# Stop ubuntu pop ups that applications are not responding
gsettings set org.gnome.mutter check-alive-timeout 60000

# Setup a unique domain 
# Avoids conflicts with others on the same network
if [ -z "$(cat $devenv_path | grep ROS_NAMESPACE)" ]; then
  echo -e "${ORANGE}SETTING ROS_NAMESPACE${NC}"
  read -p "Enter a name for your ROS_NAMESPACE: " ros_namespace
  echo "ROS_NAMESPACE=${ros_namespace}" >> $devenv_path
else
  echo -e "${GREEN}ROS_NAMESPACE ALREADY SET${NC}"
fi

# Docker
if [[ -z "$(which docker)" ]]; then
  echo -e "${ORANGE}INSTALLING DOCKER${NC}"
  curl https://get.docker.com | sh \
  && sudo systemctl --now enable docker
  sudo usermod -aG docker $USER
else
  echo -e "${GREEN}DOCKER ALREADY INSTALLED${NC}"
fi

# Deployment Keys
if [ ! -f ~/.ssh/robot_deploy ]; then
  echo -e "${ORANGE}SETTING UP DEPLOY KEYS${NC}"
  ssh-keygen -b 2048 -t rsa -f ~/.ssh/robot_deploy -q -N ""
else
  echo -e "${GREEN}DEPLOY KEYS ALREADY SET${NC}"
fi