#!/bin/bash

# Linux Docker Desktop Setup Script for YOLOP
# Run this script on Ubuntu 20.04/22.04

set -e

echo "Setting up YOLOP Docker Desktop environment for Linux with NVIDIA GPU..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Check if running on supported Ubuntu version
if ! grep -q "Ubuntu" /etc/os-release; then
    echo -e "${RED}This script is designed for Ubuntu. Other distributions may work but are not tested.${NC}"
fi

# Check if Docker Desktop is installed
if ! command -v docker &> /dev/null; then
    echo -e "${YELLOW}Docker Desktop is not installed. Installing Docker Desktop for Linux...${NC}"
    
    # Download and install Docker Desktop
    wget -O docker-desktop.deb https://desktop.docker.com/linux/main/amd64/docker-desktop-4.25.0-amd64.deb
    sudo apt update
    sudo apt install -y ./docker-desktop.deb
    rm docker-desktop.deb
    
    # Start Docker Desktop
    systemctl --user start docker-desktop
    systemctl --user enable docker-desktop
    
    # Add user to docker group
    sudo usermod -aG docker $USER
    
    echo -e "${GREEN}Docker Desktop installed successfully!${NC}"
    echo -e "${YELLOW}Please logout and login again for group changes to take effect.${NC}"
    echo -e "${YELLOW}You may also need to start Docker Desktop from the applications menu.${NC}"
fi

# Check if user is in docker group
if ! groups $USER | grep -q docker; then
    echo -e "${YELLOW}Adding user to docker group...${NC}"
    sudo usermod -aG docker $USER
    echo -e "${YELLOW}Please logout and login again for group changes to take effect.${NC}"
fi

# Check if Docker Desktop is running
if ! docker info &> /dev/null; then
    echo -e "${YELLOW}Docker Desktop is not running. Please start Docker Desktop and run this script again.${NC}"
    echo -e "${CYAN}You can start it from the applications menu or run: systemctl --user start docker-desktop${NC}"
    exit 1
fi

# Check if NVIDIA drivers are installed
if ! command -v nvidia-smi &> /dev/null; then
    echo -e "${RED}NVIDIA drivers not found. Please install NVIDIA drivers first:${NC}"
    echo -e "${CYAN}sudo apt update && sudo apt install nvidia-driver-470${NC}"
    exit 1
fi

# Check NVIDIA driver version
NVIDIA_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader,nounits | head -1)
echo -e "${GREEN}NVIDIA Driver version: $NVIDIA_VERSION${NC}"

# Check if NVIDIA Container Toolkit is installed
if ! docker info 2>/dev/null | grep -q nvidia; then
    echo -e "${YELLOW}NVIDIA Container Toolkit not detected. Installing...${NC}"
    
    # Setup NVIDIA package repository
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID) && \
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && \
    curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
    
    # Install NVIDIA Container Toolkit
    sudo apt update
    sudo apt install -y nvidia-container-toolkit
    
    # Configure Docker to use NVIDIA runtime
    sudo nvidia-ctk runtime configure --runtime=docker
    sudo systemctl restart docker
    
    # Restart Docker Desktop
    systemctl --user restart docker-desktop
    sleep 5
    
    echo -e "${GREEN}NVIDIA Container Toolkit installed successfully!${NC}"
fi

# Test GPU access
echo -e "${YELLOW}Testing GPU access in Docker...${NC}"
if docker run --rm --gpus all nvidia/cuda:11.8-base nvidia-smi; then
    echo -e "${GREEN}✓ GPU access confirmed!${NC}"
else
    echo -e "${RED}✗ GPU access failed. Please check NVIDIA Container Toolkit installation.${NC}"
    exit 1
fi

# Install docker-compose if not present
if ! command -v docker-compose &> /dev/null; then
    echo -e "${YELLOW}Installing docker-compose...${NC}"
    sudo apt update
    sudo apt install -y docker-compose
fi

echo -e "${GREEN}Building YOLOP Docker image...${NC}"
docker build -t yolop:latest .

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Docker image built successfully!${NC}"
else
    echo -e "${RED}✗ Failed to build Docker image${NC}"
    exit 1
fi

echo -e "\n${GREEN}Setup complete!${NC}"
echo -e "\n${CYAN}To run the simulation:${NC}"
echo -e "${GREEN}docker-compose up${NC}"
echo -e "\n${CYAN}To run manually:${NC}"
echo -e "${GREEN}1. Start CARLA server:${NC}"
echo -e "   docker run --rm --gpus all -p 2000-2002:2000-2002 --env DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlasim/carla:0.9.14 /bin/bash CarlaUE4.sh -world-port=2000 -resx=800 -resy=600"
echo -e "\n${GREEN}2. In another terminal, start YOLOP simulation:${NC}"
echo -e "   docker run --rm --gpus all --network host --env DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -it yolop:latest"
echo -e "\n${GREEN}3. Inside the container, run:${NC}"
echo -e "   ros2 launch av_sim_core spawn_and_perception.launch.py"
