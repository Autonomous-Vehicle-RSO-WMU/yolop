# YOLOP Autonomous Vehicle Simulation

## GPU-Accelerated Docker Simulation

Optimized two-container setup:

1. **`carla`** – CARLA UE4 simulator with GPU acceleration
2. **`ros-sim`** – ROS 2 Humble + YOLOv8 inference + vehicle control nodes

## Architecture Benefits
* **Layered caching**: Heavy dependencies (PyTorch, ROS) cached in base image
* **Fast iteration**: Code changes don't trigger dependency rebuilds  
* **GPU optimized**: CUDA runtime + PyTorch GPU for inference
* **Minimal footprint**: Only required packages, no desktop bloat

## Prerequisites
* NVIDIA GPU with drivers installed
* Docker Engine (not Desktop) with NVIDIA Container Toolkit
* Ubuntu 22.04 or compatible

## Quick Start

### 1. Build Base Image (Heavy Dependencies)
```bash
sudo docker build -f Dockerfile.ros.base -t yolop:ros-base .
```

### 2. Build Simulation Layer  
```bash
sudo docker compose build ros-sim
```

### 3. Run Full Simulation
```bash
sudo docker compose up
```

## Development Workflow

### Rapid Iteration
Source code is mounted read/write - edit Python files and restart containers:

```bash
# Restart single node
sudo docker compose exec ros-sim bash
source /opt/ros/$ROS_DISTRO/setup.bash  
python -m av_sim_core.vision_detector

# Add new node
python -m av_sim_core.my_new_node
```

### Active Nodes
* **spawn_actors** - Populates simulation with vehicles/pedestrians
* **sensor_fusion** - Combines LiDAR + radar data  
* **vision_detector** - YOLOv8 stop sign detection
* **pure_pursuit** - Path following controller
* **stop_controller** - Emergency braking on detection

### Model Management
YOLOv8 weights auto-download to `./models/yolov8n.pt`. To use different model:
```bash
curl -L -o models/yolov8s.pt https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt
```
Edit model path in `vision_detector.py`.

## System Requirements
* 8GB+ RAM (for PyTorch + CARLA)
* NVIDIA GPU with 4GB+ VRAM
* 10GB+ disk space for images

## Cleanup
```bash
sudo docker compose down
sudo docker system prune -f  # Remove unused images/containers
```

