# YOLOP - Docker Setup for Linux

This project provides a complete Docker setup for running the YOLOP (YOLO + Pure Pursuit) autonomous vehicle simulation on Linux with NVIDIA GPU support.

## Prerequisites

1. **Ubuntu 20.04/22.04** (recommended for ROS2 compatibility)
2. **NVIDIA GPU** with updated drivers (NVIDIA Driver 470+)
3. **Docker Desktop for Linux**
4. **NVIDIA Container Toolkit**

## Quick Setup

## Quick Setup

### 1. Install Docker Desktop for Linux
```bash
# Download Docker Desktop for Linux
wget https://desktop.docker.com/linux/main/amd64/docker-desktop-4.25.0-amd64.deb

# Install Docker Desktop
sudo apt update
sudo apt install ./docker-desktop-4.25.0-amd64.deb

# Start Docker Desktop
systemctl --user start docker-desktop

# Enable Docker Desktop to start on boot
systemctl --user enable docker-desktop

# Add your user to docker group (logout/login required)
sudo usermod -aG docker $USER
```

### 2. Install NVIDIA Container Toolkit
```bash
# Add NVIDIA package repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
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
```

### 3. Clone and Build
```bash
# Clone your repository
git clone <your-repo-url>
cd yolop

# Run the Linux setup script
chmod +x setup-linux.sh
./setup-linux.sh
```

## Running the Simulation

### Option 1: Docker Compose (Recommended)
```bash
# Ensure Docker Desktop is running
docker info

# Start the simulation
docker-compose up
```

This will automatically:
- Start the CARLA server with GPU acceleration
- Build and run the YOLOP simulation container
- Launch all ROS2 nodes

### Option 2: Using Docker Desktop GUI
1. Open Docker Desktop application
2. Go to the "Images" tab and build the YOLOP image
3. Use the "Containers" tab to manage running containers
4. Monitor resource usage in the "Stats" section

### Option 3: Manual Docker Commands

1. **Start CARLA Server:**
```bash
docker run --rm --gpus all -p 2000-2002:2000-2002 \
  --env DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  carlasim/carla:0.9.14 /bin/bash CarlaUE4.sh \
  -world-port=2000 -resx=800 -resy=600 -quality-level=Low
```

2. **Start YOLOP Simulation:**
```bash
docker run --rm --gpus all --network host \
  --env DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -it yolop:latest
```

3. **Inside the container, launch ROS2 nodes:**
```bash
ros2 launch av_sim_core spawn_and_perception.launch.py
```

## Architecture

The Docker setup includes:

- **CARLA Server**: Autonomous driving simulator with GPU acceleration
- **ROS2 Humble**: Robot Operating System 2
- **YOLO**: Object detection for stop signs
- **Pure Pursuit Controller**: Path following algorithm
- **Sensor Fusion**: LiDAR and radar data fusion

## Available ROS2 Nodes

- `spawn_actors`: Spawns traffic vehicles and pedestrians
- `sensor_fusion`: Fuses LiDAR and radar data
- `vision_detector`: YOLO-based stop sign detection
- `stop_controller`: Emergency stopping when stop signs detected
- `pure_pursuit`: Lateral control using waypoints

## Key Topics

**Input:**
- `/carla/hero/lidar` (PointCloud2)
- `/carla/hero/radar_front` (PointCloud2)
- `/carla/hero/rgb_front/image` (Image)
- `/carla/hero/odometry` (Odometry)
- `/carla/hero/speedometer` (Float32)

**Output:**
- `/carla/hero/fused_objects` (PointCloud2)
- `/carla/hero/stop_flag` (Bool)
- `/carla/hero/ackermann_cmd` (AckermannDrive)

## Troubleshooting

### Docker Desktop Not Running
Ensure Docker Desktop is running:
```bash
# Check if Docker Desktop is running
docker info

# Start Docker Desktop if not running
systemctl --user start docker-desktop

# Or launch from applications menu
```

### GPU Not Detected
Ensure NVIDIA Container Toolkit is properly installed:
```bash
docker run --rm --gpus all nvidia/cuda:11.8-base nvidia-smi
```

### X11 Display Issues
If you get display-related errors, ensure X11 forwarding is set up:
```bash
# Allow X11 connections
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY  # Should be something like :0 or :1
```

### CARLA Connection Issues
Check if CARLA server is running and accessible:
```bash
docker ps  # Should show CARLA container
netstat -ln | grep 2000  # Check if port 2000 is listening
```

### ROS2 Node Communication
Verify ROS2 domain ID is consistent:
```bash
echo $ROS_DOMAIN_ID  # Should be 0
ros2 node list  # Should show your nodes
```

### Permission Issues
If you get permission denied errors:
```bash
# Ensure user is in docker group
sudo usermod -aG docker $USER
# Logout and login again

# Check docker group membership
groups $USER | grep docker
```

## Customization

### Waypoints
Edit `av_sim_core/data/waypoints.csv` to customize the vehicle path.

### YOLO Model
The default model is YOLOv8n. To use a different model, modify `vision_detector.py`.

### CARLA Settings
Adjust CARLA server parameters in `docker-compose.yml` for different resolutions or quality levels.

## Development

To modify the code and rebuild:

```bash
# Make changes to your code
# Rebuild the Docker image
docker build -t yolop:latest .

# Restart the simulation
docker-compose down
docker-compose up
```

## Performance Tips

1. **Lower CARLA quality** for better performance: `-quality-level=Low`
2. **Reduce resolution**: `-resx=640 -resy=480`
3. **Limit spawned actors** in `spawn_actors.py`
4. **Monitor GPU usage**: `nvidia-smi`
5. **Use headless mode** for servers: Add `-RenderOffScreen` to CARLA command
6. **Optimize Docker**: Use `--shm-size=2g` for larger shared memory if needed

## Additional Notes

### Docker Desktop Benefits
- **GUI Management**: Visual interface for managing containers, images, and volumes
- **Resource Monitoring**: Built-in monitoring for CPU, memory, and GPU usage
- **Easy Updates**: Automatic updates and simplified configuration
- **Dev Environments**: Integration with VS Code and other development tools
- **Extensions**: Access to Docker Desktop extensions for enhanced functionality

### ROS2 Compatibility
This setup uses ROS2 Humble, which is the recommended LTS version for Ubuntu 22.04. For Ubuntu 20.04, you may want to use ROS2 Foxy instead.

### CARLA Versions
The setup uses CARLA 0.9.14. Other versions may work but may require adjustments to the Python API calls.

### Network Configuration
The containers use `host` networking for simplicity. For production deployments, consider using custom Docker networks with proper port mapping.
