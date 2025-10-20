# av_sim_core

Minimal ROS 2 + CARLA autonomous vehicle simulation package.

## Nodes
- spawn_actors: Spawns traffic (vehicles + pedestrians) via CARLA Python API.
- sensor_fusion: Subscribes to lidar + radar topics and publishes fused PointCloud2.
- vision_detector: YOLOv8 stop-sign detector on front RGB camera; publishes /carla/hero/stop_flag.
- stop_controller: Simple longitudinal control that stops the vehicle if stop flag received.
- pure_pursuit: Lateral control using waypoints (Ackermann steering).

## Launch
`ros2 launch av_sim_core spawn_and_perception.launch.py`
(Ensure CARLA server and ROS bridge are running.)

## Topics
Input:
- /carla/hero/lidar (PointCloud2)
- /carla/hero/radar_front (PointCloud2)
- /carla/hero/rgb_front/image (Image)
- /carla/hero/odometry (Odometry)
- /carla/hero/speedometer (Float32)

Output:
- /carla/hero/fused_objects (PointCloud2)
- /carla/hero/stop_flag (Bool)
- /carla/hero/ackermann_cmd (AckermannDrive)

## Dependencies
See package.xml. Install YOLO model weights (e.g. place `yolov8n.pt` in working directory).

## Waypoints
Put a reduced set of waypoints in `data/waypoints.csv` with header `time,x,y`. Adapt Pure Pursuit LOOKAHEAD as needed.
