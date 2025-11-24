# PyCuVSLAM ROS2 Integration

This project integrates NVIDIA's PyCuVSLAM library with ROS2, providing Visual SLAM capabilities for robotics applications. It supports both x86_64 (Intel with RTX3070) and aarch64 (Jetson Orin Nano 8GB) platforms.

## Overview

PyCuVSLAM is NVIDIA's GPU-accelerated Visual SLAM library that provides real-time visual odometry and mapping. This project wraps it in a ROS2 node for easy integration with robotic systems.

## Platform Support

### x86_64 (Intel with RTX3070)
- **OS**: Ubuntu 22.04
- **CUDA**: 12.6
- **Python**: 3.10
- **GPU**: NVIDIA RTX3070
- **Recommended Resolution**: Up to 1920x1080
- **Target FPS**: 60

### aarch64 (Jetson Orin Nano 8GB)
- **OS**: Ubuntu 22.04
- **JetPack**: 6.1/6.2
- **CUDA**: 12.6
- **Python**: 3.10
- **GPU**: Jetson Orin Nano
- **Recommended Resolution**: VGA (640x480)
- **Target FPS**: 30

## Project Structure

```
VSLAM/
├── ros2_ws/                    # ROS2 workspace
│   └── src/
│       └── pycuvslam_ros2/    # ROS2 package
│           ├── config/        # Configuration files
│           ├── launch/        # Launch files
│           └── pycuvslam_ros2/  # Python package
├── config/                    # Platform-specific configs
│   ├── platform_x86_64.yaml
│   └── platform_aarch64.yaml
├── scripts/                   # Installation scripts
│   ├── install_x86_64.sh
│   └── install_aarch64.sh
├── docker/                    # Docker configurations
│   ├── Dockerfile.x86_64
│   ├── Dockerfile.aarch64
│   ├── docker-compose.x86_64.yaml
│   └── docker-compose.aarch64.yaml
└── pycuvslam_clone/          # PyCuVSLAM source (created during install)
```

## Prerequisites

### Common Requirements
- NVIDIA GPU with CUDA support
- CUDA Toolkit 12.6
- Python 3.10
- Git with Git LFS
- ROS2 Humble

### Platform-Specific

**x86_64:**
- Ubuntu 22.04
- NVIDIA drivers (latest recommended)

**aarch64 (Jetson):**
- JetPack 6.1 or 6.2
- Jetson set to MAXN power mode

## Installation

### Option 1: Native Installation

#### For x86_64 (Intel with RTX3070):
```bash
cd /home/dingo/Programming/VSLAM
./scripts/install_x86_64.sh
```

#### For aarch64 (Jetson Orin Nano):
```bash
cd /home/dingo/Programming/VSLAM
./scripts/install_aarch64.sh
```

The installation script will:
1. Check system requirements
2. Install dependencies
3. Clone PyCuVSLAM repository
4. Create a virtual environment
5. Install PyCuVSLAM for your platform
6. Install ROS2 dependencies

### Option 2: Docker Installation

#### For x86_64:
```bash
cd docker
docker-compose -f docker-compose.x86_64.yaml build
docker-compose -f docker-compose.x86_64.yaml up -d
docker-compose -f docker-compose.x86_64.yaml exec pycuvslam_ros2 bash
```

#### For aarch64 (Jetson):
```bash
cd docker
docker-compose -f docker-compose.aarch64.yaml build
docker-compose -f docker-compose.aarch64.yaml up -d
docker-compose -f docker-compose.aarch64.yaml exec pycuvslam_ros2 bash
```

## Building the ROS2 Workspace

After installation, build the ROS2 workspace:

```bash
# Activate virtual environment (if using native installation)
source ~/pycuvslam_venv/bin/activate

# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace
cd ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Usage

### Basic Launch

```bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source ~/Programming/VSLAM/ros2_ws/install/setup.bash

# Launch VSLAM node
ros2 launch pycuvslam_ros2 vslam.launch.py
```

### Custom Parameters

```bash
ros2 launch pycuvslam_ros2 vslam.launch.py \
    camera_topic:=/my_camera/image_raw \
    camera_info_topic:=/my_camera/camera_info \
    mode:=mono \
    publish_tf:=true
```

### Available Parameters

- `camera_topic`: Camera image topic (default: `/camera/image_raw`)
- `camera_info_topic`: Camera info topic (default: `/camera/camera_info`)
- `mode`: VSLAM mode - `mono`, `stereo`, or `rgbd` (default: `mono`)
- `publish_tf`: Publish TF transforms (default: `true`)
- `world_frame`: World/map frame ID (default: `world`)
- `camera_frame`: Camera frame ID (default: `camera`)

## ROS2 Topics

### Subscribed Topics
- `${camera_topic}` (sensor_msgs/Image): Camera images
- `${camera_info_topic}` (sensor_msgs/CameraInfo): Camera calibration

### Published Topics
- `~/odometry` (nav_msgs/Odometry): Visual odometry estimates
- `~/path` (nav_msgs/Path): Trajectory path for visualization

### Published TF Transforms
- `world` → `camera`: Camera pose in world frame

## Configuration

### Platform-Specific Configuration

Platform configurations are stored in `config/platform_*.yaml`. These define:
- Performance parameters
- Resolution limits
- GPU settings
- Memory allocation

### ROS2 Node Configuration

Edit `ros2_ws/src/pycuvslam_ros2/config/default_params.yaml` to customize:
- Topic names
- Frame IDs
- Performance settings
- Feature flags

## Performance Tuning

### For x86_64 (RTX3070):
- Can handle higher resolutions (up to 1080p)
- Visualization can be enabled
- Higher keyframe limits

### For Jetson Orin Nano:
- Use VGA resolution (640x480) for best performance
- Disable visualization (`enable_visualization: false`)
- Ensure camera runs at 30+ FPS
- Set power mode to MAXN: `sudo nvpmodel -m 0 && sudo jetson_clocks`
- Monitor GPU/CPU usage: `jtop`

## Troubleshooting

### Camera Calibration
Accurate camera calibration is critical. Use ROS2 camera calibration:
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 \
    image:=/camera/image_raw camera:=/camera
```

### Performance Issues
- Reduce image resolution
- Disable visualization
- Lower frame rate
- Check for thermal throttling (especially on Jetson)
- Ensure no other intensive processes are running

### CUDA Errors
- Verify CUDA installation: `nvcc --version`
- Check GPU status: `nvidia-smi`
- Ensure CUDA 12.6 is installed
- Restart system after first CUDA installation

### Import Errors
- Ensure virtual environment is activated
- Verify PyCuVSLAM installation: `pip show cuvslam`
- Check Python version: `python --version` (should be 3.10)

## Development

### Modifying the ROS2 Node

The main VSLAM node is located at:
```
ros2_ws/src/pycuvslam_ros2/pycuvslam_ros2/vslam_node.py
```

After making changes:
```bash
cd ros2_ws
colcon build --symlink-install --packages-select pycuvslam_ros2
```

### Integrating PyCuVSLAM API

The current implementation includes a skeleton integration. To complete it:

1. Review PyCuVSLAM documentation and examples
2. Update `initialize_slam()` method with proper tracker initialization
3. Update `image_callback()` to process images with cuVSLAM
4. Extract and publish pose data from tracker output

Example integration points in `vslam_node.py`:
- Line 88-106: `initialize_slam()` - Initialize tracker
- Line 108-124: `image_callback()` - Process images and track
- Line 126-157: `publish_pose()` - Publish results

## License

This project wraps PyCuVSLAM, which is under NVIDIA's non-commercial license. Please review the license terms at: https://github.com/NVlabs/PyCuVSLAM

## Resources

- [PyCuVSLAM GitHub](https://github.com/NVlabs/PyCuVSLAM)
- [NVIDIA cuVSLAM Documentation](https://docs.nvidia.com/isaac/packages/visual_slam/doc/cuvslam_node.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## Support

For issues related to:
- PyCuVSLAM library: [NVlabs/PyCuVSLAM Issues](https://github.com/NVlabs/PyCuVSLAM/issues)
- ROS2 integration: Create an issue in this repository
- Platform-specific problems: Check platform documentation (Jetson, CUDA, etc.)
