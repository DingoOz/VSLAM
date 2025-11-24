# Quick Start Guide

Get PyCuVSLAM running with ROS2 in minutes!

## Prerequisites Check

Run these commands to verify your system:

```bash
# Check platform
uname -m
# Should show: x86_64 or aarch64

# Check Python
python3 --version
# Should show: Python 3.10.x

# Check NVIDIA GPU
nvidia-smi
# Should show your GPU

# Check CUDA (optional, will be installed)
nvcc --version
# Should show CUDA 12.6
```

## Quick Install

### Step 1: Install Prerequisites

```bash
# Install git-lfs (required)
sudo apt-get update
sudo apt-get install git-lfs
git lfs install
```

### Step 2: Run Installation Script

Navigate to the project directory:
```bash
cd /home/dingo/Programming/VSLAM
```

**For x86_64 (Intel with RTX3070):**
```bash
./scripts/install_x86_64.sh
```

**For Jetson Orin Nano:**
```bash
./scripts/install_aarch64.sh
```

The script will guide you through the installation process.

### Step 3: Build ROS2 Workspace

```bash
# Activate virtual environment
source ~/pycuvslam_venv/bin/activate

# Source ROS2
source /opt/ros/humble/setup.bash

# Build
cd ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Step 4: Launch!

```bash
# Make sure everything is sourced
source ~/pycuvslam_venv/bin/activate
source /opt/ros/humble/setup.bash
source ~/Programming/VSLAM/ros2_ws/install/setup.bash

# Launch VSLAM node
ros2 launch pycuvslam_ros2 vslam.launch.py
```

## Test with USB Camera

If you have a USB camera connected:

```bash
# Install USB camera driver
sudo apt-get install ros-humble-usb-cam

# Terminal 1: Start camera
ros2 run usb_cam usb_cam_node_exe

# Terminal 2: Start VSLAM
ros2 launch pycuvslam_ros2 vslam.launch.py \
    camera_topic:=/image_raw \
    camera_info_topic:=/camera_info

# Terminal 3: Visualize
ros2 run rviz2 rviz2
```

In RViz:
1. Set Fixed Frame to `world`
2. Add → By topic → `/pycuvslam_node/path` → Path
3. Add → By topic → `/pycuvslam_node/odometry` → Odometry

## Using Docker (Alternative)

If you prefer Docker:

**For x86_64:**
```bash
cd docker
docker-compose -f docker-compose.x86_64.yaml up -d
docker-compose -f docker-compose.x86_64.yaml exec pycuvslam_ros2 bash
```

**For Jetson:**
```bash
cd docker
docker-compose -f docker-compose.aarch64.yaml up -d
docker-compose -f docker-compose.aarch64.yaml exec pycuvslam_ros2 bash
```

## Common Commands

```bash
# Check if package is installed
ros2 pkg list | grep pycuvslam

# View node info
ros2 node info /pycuvslam_node

# List topics
ros2 topic list

# Echo odometry
ros2 topic echo /pycuvslam_node/odometry

# View parameters
ros2 param list /pycuvslam_node
```

## Next Steps

1. **Camera Calibration**: Calibrate your camera for best results
   ```bash
   ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108
   ```

2. **Tune Performance**: Edit `config/platform_*.yaml` for your needs

3. **Integration**: Complete the PyCuVSLAM API integration in:
   ```
   ros2_ws/src/pycuvslam_ros2/pycuvslam_ros2/vslam_node.py
   ```

## Troubleshooting

**Node won't start:**
- Check that all dependencies are sourced
- Verify camera topics exist: `ros2 topic list`
- Check logs: `ros2 run pycuvslam_ros2 vslam_node --ros-args --log-level debug`

**Poor performance on Jetson:**
```bash
# Set max performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor
jtop
```

**Can't import cuvslam:**
```bash
# Activate venv
source ~/pycuvslam_venv/bin/activate

# Verify installation
pip show cuvslam
```

## Documentation

- Full documentation: [README.md](README.md)
- Detailed setup: [SETUP_GUIDE.md](SETUP_GUIDE.md)
- PyCuVSLAM docs: https://github.com/NVlabs/PyCuVSLAM

## Help

Need help?
1. Check the troubleshooting section in README.md
2. Review PyCuVSLAM examples in `pycuvslam_clone/examples/`
3. Open an issue with details about your platform and error messages
