# PyCuVSLAM ROS2 - Detailed Setup Guide

This guide provides step-by-step instructions for setting up PyCuVSLAM with ROS2 on both platforms.

## Table of Contents

1. [Prerequisites Installation](#prerequisites-installation)
2. [Platform-Specific Setup](#platform-specific-setup)
3. [ROS2 Installation](#ros2-installation)
4. [PyCuVSLAM Installation](#pycuvslam-installation)
5. [Testing the Installation](#testing-the-installation)
6. [Common Issues](#common-issues)

## Prerequisites Installation

### CUDA Toolkit 12.6

#### x86_64:
```bash
# Download CUDA 12.6
wget https://developer.download.nvidia.com/compute/cuda/12.6.0/local_installers/cuda_12.6.0_560.28.03_linux.run

# Install CUDA
sudo sh cuda_12.6.0_560.28.03_linux.run

# Add to PATH (add to ~/.bashrc)
export PATH=/usr/local/cuda-12.6/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH

# Verify installation
nvcc --version
nvidia-smi
```

#### aarch64 (Jetson):
CUDA comes with JetPack. Install JetPack 6.1 or 6.2:
```bash
# Use NVIDIA SDK Manager on a host Ubuntu machine
# Or use the command line method:
sudo apt-get update
sudo apt-get install nvidia-jetpack
```

### Python 3.10

Both platforms should have Python 3.10:
```bash
# Verify Python version
python3 --version

# If not 3.10, install it:
sudo apt-get update
sudo apt-get install python3.10 python3.10-venv python3-pip
```

### Git LFS

Required for cloning PyCuVSLAM:
```bash
sudo apt-get update
sudo apt-get install git-lfs
git lfs install
```

## Platform-Specific Setup

### x86_64 (Intel with RTX3070)

1. **Install NVIDIA Drivers**:
```bash
# Recommended: Use Ubuntu's additional drivers tool
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall

# Or install specific driver version
sudo apt-get install nvidia-driver-560

# Reboot
sudo reboot
```

2. **Verify GPU**:
```bash
nvidia-smi
# Should show RTX3070
```

### aarch64 (Jetson Orin Nano)

1. **Flash JetPack**:
   - Use NVIDIA SDK Manager from a host Ubuntu machine
   - Select JetPack 6.1 or 6.2
   - Flash to Jetson Orin Nano

2. **Set Power Mode**:
```bash
# Set to MAXN (maximum performance)
sudo nvpmodel -m 0
sudo jetson_clocks

# Verify
sudo nvpmodel -q
```

3. **Install jtop for monitoring**:
```bash
sudo apt-get update
sudo apt-get install python3-pip
sudo pip3 install jetson-stats
# Reboot
sudo reboot
# Run jtop
jtop
```

## ROS2 Installation

### Install ROS2 Humble

Both platforms use the same steps:

```bash
# Set locale
sudo apt-get update
sudo apt-get install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt-get install software-properties-common
sudo add-apt-repository universe

sudo apt-get update
sudo apt-get install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
sudo apt-get update

# For x86_64 (desktop):
sudo apt-get install ros-humble-desktop

# For Jetson (base - lighter):
sudo apt-get install ros-humble-ros-base

# Install development tools
sudo apt-get install ros-dev-tools
sudo apt-get install ros-humble-cv-bridge
sudo apt-get install python3-colcon-common-extensions

# Initialize rosdep
sudo rosdep init
rosdep update

# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Verify ROS2 Installation

```bash
# Check ROS2 installation
ros2 --help

# Test with demo nodes
# Terminal 1:
ros2 run demo_nodes_cpp talker

# Terminal 2:
ros2 run demo_nodes_py listener
```

## PyCuVSLAM Installation

### Automated Installation

Navigate to your project directory:
```bash
cd /home/dingo/Programming/VSLAM
```

#### For x86_64:
```bash
./scripts/install_x86_64.sh
```

#### For aarch64:
```bash
./scripts/install_aarch64.sh
```

### Manual Installation (Alternative)

If you prefer to install manually:

```bash
# Install git-lfs
sudo apt-get install git-lfs
git lfs install

# Create virtual environment
python3 -m venv ~/pycuvslam_venv
source ~/pycuvslam_venv/bin/activate

# Clone PyCuVSLAM
cd /home/dingo/Programming/VSLAM
git clone https://github.com/NVlabs/PyCuVSLAM.git pycuvslam_clone
cd pycuvslam_clone

# Install based on platform
# For x86_64:
pip install -e bin/x86_64

# For aarch64:
pip install -e bin/aarch64

# Install dependencies
pip install -r examples/requirements.txt
pip install opencv-python numpy
```

### Build ROS2 Workspace

```bash
# Activate virtual environment
source ~/pycuvslam_venv/bin/activate

# Source ROS2
source /opt/ros/humble/setup.bash

# Build
cd /home/dingo/Programming/VSLAM/ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to ~/.bashrc for convenience
echo "source ~/pycuvslam_venv/bin/activate" >> ~/.bashrc
echo "source ~/Programming/VSLAM/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Testing the Installation

### Test PyCuVSLAM

```bash
# Activate virtual environment
source ~/pycuvslam_venv/bin/activate

# Test import
python3 -c "import cuvslam; print('PyCuVSLAM imported successfully')"

# Run an example (if available)
cd /home/dingo/Programming/VSLAM/pycuvslam_clone/examples
# Follow examples in the repository
```

### Test ROS2 Node

```bash
# Source everything
source ~/pycuvslam_venv/bin/activate
source /opt/ros/humble/setup.bash
source ~/Programming/VSLAM/ros2_ws/install/setup.bash

# Check if package is found
ros2 pkg list | grep pycuvslam

# Check available nodes
ros2 pkg executables pycuvslam_ros2

# Run the node (will wait for camera topics)
ros2 run pycuvslam_ros2 vslam_node

# In another terminal, check topics
ros2 topic list
ros2 topic echo /pycuvslam_node/odometry
```

### Test with Camera

If you have a USB camera:

```bash
# Install usb_cam package
sudo apt-get install ros-humble-usb-cam

# Terminal 1: Start camera
ros2 run usb_cam usb_cam_node_exe --ros-args \
    -r /image_raw:=/camera/image_raw \
    -r /camera_info:=/camera/camera_info

# Terminal 2: Start VSLAM
ros2 launch pycuvslam_ros2 vslam.launch.py

# Terminal 3: Visualize in RViz
ros2 run rviz2 rviz2
```

## Common Issues

### Issue: "CUDA not found"

**Solution**:
```bash
# Check CUDA installation
ls /usr/local/cuda-12.6

# Add to environment
export CUDA_HOME=/usr/local/cuda-12.6
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
export PATH=$CUDA_HOME/bin:$PATH

# Add to ~/.bashrc to make permanent
```

### Issue: "ModuleNotFoundError: No module named 'cuvslam'"

**Solution**:
```bash
# Make sure virtual environment is activated
source ~/pycuvslam_venv/bin/activate

# Reinstall PyCuVSLAM
cd /home/dingo/Programming/VSLAM/pycuvslam_clone

# For x86_64:
pip install -e bin/x86_64

# For aarch64:
pip install -e bin/aarch64
```

### Issue: "Package 'pycuvslam_ros2' not found"

**Solution**:
```bash
# Rebuild workspace
cd ~/Programming/VSLAM/ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Issue: Poor performance on Jetson

**Solution**:
```bash
# Set maximum performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor with jtop
jtop

# Reduce image resolution in camera driver
# Disable visualization in config file
# Lower frame rate to 30 FPS
```

### Issue: Git LFS files not downloaded

**Solution**:
```bash
# Initialize git-lfs
git lfs install

# Fetch LFS files
cd /home/dingo/Programming/VSLAM/pycuvslam_clone
git lfs pull
```

## Next Steps

After successful installation:

1. Calibrate your camera using ROS2 camera calibration tools
2. Configure platform-specific settings in `config/platform_*.yaml`
3. Test with your camera setup
4. Review and complete the PyCuVSLAM API integration in `vslam_node.py`
5. Tune performance parameters for your use case

For more information, refer to the main [README.md](README.md).
