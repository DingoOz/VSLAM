#!/bin/bash
set -e

echo "=========================================="
echo "PyCuVSLAM Installation Script for aarch64"
echo "Platform: Jetson Orin Nano 8GB"
echo "=========================================="

# Detect platform
PLATFORM=$(uname -m)
if [ "$PLATFORM" != "aarch64" ]; then
    echo "ERROR: This script is for aarch64 platform, detected: $PLATFORM"
    exit 1
fi

# Check for Jetson
if [ ! -f "/etc/nv_tegra_release" ]; then
    echo "ERROR: This doesn't appear to be a Jetson device"
    exit 1
fi

echo "Jetson platform detected:"
cat /etc/nv_tegra_release

# Check for NVIDIA GPU
if ! command -v nvidia-smi &> /dev/null; then
    echo "ERROR: nvidia-smi not found. Please install JetPack first."
    exit 1
fi

# Check CUDA version
if [ ! -d "/usr/local/cuda-12.6" ]; then
    echo "WARNING: CUDA 12.6 not found at /usr/local/cuda-12.6"
    echo "Please ensure JetPack 6.1 or 6.2 is installed."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check Python version
PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
if [ "$PYTHON_VERSION" != "3.10" ]; then
    echo "WARNING: Python 3.10 is required, found: $PYTHON_VERSION"
    echo "Please install Python 3.10"
    exit 1
fi

echo "Python version OK: $PYTHON_VERSION"

# Set Jetson to maximum performance mode
echo "Setting Jetson to MAXN power mode..."
sudo nvpmodel -m 0
sudo jetson_clocks

# Install system dependencies
echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y \
    git-lfs \
    python3-pip \
    python3.10-venv \
    build-essential \
    cmake

# Initialize git-lfs
git lfs install

# Create virtual environment
VENV_PATH="$HOME/pycuvslam_venv"
if [ ! -d "$VENV_PATH" ]; then
    echo "Creating virtual environment at $VENV_PATH"
    python3 -m venv $VENV_PATH
else
    echo "Virtual environment already exists at $VENV_PATH"
fi

# Activate virtual environment
source $VENV_PATH/bin/activate

# Upgrade pip
pip install --upgrade pip

# Clone PyCuVSLAM if not already cloned
PYCUVSLAM_DIR="$(pwd)/pycuvslam_clone"
if [ ! -d "$PYCUVSLAM_DIR" ]; then
    echo "Cloning PyCuVSLAM repository..."
    git clone https://github.com/NVlabs/PyCuVSLAM.git $PYCUVSLAM_DIR
    cd $PYCUVSLAM_DIR
else
    echo "PyCuVSLAM repository already cloned at $PYCUVSLAM_DIR"
    cd $PYCUVSLAM_DIR
    git pull
fi

# Install PyCuVSLAM for aarch64
echo "Installing PyCuVSLAM for aarch64..."
pip install -e bin/aarch64

# Install example dependencies
if [ -f "examples/requirements.txt" ]; then
    echo "Installing example dependencies..."
    pip install -r examples/requirements.txt
fi

# Install ROS2 Python dependencies
echo "Installing ROS2 Python dependencies..."
pip install opencv-python numpy

echo ""
echo "=========================================="
echo "Installation completed successfully!"
echo "=========================================="
echo ""
echo "To activate the virtual environment, run:"
echo "  source $VENV_PATH/bin/activate"
echo ""
echo "Next steps:"
echo "1. Build ROS2 workspace:"
echo "   cd ros2_ws"
echo "   colcon build --symlink-install"
echo ""
echo "2. Source ROS2 workspace:"
echo "   source ros2_ws/install/setup.bash"
echo ""
echo "3. Launch VSLAM node:"
echo "   ros2 launch pycuvslam_ros2 vslam.launch.py"
echo ""
echo "Note: For best performance on Jetson:"
echo "- Disable visualization (enable_visualization: false)"
echo "- Use VGA resolution or lower"
echo "- Ensure camera runs at 30+ FPS"
echo ""
