#!/bin/bash

# SFML 3.0 Installation Script
# This script automates the installation of SFML 3.0 from source

set -e  # Exit on error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_message() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Check if script is run with sudo privileges for installation
check_sudo() {
    if [ "$EUID" -eq 0 ]; then 
        print_warning "Please do not run this script as root. It will ask for sudo when needed."
        exit 1
    fi
}

print_message "Starting SFML 3.0 installation..."

# Step 1: System Update and Dependency Installation
print_message "Step 1: Updating system packages..."
sudo apt update && sudo apt upgrade -y

print_message "Installing essential build tools..."
sudo apt install -y build-essential cmake git pkg-config gdb

print_message "Installing SFML 3.0 dependencies..."
sudo apt install -y \
    libfreetype-dev \
    libudev-dev \
    libx11-dev \
    libxcursor-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxi-dev \
    libflac-dev \
    libogg-dev \
    libvorbis-dev \
    libopenal-dev \
    libgl1-mesa-dev

# Step 2: Download and Build SFML 3.0 from Source
print_message "Step 2: Creating build directory..."
mkdir -p ~/sfmlbuild
cd ~/sfmlbuild

print_message "Cloning SFML repository..."
if [ -d "SFML" ]; then
    print_warning "SFML directory already exists. Removing old directory..."
    rm -rf SFML
fi

git clone https://github.com/SFML/SFML.git
cd SFML

print_message "Checking out SFML 3.0.0 release..."
git checkout 3.0.0

print_message "Configuring build with CMake..."
mkdir -p build
cd build

cmake .. \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_CXX_STANDARD=17 \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DSFML_BUILD_EXAMPLES=OFF \
    -DSFML_BUILD_DOC=OFF

print_message "Compiling SFML (this may take a few minutes)..."
make -j$(nproc)

print_message "Installing SFML system-wide..."
sudo make install

print_message "Updating dynamic linker cache..."
sudo ldconfig

print_message "${GREEN}âœ“${NC} SFML 3.0 installation completed successfully!"
print_message "You can now use SFML 3.0 in your projects."

# Cleanup option
read -p "Do you want to remove the build directory to save space? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    print_message "Removing build directory..."
    cd ~
    rm -rf ~/sfmlbuild
    print_message "Build directory removed."
fi

print_message "Installation process finished!"