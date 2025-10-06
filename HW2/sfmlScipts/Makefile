rwildcard=$(wildcard $1$2) $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2))
src := $(call rwildcard,./,*.cpp)
obj = $(patsubst %.cpp,%.o,$(src))

# SFML static library linking flags
# Note: Static libraries require specific linking order and system dependencies
LDFLAGS = -lsfml-graphics-s -lsfml-window-s -lsfml-system-s -lGL -lX11 -lXrandr -lXcursor -lXi -lfreetype -lpthread -ldl -ludev

# Include paths for different platforms
INTELMAC_INCLUDE=-I/usr/local/include						# Intel mac
APPLESILICON_INCLUDE=-I/opt/homebrew/include					# Apple Silicon
UBUNTU_INCLUDE=-I/usr/local/include 						# Ubuntu with source-compiled SFML
UBUNTU_APPLESILICON_INCLUDE=-I/usr/include 					# Apple Silicon Ubuntu VM
UBUNTU_INTEL_INCLUDE=-I/usr/include 						# Intel Ubuntu VM

# Library paths for different platforms  
INTELMAC_LIB=-L/usr/local/lib							# Intel mac
APPLESILICON_LIB=-L/opt/homebrew/lib						# Apple Silicon
# Try multiple possible locations for Ubuntu SFML libraries
UBUNTU_LIB=-L/usr/local/lib -L/usr/lib -L/usr/lib/x86_64-linux-gnu		# Ubuntu with source-compiled SFML
UBUNTU_APPLESILICON_LIB=-L/usr/lib/aarch64-linux-gnu			# Apple Silicon Ubuntu VM
UBUNTU_INTEL_LIB=-L/usr/lib/x86_64-linux-gnu					# Intel Ubuntu VM

# Platform-specific settings
MACOS_INCLUDE=$(APPLESILICON_INCLUDE)
MACOS_LIB=$(APPLESILICON_LIB)

# Compilers
MACOS_COMPILER=/usr/bin/clang++
UBUNTU_COMPILER=/usr/bin/g++

# C++ standard - SFML 3.0 requires C++17 or higher
CXX_STANDARD = -std=c++17

# Detect OS
uname_s := $(shell uname -s)

all: main

main: $(obj)
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) $(CXX_STANDARD) -o $@ $^ $(MACOS_LIB) $(LDFLAGS)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) $(CXX_STANDARD) -static-libgcc -static-libstdc++ -o $@ $^ $(UBUNTU_LIB) $(LDFLAGS)
endif

%.o: %.cpp
ifeq ($(uname_s),Darwin)
	$(MACOS_COMPILER) $(CXX_STANDARD) -c $^ -o $@ $(MACOS_INCLUDE)
else ifeq ($(uname_s),Linux)
	$(UBUNTU_COMPILER) $(CXX_STANDARD) -c $^ -o $@ $(UBUNTU_INCLUDE)
endif

.PHONY: clean
clean:
	rm -f $(obj) main
