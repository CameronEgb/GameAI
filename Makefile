# Compiler
CXX = g++

# Check architecture to set correct Homebrew paths
ARCH := $(shell uname -m)
ifeq ($(ARCH), arm64)
	# Path for Apple Silicon Macs (M1, M2, etc.)
	BREW_PREFIX = /opt/homebrew
else
	# Path for Intel Macs
	BREW_PREFIX = /usr/local
endif

# Compiler flags
CXXFLAGS = -std=c++17 -Wall -I$(BREW_PREFIX)/include

# Linker flags (we no longer need the thread library)
LDFLAGS = -L$(BREW_PREFIX)/lib -lsfml-graphics -lsfml-window -lsfml-system

# Source file and target executable name
SRC = main.cpp
TARGET = app

# Default target: builds the executable
all: $(TARGET)

# Rule that describes how to build the target
$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(LDFLAGS)

# A convenient rule to run the program
run: all
	./$(TARGET)

# A rule to clean up compiled files
clean:
	rm -f $(TARGET)

# Tells make that these targets don't correspond to actual files
.PHONY: all run clean