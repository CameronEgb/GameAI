# Executable name
TARGET = app

# Automatically find all .cpp files (works for single or multiple files)
SRC := $(wildcard *.cpp)

# Create a list of object files (.o) from the source files (.cpp)
OBJ = $(patsubst %.cpp,%.o,$(SRC))

# Common linker flags for SFML
LDFLAGS = -lsfml-graphics -lsfml-window -lsfml-system

# --- Platform-Specific Configuration ---

# Default Include/Library paths for different systems
MACOS_INCLUDE = -I/opt/homebrew/include
MACOS_LIB = -L/opt/homebrew/lib
UBUNTU_INCLUDE = -I/usr/include
UBUNTU_LIB = # Standard Ubuntu paths are usually searched by default

# Compilers
MACOS_COMPILER = clang++
UBUNTU_COMPILER = g++

# --- Build Rules ---

# Default target that `make` will run
all: $(TARGET)

# Get the Operating System name (e.g., "Darwin" for macOS, "Linux" for Linux)
UNAME_S := $(shell uname -s)

# Rule for linking the final executable
$(TARGET): $(OBJ)
ifeq ($(UNAME_S),Darwin)
	# macOS linking command
	$(MACOS_COMPILER) -std=c++17 -o $@ $^ $(MACOS_LIB) $(LDFLAGS)
else ifeq ($(UNAME_S),Linux)
	# Linux linking command
	$(UBUNTU_COMPILER) -std=c++17 -o $@ $^ $(UBUNTU_LIB) $(LDFLAGS)
endif

# Rule for compiling .cpp source files into .o object files
%.o: %.cpp
ifeq ($(UNAME_S),Darwin)
	# macOS compilation command
	$(MACOS_COMPILER) -std=c++17 -c $< -o $@ $(MACOS_INCLUDE)
else ifeq ($(UNAME_S),Linux)
	# Linux compilation command
	$(UBUNTU_COMPILER) -std=c++17 -c $< -o $@ $(UBUNTU_INCLUDE)
endif


# --- Utility Rules ---

# Rule to run the program
run: all
	./$(TARGET)

# Rule to clean up all compiled files
clean:
	rm -f $(OBJ) $(TARGET)

# Tell 'make' that these are command names, not files
.PHONY: all run clean