# Compiler and executable name
CXX      := g++
TARGET   := boid_simulation

# C++ standard and compiler flags (warnings, etc.)
# Using C++17 for modern features like structured bindings in the event loop
CXXFLAGS := -std=c++17 -Wall -Wextra

# Source files and corresponding object files
SRCS     := main.cpp
OBJS     := $(SRCS:.cpp=.o)

# --- OS-Specific Configuration ---
# Detects the OS and sets the correct paths and libraries for SFML
UNAME_S := $(shell uname -s)

# macOS configuration
ifeq ($(UNAME_S), Darwin)
    # Assumes SFML was installed with Homebrew.
    # This command finds the Homebrew prefix automatically for both Apple Silicon and Intel Macs.
    BREW_PREFIX := $(shell brew --prefix)
    CXXFLAGS    += -I$(BREW_PREFIX)/include
    LDFLAGS     := -L$(BREW_PREFIX)/lib
    LDLIBS      := -lsfml-graphics -lsfml-window -lsfml-system

# Linux (Ubuntu) configuration
else ifeq ($(UNAME_S), Linux)
    # Assumes SFML was installed via apt or another system package manager.
    LDLIBS      := -lsfml-graphics -lsfml-window -lsfml-system

# Fallback for other systems
else
    $(warning "Unsupported OS. You may need to set SFML paths manually.")
endif
# --- End of OS-Specific Configuration ---


# --- Build Rules ---

# Default rule: builds the executable
all: $(TARGET)

# Rule to link the object files into the final executable
$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $(TARGET) $(LDFLAGS) $(LDLIBS)

# Rule to compile a .cpp source file into a .o object file
# The -c flag means "compile only, do not link"
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to remove generated object files
clean:
	rm -f $(OBJS)

# Rule to remove the executable and object files
fclean: clean
	rm -f $(TARGET)

# Rule to force a full rebuild
re: fclean all

# Phony targets are not files; they are just names for commands.
.PHONY: all clean fclean re