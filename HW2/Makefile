# Simple Makefile for HW2 Part 1 (no CMake)
# Tested on Ubuntu 20.04 / 22.04 with SFML 3.0.0

CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2
LDFLAGS := -lsfml-graphics -lsfml-window -lsfml-system

SRC := main.cpp
OBJ := $(SRC:.cpp=.o)
BIN := steering_demo

all: $(BIN)

$(BIN): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(OBJ) $(BIN)

.PHONY: all run clean
