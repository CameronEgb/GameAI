# CSC 584/484 Fall 2025 - Homework 2: Steering Behaviors
# Author: [Your Name]
# Description:
#   Makefile for compiling and running the SFML steering behavior demos.

# === Compiler and Flags ===
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2

# === SFML Linking (adjust if SFML is in a custom path) ===
SFML_LIBS := -Lusr/local/lib/cmake -lsfml-graphics -lsfml-window -lsfml-system

# === Files and Output ===
SRC := main.cpp
TARGET := steering

# === Default Target ===
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(SFML_LIBS)

# === Run Program ===
run: $(TARGET)
	./$(TARGET)

# === Clean ===
clean:
	rm -f $(TARGET)

# === Rebuild ===
rebuild: clean all

.PHONY: all run clean rebuild
