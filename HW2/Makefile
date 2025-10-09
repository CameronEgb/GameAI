# CSC 584/484 Fall 2025 - Homework 2: Steering Behaviors
# Author: [Your Name]
# Description:
#   Versatile Makefile supporting both static and shared SFML linking.

# === Compiler and Flags ===
CXX := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2

# === SFML Search Paths (override with: make SFML_PREFIX=/opt/sfml3) ===
SFML_PREFIX ?= /usr/local
CANDIDATE_INCDIRS := $(SFML_PREFIX)/include /usr/include
CANDIDATE_LIBDIRS := $(SFML_PREFIX)/lib $(SFML_PREFIX)/lib64 /usr/lib/x86_64-linux-gnu

# === Detect SFML include directory ===
SFML_INCDIR := $(shell for d in $(CANDIDATE_INCDIRS); do \
    [ -d $$d/SFML ] && echo $$d && break; \
done)
CXXFLAGS += -I$(SFML_INCDIR)

# === Detect SFML library directory (static preferred) ===
SFML_LIBDIR_STATIC := $(shell for d in $(CANDIDATE_LIBDIRS); do \
    ls $$d/libsfml-graphics-s.a $$d/libsfml-window-s.a $$d/libsfml-system-s.a >/dev/null 2>&1 && { echo $$d; break; }; \
done)

SFML_LIBDIR_SHARED := $(shell for d in $(CANDIDATE_LIBDIRS); do \
    ls $$d/libsfml-graphics.so* $$d/libsfml-window.so* $$d/libsfml-system.so* >/dev/null 2>&1 && { echo $$d; break; }; \
done)

# === Choose linking type automatically ===
ifeq ($(SFML_LIBDIR_STATIC),)
    SFML_LIBDIR := $(SFML_LIBDIR_SHARED)
    SFML_LIBS := -L$(SFML_LIBDIR) -lsfml-graphics -lsfml-window -lsfml-system
else
    SFML_LIBDIR := $(SFML_LIBDIR_STATIC)
    CXXFLAGS += -DSFML_STATIC
    SFML_LIBS := -L$(SFML_LIBDIR) -lsfml-graphics-s -lsfml-window-s -lsfml-system-s \
                 -lX11 -lXrandr -lXcursor -lXi -lXinerama -lfreetype -lGL -ludev -lpthread -ldl
endif

# === Files and Targets ===
SRC := main.cpp
TARGET := steering

# === Default Target ===
all: $(TARGET)

$(TARGET): $(SRC)
	@echo "Using SFML include dir: $(SFML_INCDIR)"
	@echo "Using SFML lib dir: $(SFML_LIBDIR)"
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET) $(SFML_LIBS)

# === Run Program ===
run: $(TARGET)
	./$(TARGET)

# === Clean ===
clean:
	rm -f $(TARGET)
