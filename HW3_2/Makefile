CXX = g++
CXXFLAGS = -std=c++17 -O2 -Iinclude -Wall -Wextra
LDFLAGS =

SRCS = src/Graph.cpp src/Pathfinder.cpp src/main.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = pathfinder_demo

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
