# Compiler
CXX = g++
# Flags: C++17, Level 3 Optimization (Make it fast!), Warnings
CXXFLAGS = -std=c++17 -O3 -Wall

# Libraries to link: Raylib, OpenGL, Math, Pthread
LDFLAGS = -lraylib -lGL -lm -lpthread -ldl -lrt -lX11

# Your executable name
TARGET = engine

# Files to compile
SRC = main.cpp

all:
	$(CXX) $(SRC) -o $(TARGET) $(CXXFLAGS) $(LDFLAGS)

clean:
	rm -f $(TARGET)