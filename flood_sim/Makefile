CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -O2 -Wno-unused-result
TARGET   := micromouse
SRCS     := main.cpp Maze.cpp Interface.cpp API.cpp

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRCS)

clean:
	rm -f $(TARGET)
