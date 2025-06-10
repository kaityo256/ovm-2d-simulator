CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

TARGET = ovm-2d-simulator

SRC = main.cpp

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

.PHONY: clean
clean:
	rm -f $(TARGET)
