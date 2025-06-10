CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall

TARGET = ovm-2d-simulator

SRC = main.cpp

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC)

run: $(TARGET)
	./ovm-2d-simulator

gif: run
	python3 render_frames.py
	ffmpeg -y -i frame%04d.png animation.gif
	ffmpeg -y -i frame%04d.png animation.gif

.PHONY: clean clear run
clean:
	rm -f $(TARGET)

clear: clean
	rm -f *.png *.dat
