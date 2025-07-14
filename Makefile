# Makefile for auto_drive_project

CXX = g++
CXXFLAGS = -std=c++17 -Iinclude `pkg-config --cflags --libs opencv4`

SRC = \
    src/main.cpp \
    src/usb_cam.cpp \
    src/video_recorder.cpp \
	src/lane_detector.cpp \
	src/object_detector.cpp

OUT = auto_drive

all:
	$(CXX) $(SRC) -o $(OUT) $(CXXFLAGS)

clean:
	rm -f $(OUT)
