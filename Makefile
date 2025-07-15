# Makefile for auto_drive_project

CXX = ccache g++
PYTHON_INCLUDE = -I/usr/include/python3.10 -I/home/orda/.local/lib/python3.10/site-packages/pybind11/include
PYTHON_LIBS = -lpython3.10
CXXFLAGS = -std=c++17 -Iinclude $(PYTHON_INCLUDE) `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4` -lrt -pthread $(PYTHON_LIBS)

SRC = \
    src/main.cpp \
    src/usb_cam.cpp \
    src/video_recorder.cpp \
    src/lane_detector.cpp \
    src/object_detector.cpp \
    src/control.cpp

OUT = auto_drive

all:
	$(CXX) $(SRC) -o $(OUT) $(CXXFLAGS) $(LDFLAGS)

clean:
	rm -f $(OUT)
