# Makefile for auto_drive_project

CXX = g++
CXXFLAGS = -std=c++17 -Iinclude -Ipiracer-cpp `pkg-config --cflags opencv4`
LDFLAGS = `pkg-config --libs opencv4` -lpigpio -lrt -pthread

SRC = \
    src/main.cpp \
    src/usb_cam.cpp \
    src/video_recorder.cpp \
	src/lane_detector.cpp \
	src/object_detector.cpp \
	src/control.cpp \
	piracer-cpp/PiRacer/PiRacer.cpp \
	piracer-cpp/Adafruit_INA219/Adafruit_INA219.cpp \
	piracer-cpp/Adafruit_PCA9685/Adafruit_PCA9685.cpp

OUT = auto_drive

all:
	$(CXX) $(SRC) -o $(OUT) $(CXXFLAGS) $(LDFLAGS)

clean:
	rm -f $(OUT)
