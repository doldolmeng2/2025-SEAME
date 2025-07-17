# 변수 설정
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
LDFLAGS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lgstreamer-1.0 -L/usr/lib/python3.10/config-*/ -lpython3.10

INCLUDES = -I./ -I/usr/include/opencv4 -I./include -I/usr/include/python3.10 -I/home/dengg2/.local/lib/python3.10/site-packages/pybind11/include

# 디렉토리 설정
SRC_DIR = src
BUILD_DIR = build

# 소스 및 타겟
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)
EXEC = auto_drive

# 빌드 디렉토리 생성
$(shell mkdir -p $(BUILD_DIR))

# 기본 타겟
all: $(EXEC)

# 실행 파일 생성
$(EXEC): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(EXEC) $(LDFLAGS)

# 객체 파일 생성 규칙
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# 클린 작업
clean:
	rm -rf $(BUILD_DIR)/* $(EXEC)

# 디버깅용
debug:
	$(CXX) $(CXXFLAGS) -g $(SOURCES) -o debug_$(EXEC) $(LDFLAGS)
