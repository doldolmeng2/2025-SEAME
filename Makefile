# ──────────────────────────────────────────────────
# Project Makefile (faster, parallel, incremental)
# ──────────────────────────────────────────────────

# 콤파일러 및 플래그
CXX       := ccache g++
CXXFLAGS  := -std=c++17 -Iinclude \
              -I/usr/include/python3.10 \
              `pkg-config --cflags opencv4` \
              -MMD -MP               # 의존성 자동 생성
LDFLAGS   := `pkg-config --libs opencv4` -lrt -pthread -lpython3.10

# 소스/객체 정의
SRC_DIR   := src
BUILD_DIR := build
SRCS      := $(wildcard $(SRC_DIR)/*.cpp)
OBJS      := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRCS))
DEPS      := $(OBJS:.o=.d)

# 출력 바이너리
TARGET    := auto_drive

# 병렬 빌드 기본 옵션 (CPU 코어 개수만큼)
MAKEFLAGS += -j$(shell nproc)

.PHONY: all clean

all: $(TARGET)

# 최종 링크
$(TARGET): $(OBJS)
	$(CXX) $^ -o $@ $(LDFLAGS)

# 객체 파일 빌드
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 빌드 폴더 생성
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# 클린업
clean:
	rm -rf $(BUILD_DIR) $(TARGET) $(DEPS)

# 의존성 자동 포함
-include $(DEPS)
