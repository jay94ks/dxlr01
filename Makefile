CXX      ?= g++
CXXFLAGS ?= -std=c++17 -g -O0 -Wall -Wextra
CPPFLAGS += -Isrc -MMD -MP
LDFLAGS  ?=
LDLIBS   ?=

SRC_DIR  := src test
BUILD_DIR := build
TARGET   := dxlr01

SRCS := $(shell find $(SRC_DIR) -type f \( -name '*.cpp' -o -name '*.cc' -o -name '*.cxx' -o -name '*.c++' \))
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

PORT_A ?= /dev/ttyUSB0
PORT_B ?= /dev/ttyUSB1

.PHONY: all clean rebuild run run-a run-b test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(BUILD_DIR)/%.o: %
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

rebuild: clean all

# 단일 포트 실행: make run PORT=/dev/ttyUSB0 EMIT=y
PORT ?= $(PORT_A)
EMIT ?= n
run: $(TARGET)
	./$(TARGET) $(PORT) $(EMIT)

# 송신(A, 최초 emit) / 수신 에코(B) 각각 실행
run-a: $(TARGET)
	./$(TARGET) $(PORT_A) y

run-b: $(TARGET)
	./$(TARGET) $(PORT_B) n

# 두 모듈 동시 테스트: A가 최초 송신, B가 수신 후 에코. Ctrl+C로 종료.
test: $(TARGET)
	@echo "A=$(PORT_A) (initial emit)  B=$(PORT_B)"
	@trap 'trap - INT TERM EXIT; kill 0' INT TERM EXIT; \
	 ./$(TARGET) $(PORT_B) n 2>&1 | sed -u 's/^/[B] /' & \
	 ./$(TARGET) $(PORT_A) y 2>&1 | sed -u 's/^/[A] /' & \
	 wait

-include $(DEPS)
