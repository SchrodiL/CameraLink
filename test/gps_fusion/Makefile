CC      = gcc
CFLAGS  = -Wall -Wextra -std=c99 -O2 -I../../services
SRCDIR  = ../..
SOURCES = test_gps_fusion.c $(SRCDIR)/services/gps_fusion_math.c
TARGET  = test_gps_fusion
LDLIBS  = -lm

# 纯算法模块不依赖 ESP-IDF，可以直接在主机上编译运行。
# 注意路径是**重构后**的；test/frame_check/Makefile 里还是重构前的 ../utils/crc，
# 那个已经失效了（本次没顺手修，避免混进无关改动）。
$(TARGET): $(SOURCES) $(SRCDIR)/services/gps_fusion_math.h
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCES) $(LDLIBS)

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

.PHONY: test clean
