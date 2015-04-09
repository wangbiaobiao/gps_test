# 制作的目标文件名libnmeagps.so
TARGET = libnmeagps.so       

# 包含所有头文件
INCLUDE_FILE = context.h generate.h parse.h parser.h gtime.h
INCLUDE_FILE += tok.h units.h gmath.h nmea.h sentence.h info.h generator.h config.h

# 设置交叉编译工具
COMPILER_PATH = arm-linux-gnueabi-

CC = $(COMPILER_PATH)gcc
CXX = $(COMPILER_PATH)g++
LD = $(COMPILER_PATH)ld
AR = $(COMPILER_PATH)ar

LIBRARY = ./

INCLUDE_DIR = ./

FLAGS = -Wall -O3 -s -I. -I$(INCLUDE_DIR)

OBJS = $(patsubst %.c, %.o, $(wildcard *.c))

%.o:%.c
	@$(CC) -c $(FLAGS) $< -o $@ 

all:$(OBJS)
	@$(CC) -shared -Wall -fPIC -s $(OBJS) -o $(TARGET)
	@$(ar) libnmeagps.a $(OBJS)
    
update:
	@cp -a $(INCLUDE_FILE) $(INCLUDE_DIR) 1>/dev/null

clean:
	rm $(TARGET) *.o -f
