BIN_DIR=bin/
BUILD_DIR=build/
SRC_DIR=src/
INCLUDE_DIR=include/
LIB_DIR=lib/
RPATH==$(LIB_DIR)

SOURCES		:= $(wildcard $(SRC_DIR)*.c)
INCLUDES	:= $(wildcard $(INCLUDE_DIR)*.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)

LOCAL_CFLAGS=-O2 -Wall -Wextra -g -I$(INCLUDE_DIR)
LIBS=

TARGET=read_rgb

CXX=gcc

$(TARGET): $(OBJECTS)
	$(CXX) $(LOCAL_CFLAGS) $(CFLAGS) -o $(BIN_DIR)$(@) $(OBJECTS) $(LIBS) $(LD_FLAGS)
	mv $(SRC_DIR)*.o $(BUILD_DIR)

$(OBJECTS): %.o: %.c $(INCLUDES)
	$(CXX) $(LOCAL_CFLAGS) $(CFLAGS) -c $< -o $(@)
	

all: $(TARGET)

clean:
	rm -rf $(BUILD_DIR)*.o $(BIN_DIR)$(TARGET)
	rm -rf $(BIN_DIR)/*
	rm -rf $(SRC_DIR)/*.o