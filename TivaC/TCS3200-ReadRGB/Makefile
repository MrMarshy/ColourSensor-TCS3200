# Defines the part type that this project uses.
#
PART=TM4C123GH6PM
LIBROOT=C:\Users\alan\Downloads\CrashCourse\BareMetal\ExampleProgs\TivaExampleProgs\tiva-c

# ADD Sensor Libraries etc include directories here 
TCS3200_DIR=./TCS3200
vpath %.c $(TCS3200_DIR)
CFLAGS+= -I$(TCS3200_DIR)

# addition al compilation flags
OBJS= main.o
OBJS+= tiva_uart.o 
OBJS+= bsp.o 
OBJS+= Utils.o 
OBJS+= tinyprintf.o
OBJS+= $(STARTUP) syscalls.o
OBJS+= TCS3200.o
#
# Include the common make definitions.
#
include $(LIBROOT)/makedefswin

