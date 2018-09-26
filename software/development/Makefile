#
TARGET = HPS_CONTROL_FPGA

#
CROSS_COMPILE = arm-linux-gnueabihf-
#SOCEDS_DEST_ROOT = C:/intelFPGA_pro/17.0/embedded
ALT_DEVICE_FAMILY ?= soc_cv_av

CFLAGS = -g -Wall -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include -D $(ALT_DEVICE_FAMILY)
LDFLAGS = -g -Wall -pthread
CC = $(CROSS_COMPILE)gcc
ARCH= arm

DEPS = main.h
OBJ = main.o

LIBS = -lm

#notes ->  $< name of first prerequisite, $@ the file name of the target of the rule, $^ the names of all the prerequisites, with spaces between them.

#is the rule for all .o files
#%.0 : %.c $(DEPS)


build: $(TARGET)
$(TARGET): main.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)
#	$(CC) $(LDFLAGS)   $^ -o $@ -lm
#%.o : %.c
#	$(CC) $(CFLAGS) -c $< -o $@ -lm

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~ 
