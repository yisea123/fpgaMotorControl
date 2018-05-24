#
TARGET = HPS_CONTROL_FPGA_LED

#
CROSS_COMPILE = arm-linux-gnueabihf-
#SOCEDS_DEST_ROOT = C:/intelFPGA_pro/17.0/embedded
ALT_DEVICE_FAMILY ?= soc_cv_av

CFLAGS = -g -Wall -I${SOCEDS_DEST_ROOT}/ip/altera/hps/altera_hps/hwlib/include -D $(ALT_DEVICE_FAMILY)
LDFLAGS = -g -Wall -pthread
CC = $(CROSS_COMPILE)gcc
ARCH= arm


build: $(TARGET)
$(TARGET): main.o led.o
	$(CC) $(LDFLAGS)   $^ -o $@ -lm
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@ -lm

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~ 
