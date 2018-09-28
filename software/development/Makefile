TARGET = HPS_CONTROL_FPGA

#fpga compile flags
CROSS_COMPILE = arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc
ARCH= arm
ALT_DEVICE_FAMILY ?= soc_cv_av

CFLAGS = -g -Wall -D $(ALT_DEVICE_FAMILY) -I
LDFLAGS = -g -Wall -pthread

VPATH = functions fpga_headers
HEADERS = main.h hps_0.h
OBJ_DIR = objs
_OBJS = main.o thread_funcs.o utils.o signal_handler.o
OBJS = $(patsubst %,$(OBJ_DIR)/%,$(_OBJS))

LIBS = -lm

$(OBJ_DIR)/%.o: %.c $(HEADERS)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

build: $(TARGET)
$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~ $(OBJ_DIR)/*.o

#notes ->  $< name of first prerequisite, $@ the file name of the target of the rule, $^ the names of all the prerequisites, with spaces between them.
#-lm is to load the <math.h> library
#.o the rule for all .o files, things after the colon indicate if any of those change the object file needs to be regenerated

#VPATH The value of the make variable VPATH specifies a list of directories that make should search. Most often, the directories are expected to 
#contain prerequisite files that are not in the current directory; however, make uses VPATH as a search list for both prerequisites and targets of rules.