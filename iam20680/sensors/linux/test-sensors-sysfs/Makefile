# HAL source files location
HAL_SRC_DIR := ../..

# Compiler flags
CFLAGS += -O2
CFLAGS += -Wall -Wextra -Werror
CFLAGS += -std=gnu11

# Enable FIFO high resolution mode (only for ICM42686)
#CFLAGS += -DHIGH_RES_MODE_ENABLE

# source C files
SRC_C_FILES += test-sensors-sysfs.c

# include dirs
CFLAGS += -I$(HAL_SRC_DIR)/tools

# test application
TEST_MODULE := test-sensors-sysfs

OBJ_FILES := $(SRC_C_FILES:.c=.o)

.PHONY: all clean

all: $(TEST_MODULE)

clean:
	-rm -f $(OBJ_FILES) $(TEST_MODULE)

$(TEST_MODULE): $(OBJ_FILES)
	$(CC) $(CFLAGS) $(OBJ_FILES) -o $@
