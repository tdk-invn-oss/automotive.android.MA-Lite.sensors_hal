# HAL source files location
HAL_SRC_DIR := ../..

include $(HAL_SRC_DIR)/invensense.mk

# Compiler flags
CFLAGS += -O2
CFLAGS += -Wall -Wextra -Werror
CFLAGS += -std=gnu11
CFLAGS += $(INV_CFLAGS)

# source C files
SRC_C_FILES += test-sensors-sysfs.c

# include dirs
CFLAGS += $(foreach inc, $(INV_INCLUDES), -I$(HAL_SRC_DIR)/$(inc))

# test application
TEST_MODULE := test-sensors-sysfs

OBJ_FILES := $(SRC_C_FILES:.c=.o)

.PHONY: all clean

all: $(TEST_MODULE)

clean:
	-rm -f $(OBJ_FILES) $(TEST_MODULE)

$(TEST_MODULE): $(OBJ_FILES)
	$(CC) $(CFLAGS) $(OBJ_FILES) -o $@
