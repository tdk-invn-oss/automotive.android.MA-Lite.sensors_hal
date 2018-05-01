# Sensors HAL
CPPFLAGS += -DLOG_TAG=\"Sensors\"
CPPFLAGS += -Iandroid_linux -D_HW_DONT_INCLUDE_CORE_

# HAL source files location
HAL_SRC_DIR := ../..

# Sensors HAL version
INV_SENSORS_HAL_VERSION_MAJOR := 8
INV_SENSORS_HAL_VERSION_MINOR := 1
INV_SENSORS_HAL_VERSION_PATCH := 5
INV_SENSORS_HAL_VERSION_SUFFIX := -simple-android-linux-test1
$(info InvenSense Sensors HAL version MA-$(INV_SENSORS_HAL_VERSION_MAJOR).$(INV_SENSORS_HAL_VERSION_MINOR).$(INV_SENSORS_HAL_VERSION_PATCH)$(INV_SENSORS_HAL_VERSION_SUFFIX))
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_MAJOR=$(INV_SENSORS_HAL_VERSION_MAJOR)
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_MINOR=$(INV_SENSORS_HAL_VERSION_MINOR)
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_PATCH=$(INV_SENSORS_HAL_VERSION_PATCH)
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_SUFFIX=\"$(INV_SENSORS_HAL_VERSION_SUFFIX)\"

# InvenSense chip type
INVENSENSE_CHIP ?= iam20680
$(info InvenSense chip $(INVENSENSE_CHIP))

# Batching support
ifneq (,$(filter $(INVENSENSE_CHIP), iam20680))
CPPFLAGS += -DBATCH_MODE_SUPPORT
endif

# Compass support
COMPASS_SUPPORT := false
$(info InvenSense Compass support = $(COMPASS_SUPPORT))
ifeq ($(COMPASS_SUPPORT), true)
CPPFLAGS += -DCOMPASS_SUPPORT
endif

# Android version
MAJOR_VERSION := 8
MINOR_VERSION := 1
$(info ANDROID_VERSION = $(MAJOR_VERSION).$(MINOR_VERSION))
CPPFLAGS += -DANDROID_MAJOR_VERSION=$(MAJOR_VERSION)
CPPFLAGS += -DANDROID_MINOR_VERSION=$(MINOR_VERSION)

# Compiler flags
MODULE := libinvnsensors.so
CFLAGS += -O2
CFLAGS += -Wall -Wextra -Werror
CFLAGS += -fPIC -pthread
CXXFLAGS := $(CFLAGS)
LDFLAGS := -shared -lstdc++ -lpthread

# source C++ files
SRC_CPP_FILES += $(HAL_SRC_DIR)/SensorsMain.cpp
SRC_CPP_FILES += $(HAL_SRC_DIR)/SensorBase.cpp
SRC_CPP_FILES += $(HAL_SRC_DIR)/MPLSensor.cpp
SRC_CPP_FILES += $(HAL_SRC_DIR)/MPLSupport.cpp
ifeq ($(COMPASS_SUPPORT), true)
SRC_CPP_FILES += $(HAL_SRC_DIR)/CompassSensor.IIO.primary.cpp
endif

# source C files
SRC_C_FILES += $(HAL_SRC_DIR)/tools/inv_sysfs_utils.c
SRC_C_FILES += $(HAL_SRC_DIR)/tools/inv_iio_buffer.c
SRC_C_FILES += $(HAL_SRC_DIR)/tools/ml_sysfs_helper.c
SRC_C_FILES += $(HAL_SRC_DIR)/tools/ml_sensor_parsing.c

# include dirs
CPPFLAGS += -I$(HAL_SRC_DIR)/tools

# test application
TEST_MODULE := test-sensors-hal
TEST_MODULE_FILES := test-sensors-hal.cpp

OBJ_FILES := $(SRC_C_FILES:.c=.o) $(SRC_CPP_FILES:.cpp=.o)

.PHONY: all clean

all: $(MODULE) $(TEST_MODULE)

clean:
	-rm -f $(OBJ_FILES) $(MODULE) $(TEST_MODULE)

$(MODULE): $(OBJ_FILES)
	$(CXX) $(CFLAGS) $(LDFLAGS) $^ -o $@

$(TEST_MODULE): $(MODULE)
	$(CXX) $(CPPFLAGS) $(CFLAGS) $(TEST_MODULE_FILES) -ldl -lpthread -lstdc++ -o $@
