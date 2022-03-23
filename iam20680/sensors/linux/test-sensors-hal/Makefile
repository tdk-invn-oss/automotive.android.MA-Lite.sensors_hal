# HAL source files location
HAL_SRC_DIR := ../..

# InvenSense chip type
# Select from "iam20680", "icm20602", "icm20690", "icm42600", "icm42686"
# (Note: select "icm42600" for icm40607)
INVENSENSE_CHIP ?= iam20680

# Sensors HAL version
INV_SENSORS_HAL_VERSION_MAJOR := 9
INV_SENSORS_HAL_VERSION_MINOR := 1
INV_SENSORS_HAL_VERSION_PATCH := 2
INV_SENSORS_HAL_VERSION_SUFFIX := -simple-android-linux
$(info InvenSense Sensors HAL version MA-$(INV_SENSORS_HAL_VERSION_MAJOR).$(INV_SENSORS_HAL_VERSION_MINOR).$(INV_SENSORS_HAL_VERSION_PATCH)$(INV_SENSORS_HAL_VERSION_SUFFIX))

# Batching support
ifneq (,$(filter $(INVENSENSE_CHIP), iam20680))
CPPFLAGS += -DBATCH_MODE_SUPPORT
endif

# ODR configuration according to chip type
# Define for devices with SMPLRT_DIV register
ifneq (,$(filter $(INVENSENSE_CHIP), iam20680 icm20602 icm20690))
CPPFLAGS += -DODR_SMPLRT_DIV
endif

# Enhanced FSR support (4000dps, 32g)
ifneq (,$(filter $(INVENSENSE_CHIP), icm42686))
CPPFLAGS += -DACCEL_ENHANCED_FSR_SUPPORT
CPPFLAGS += -DGYRO_ENHANCED_FSR_SUPPORT
endif

# FIFO high resolution mode
ifneq (,$(filter $(INVENSENSE_CHIP), icm42686))
CPPFLAGS += -DFIFO_HIGH_RES_ENABLE
endif

# Compass support
COMPASS_SUPPORT := false
$(info InvenSense Compass support = $(COMPASS_SUPPORT))
ifeq ($(COMPASS_SUPPORT), true)
CPPFLAGS += -DCOMPASS_SUPPORT
endif

# Use LLVM libc++ with Clang, GNU libstdc++ by default
ifeq ($(CXX), clang++)
CXX_STL = -lc++
CXXFLAGS += -stdlib=libc++
else
CXX_STL = -lstdc++
endif

# Common flags
CPPFLAGS += -DLOG_TAG=\"Sensors\"
CPPFLAGS += -Iandroid_linux -D_HW_DONT_INCLUDE_CORE_
CPPFLAGS += -I$(HAL_SRC_DIR)/tools
COMMON_FLAGS += -O2
COMMON_FLAGS += -Wall -Wextra -Werror
COMMON_FLAGS += -fPIC -pthread
CFLAGS += -std=gnu11 $(COMMON_FLAGS)
CXXFLAGS += -std=gnu++11 $(COMMON_FLAGS)

# InvenSense Sensors HAL
INVNSENSORS := libinvnsensors.so
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_MAJOR=$(INV_SENSORS_HAL_VERSION_MAJOR)
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_MINOR=$(INV_SENSORS_HAL_VERSION_MINOR)
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_PATCH=$(INV_SENSORS_HAL_VERSION_PATCH)
CPPFLAGS += -DINV_SENSORS_HAL_VERSION_SUFFIX=\"$(INV_SENSORS_HAL_VERSION_SUFFIX)\"
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/SensorsMain.cpp
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/SensorBase.cpp
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/MPLSensor.cpp
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/MPLSupport.cpp
ifeq ($(COMPASS_SUPPORT), true)
INVNSENSORS_SRC_CPP_FILES += $(HAL_SRC_DIR)/CompassSensor.IIO.primary.cpp
endif
INVNSENSORS_SRC_C_FILES += $(HAL_SRC_DIR)/tools/inv_sysfs_utils.c
INVNSENSORS_SRC_C_FILES += $(HAL_SRC_DIR)/tools/inv_iio_buffer.c
INVNSENSORS_SRC_C_FILES += $(HAL_SRC_DIR)/tools/ml_sysfs_helper.c
INVNSENSORS_LDFLAGS += $(LDFLAGS) -L./ -shared -lpthread $(CXX_STL)
INVNSENSORS_OBJ_FILES := $(INVNSENSORS_SRC_C_FILES:.c=.o) $(INVNSENSORS_SRC_CPP_FILES:.cpp=.o)

# test application
TESTAPP := test-sensors-hal
TESTAPP_SRC_CPP_FILES := test-sensors-hal.cpp
TESTAPP_LDFLAGS = $(LDFLAGS) -ldl -lpthread $(CXX_STL)
TESTAPP_OBJ_FILES := $(TESTAPP_SRC_C_FILES:.c=.o) $(TESTAPP_SRC_CPP_FILES:.cpp=.o)

.PHONY: all clean

all: $(INVNSENSORS) $(TESTAPP)

clean:
	-rm -f $(INVNSENSORS_OBJ_FILES) $(INVNSENSORS)
	-rm -f $(TESTAPP_OBJ_FILES) $(TESTAPP)

$(INVNSENSORS): $(MLLITE) $(INVNSENSORS_OBJ_FILES)
	$(CXX) $(CXXFLAGS) $^ $(INVNSENSORS_LDFLAGS) $(INVNSENSORS_LIBS) -o $@

$(TESTAPP): $(TESTAPP_OBJ_FILES)
	$(CXX) $(CXXFLAGS) $^ $(TESTAPP_LDFLAGS) -o $@
