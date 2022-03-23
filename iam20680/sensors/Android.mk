# Copyright (C) 2016-2017 The Android Open Source Project
# Copyright (C) 2017-2019 InvenSense, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Modified 2016 by InvenSense, Inc

LOCAL_PATH := $(call my-dir)

# InvenSense Sensors HAL
include $(CLEAR_VARS)

LOCAL_MODULE := sensors.$(TARGET_DEVICE)
LOCAL_MODULE_OWNER := invensense
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_CFLAGS := -Wall -Wextra -Werror
LOCAL_CFLAGS += -DLOG_TAG=\"Sensors\"

# Sensors HAL version
INV_SENSORS_HAL_VERSION_MAJOR := 9
INV_SENSORS_HAL_VERSION_MINOR := 1
INV_SENSORS_HAL_VERSION_PATCH := 2
INV_SENSORS_HAL_VERSION_SUFFIX := -simple-android-linux
$(info InvenSense Sensors HAL version MA-$(INV_SENSORS_HAL_VERSION_MAJOR).$(INV_SENSORS_HAL_VERSION_MINOR).$(INV_SENSORS_HAL_VERSION_PATCH)$(INV_SENSORS_HAL_VERSION_SUFFIX))
LOCAL_CFLAGS += -DINV_SENSORS_HAL_VERSION_MAJOR=$(INV_SENSORS_HAL_VERSION_MAJOR)
LOCAL_CFLAGS += -DINV_SENSORS_HAL_VERSION_MINOR=$(INV_SENSORS_HAL_VERSION_MINOR)
LOCAL_CFLAGS += -DINV_SENSORS_HAL_VERSION_PATCH=$(INV_SENSORS_HAL_VERSION_PATCH)
LOCAL_CFLAGS += -DINV_SENSORS_HAL_VERSION_SUFFIX=\"$(INV_SENSORS_HAL_VERSION_SUFFIX)\"

# InvenSense chip type
# Select from "iam20680", "icm20602", "icm20690", "icm42600", "icm42686"
# (Note: select "icm42600" for icm40607)
INVENSENSE_CHIP ?= iam20680
$(info InvenSense chip $(INVENSENSE_CHIP))

# Batch mode support
ifneq (,$(filter $(INVENSENSE_CHIP), iam20680))
LOCAL_CFLAGS += -DBATCH_MODE_SUPPORT
endif

# ODR configuration according to chip type
# Define for devices with SMPLRT_DIV register
ifneq (,$(filter $(INVENSENSE_CHIP), iam20680 icm20602 icm20690))
LOCAL_CFLAGS += -DODR_SMPLRT_DIV
endif

# Enhanced FSR support (4000dps, 32g)
ifneq (,$(filter $(INVENSENSE_CHIP), icm42686))
LOCAL_CFLAGS += -DACCEL_ENHANCED_FSR_SUPPORT
LOCAL_CFLAGS += -DGYRO_ENHANCED_FSR_SUPPORT
endif

# FIFO high resolution mode
ifneq (,$(filter $(INVENSENSE_CHIP), icm42686))
LOCAL_CFLAGS += -DFIFO_HIGH_RES_ENABLE
endif

# Compass support
COMPASS_SUPPORT := false
$(info InvenSense Compass support = $(COMPASS_SUPPORT))
ifeq ($(COMPASS_SUPPORT), true)
LOCAL_CFLAGS += -DCOMPASS_SUPPORT
endif

# Android version check
MAJOR_VERSION :=$(shell echo $(PLATFORM_VERSION) | cut -f1 -d.)
$(info ANDROID_VERSION = $(MAJOR_VERSION))

# Android O support
ifeq ($(shell test $(MAJOR_VERSION) -ge 8 && echo true), true)
LOCAL_PROPRIETARY_MODULE := true
LOCAL_HEADER_LIBRARIES := libhardware_headers
endif

LOCAL_SRC_FILES += SensorsMain.cpp
LOCAL_SRC_FILES += SensorBase.cpp
LOCAL_SRC_FILES += MPLSensor.cpp
LOCAL_SRC_FILES += MPLSupport.cpp

LOCAL_SRC_FILES += tools/inv_sysfs_utils.c
LOCAL_SRC_FILES += tools/inv_iio_buffer.c
LOCAL_SRC_FILES += tools/ml_sysfs_helper.c
ifeq ($(COMPASS_SUPPORT), true)
LOCAL_SRC_FILES += CompassSensor.IIO.primary.cpp
endif

LOCAL_C_INCLUDES += $(LOCAL_PATH)/tools

LOCAL_SHARED_LIBRARIES := liblog
LOCAL_SHARED_LIBRARIES += libcutils
LOCAL_SHARED_LIBRARIES += libutils

LOCAL_PRELINK_MODULE := true

include $(BUILD_SHARED_LIBRARY)

