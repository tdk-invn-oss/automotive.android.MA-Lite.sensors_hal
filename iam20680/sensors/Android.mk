# Copyright (C) 2016-2017 The Android Open Source Project
# Copyright (C) 2017-2018 InvenSense, Inc.
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
INV_SENSORS_HAL_VERSION_MAJOR := 8
INV_SENSORS_HAL_VERSION_MINOR := 1
INV_SENSORS_HAL_VERSION_PATCH := 4
INV_SENSORS_HAL_VERSION_SUFFIX := -simple-android-linux-test1a
$(info InvenSense Sensors HAL version MA-$(INV_SENSORS_HAL_VERSION_MAJOR).$(INV_SENSORS_HAL_VERSION_MINOR).$(INV_SENSORS_HAL_VERSION_PATCH)$(INV_SENSORS_HAL_VERSION_SUFFIX))
LOCAL_CFLAGS += -DINV_SENSORS_HAL_VERSION_MAJOR=$(INV_SENSORS_HAL_VERSION_MAJOR)
LOCAL_CFLAGS += -DINV_SENSORS_HAL_VERSION_MINOR=$(INV_SENSORS_HAL_VERSION_MINOR)
LOCAL_CFLAGS += -DINV_SENSORS_HAL_VERSION_PATCH=$(INV_SENSORS_HAL_VERSION_PATCH)
LOCAL_CFLAGS += -DINV_SENSORS_HAL_VERSION_SUFFIX=\"$(INV_SENSORS_HAL_VERSION_SUFFIX)\"

# Compass support
COMPASS_SUPPORT := false
$(info InvenSense Compass support = $(COMPASS_SUPPORT))
ifeq ($(COMPASS_SUPPORT), true)
LOCAL_CFLAGS += -DCOMPASS_SUPPORT
endif

# Android version check
MAJOR_VERSION :=$(shell echo $(PLATFORM_VERSION) | cut -f1 -d.)
MINOR_VERSION :=$(shell echo $(PLATFORM_VERSION) | cut -f2 -d.)
$(info ANDROID_VERSION = $(MAJOR_VERSION).$(MINOR_VERSION))
LOCAL_CFLAGS += -DANDROID_MAJOR_VERSION=$(MAJOR_VERSION)
LOCAL_CFLAGS += -DANDROID_MINOR_VERSION=$(MINOR_VERSION)

# Android O support
ifeq ($(shell test $(MAJOR_VERSION) -gt 7 && echo true), true)
LOCAL_PROPRIETARY_MODULE := true
endif

LOCAL_SRC_FILES += SensorsMain.cpp
LOCAL_SRC_FILES += SensorBase.cpp
LOCAL_SRC_FILES += MPLSensor.cpp
LOCAL_SRC_FILES += MPLSupport.cpp

LOCAL_SRC_FILES += tools/inv_sysfs_utils.c
LOCAL_SRC_FILES += tools/inv_iio_buffer.c
LOCAL_SRC_FILES += tools/ml_sysfs_helper.c
LOCAL_SRC_FILES += tools/ml_sensor_parsing.c
ifeq ($(COMPASS_SUPPORT), true)
LOCAL_SRC_FILES += CompassSensor.IIO.primary.cpp
endif

LOCAL_C_INCLUDES += $(LOCAL_PATH)/tools

LOCAL_SHARED_LIBRARIES := liblog
LOCAL_SHARED_LIBRARIES += libcutils
LOCAL_SHARED_LIBRARIES += libutils

LOCAL_PRELINK_MODULE := true

include $(BUILD_SHARED_LIBRARY)

