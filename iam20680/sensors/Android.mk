# Copyright (C) 2016-2017 The Android Open Source Project
# Copyright (C) 2017-2020 InvenSense, Inc.
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

include $(LOCAL_PATH)/invensense.mk

# InvenSense Sensors HAL
include $(CLEAR_VARS)

LOCAL_MODULE := sensors.$(INVENSENSE_CHIP)
LOCAL_MODULE_OWNER := invensense
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_PROPRIETARY_MODULE := true

LOCAL_HEADER_LIBRARIES := libhardware_headers

LOCAL_CFLAGS := -Wall -Wextra -Werror
LOCAL_CFLAGS += -DLOG_TAG=\"Sensors\"
LOCAL_CFLAGS += $(INV_CFLAGS)

LOCAL_SRC_FILES += $(INV_SRCS)
LOCAL_C_INCLUDES += $(foreach inc, $(INV_INCLUDES), $(LOCAL_PATH)/$(inc))

LOCAL_SHARED_LIBRARIES := liblog
LOCAL_SHARED_LIBRARIES += libcutils
LOCAL_SHARED_LIBRARIES += libutils

LOCAL_PRELINK_MODULE := true

include $(BUILD_SHARED_LIBRARY)
