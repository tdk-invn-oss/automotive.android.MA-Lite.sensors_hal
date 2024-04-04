LOCAL_PATH := $(call my-dir)/../

ifneq ($(NDK_ROOT),)

include $(CLEAR_VARS)
LOCAL_MODULE := iio-buffer-read
LOCAL_CFLAGS := -Wall -Wextra -Werror
LOCAL_SRC_FILES := iio-buffer-read.c
LOCAL_LDLIBS :=
include $(BUILD_EXECUTABLE)

endif
