# Clear variables
INV_CFLAGS :=
INV_INCLUDES :=
INV_SRCS :=

# Version
INV_VERSION_MAJOR := 14
INV_VERSION_MINOR := 0
INV_VERSION_PATCH := 0
INV_VERSION_SUFFIX := -simple
$(info InvenSense MA-Lite version $(INV_VERSION_MAJOR).$(INV_VERSION_MINOR).$(INV_VERSION_PATCH)$(INV_VERSION_SUFFIX))
INV_CFLAGS += -DINV_VERSION_MAJOR=$(INV_VERSION_MAJOR)
INV_CFLAGS += -DINV_VERSION_MINOR=$(INV_VERSION_MINOR)
INV_CFLAGS += -DINV_VERSION_PATCH=$(INV_VERSION_PATCH)
INV_CFLAGS += -DINV_VERSION_SUFFIX=\"$(INV_VERSION_SUFFIX)\"

INV_SRCS += SensorsMain.cpp
INV_SRCS += SensorBase.cpp
INV_SRCS += MPLSensor.cpp
INV_SRCS += MPLSupport.cpp
INV_SRCS += CompassSensor.IIO.primary.cpp
INV_SRCS += PressureSensor.IIO.primary.cpp

INV_INCLUDES += tools
INV_SRCS += tools/inv_sysfs_utils.c
INV_SRCS += tools/inv_iio_buffer.c
INV_SRCS += tools/ml_sysfs_helper.c
