# Android Simple Sensors HAL

Simple Android Sensors HAL for Android integration without any algorithm.

## Summary

This repository provides a simple Android Sensors HAL for integrating InvenSense chips inside Android system. It doesn't provides any calibration or fusion algorithms inside.

Beware that this integration is not compliant with Android CDD because of the lack of calibration algorithms for gyroscope and magnetometer.

## Content

The `sensors/` folder contains the Android Simple Sensors HAL 1.0.

The `subhalv2_1/` folder contains an Android SubHal v2.1 wrapper loading the Sensors HAL 1.0 for implementing SubHal v2.1 interface.

## Android integration

Sensors HAL 1.0 integration, compatible with all Android <= 13:
```
# InvenSense Sensors HAL
PRODUCT_PACKAGES += \
    sensors.invensense \
    android.hardware.sensors@1.0-impl \
    android.hardware.sensors@1.0-service

PRODUCT_PROPERTY_OVERRIDES += ro.hardware.sensors=invensense
```

Sensors Multi-HAL 2.1 integration, compatible with Android 11, 12, 13:
```
# InvenSense Sensors HAL
PRODUCT_PACKAGES += \
    sensors.invensense \
    android.hardware.sensors@2.1-multihal-invensense \
    android.hardware.sensors@2.1-service.multihal

PRODUCT_PROPERTY_OVERRIDES += ro.hardware.sensors=invensense

PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/subhalv2_1/hals.conf:$(TARGET_COPY_OUT_VENDOR)/etc/sensors/hals.conf
```

Sensors Multi-HAL AIDL integration, compatible with Android >= 13:
```
# InvenSense Sensors HAL
PRODUCT_PACKAGES += \
    sensors.invensense \
    android.hardware.sensors@2.1-multihal-invensense \
    android.hardware.sensors-service.multihal

PRODUCT_PROPERTY_OVERRIDES += ro.hardware.sensors=invensense

PRODUCT_COPY_FILES += \
    $(LOCAL_PATH)/subhalv2_1/hals.conf:$(TARGET_COPY_OUT_VENDOR)/etc/sensors/hals.conf
```
