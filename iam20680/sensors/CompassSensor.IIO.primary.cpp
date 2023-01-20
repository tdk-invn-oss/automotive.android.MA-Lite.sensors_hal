/*
 * Copyright (C) 2014-2017 The Android Open Source Project
 * Copyright (C) 2017-2020 InvenSense, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_NDEBUG 0

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "CompassSensor.IIO.primary.h"
#include "InvnSensors.h"
#include "MPLSupport.h"
#include "Log.h"
#include "sensor_params.h"
#include "ml_sysfs_helper.h"
#include "inv_iio_buffer.h"

#define COMPASS_VENDOR(_chip)           COMPASS_##_chip##_VENDOR
#define COMPASS_RANGE(_chip)            COMPASS_##_chip##_RANGE
#define COMPASS_RESOLUTION(_chip)       COMPASS_##_chip##_RESOLUTION
#define COMPASS_POWER(_chip)            COMPASS_##_chip##_POWER
#define COMPASS_MINDELAY(_chip)         COMPASS_##_chip##_MINDELAY
#define COMPASS_MAXDELAY(_chip)         COMPASS_##_chip##_MAXDELAY

static const struct sensor_t sSensorList[] = {
    {
        .name = "Invensense Magnetometer",
        .vendor = "Invensense",
        .version = 1,
        .handle = SENSORS_MAGNETIC_FIELD_HANDLE,
        .type = SENSOR_TYPE_MAGNETIC_FIELD,
        .maxRange = 10240.0f,
        .resolution = 1.0f,
        .power = 0.5f,
        .minDelay = 20000,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = SENSOR_STRING_TYPE_MAGNETIC_FIELD,
	.requiredPermission = "",
	.maxDelay = 250000,
        .flags = SENSOR_FLAG_CONTINUOUS_MODE | SENSOR_FLAG_ADDITIONAL_INFO,
	.reserved = {},
    },
};

static int8_t defaultOrientation[9] = {
    0, 1, 0, 1, 0, 0, 0, 0, -1,
};

/******************************************************************************/

CompassSensor::CompassSensor() :
        SensorBase(NULL, NULL),
        dev_full_name{0},
        mEnable(-1),
        mDelay(0),
        mTimestamp(0)
{
    FILE *fptr;

    VFUNC_LOG;

    find_name_by_sensor_type(COMPASS_ON_PRIMARY, "iio:device", dev_full_name);
    if (strcmp(dev_full_name, "") == 0) {
        LOGI("Compass HAL:No sensor found\n");
        return;
    }

    if (inv_init_sysfs_attributes()) {
        LOGE("Error Instantiating Compass\n");
        return;
    }

    // set default orientation
    memcpy(mCompassOrientation, defaultOrientation, sizeof(mCompassOrientation));

    enable(ID_M, 0);

    LOGI("HAL:compass chip %s", dev_full_name);
    enable_iio_sysfs();

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:cat %s (%" PRId64 ")",
            compassSysFs[COMPASS_ORIENT], getTimestamp());
    fptr = fopen(compassSysFs[COMPASS_ORIENT], "r");
    if (fptr != NULL) {
        int om[9];
        if (fscanf(fptr, "%d, %d, %d; %d, %d, %d; %d, %d, %d",
               &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
               &om[6], &om[7], &om[8]) != 9) {
            LOGE("HAL:could not read compass mounting matrix\n"
                 "will use default mounting matrix: "
                 "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                    mCompassOrientation[0],
                    mCompassOrientation[1],
                    mCompassOrientation[2],
                    mCompassOrientation[3],
                    mCompassOrientation[4],
                    mCompassOrientation[5],
                    mCompassOrientation[6],
                    mCompassOrientation[7],
                    mCompassOrientation[8]);
        } else {
            LOGV_IF(PROCESS_VERBOSE,
                    "HAL:compass mounting matrix: "
                    "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                    om[0], om[1], om[2], om[3], om[4], om[5], om[6], om[7], om[8]);
            mCompassOrientation[0] = om[0];
            mCompassOrientation[1] = om[1];
            mCompassOrientation[2] = om[2];
            mCompassOrientation[3] = om[3];
            mCompassOrientation[4] = om[4];
            mCompassOrientation[5] = om[5];
            mCompassOrientation[6] = om[6];
            mCompassOrientation[7] = om[7];
            mCompassOrientation[8] = om[8];
        }
        fclose(fptr);
    }
}

void CompassSensor::enable_iio_sysfs()
{
    VFUNC_LOG;

    char iio_device_node[MAX_CHIP_ID_LEN];
    const char* compass = dev_full_name;
    size_t size, align, addr;
    int ret = 0;

    // enable 3-axis mag + timestamp into buffer
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            1, compassSysFs[COMPASS_X_ENABLE], getTimestamp());
    ret = write_sysfs_int(compassSysFs[COMPASS_X_ENABLE], 1);
    LOGE_IF(ret != 0, "HAL:sysfs error enabling iio buffer in_magn_x");
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            1, compassSysFs[COMPASS_Y_ENABLE], getTimestamp());
    ret = write_sysfs_int(compassSysFs[COMPASS_Y_ENABLE], 1);
    LOGE_IF(ret != 0, "HAL:sysfs error enabling iio buffer in_magn_y");
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            1, compassSysFs[COMPASS_Z_ENABLE], getTimestamp());
    ret = write_sysfs_int(compassSysFs[COMPASS_Z_ENABLE], 1);
    LOGE_IF(ret != 0, "HAL:sysfs error enabling iio buffer in_magn_z");
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            1, compassSysFs[TIMESTAMP_ENABLE], getTimestamp());
    ret = write_sysfs_int(compassSysFs[TIMESTAMP_ENABLE], 1);
    LOGE_IF(ret != 0, "HAL:sysfs error enabling iio buffer in_timestamp");

    // scan compass buffer
    inv_iio_buffer_scan_channel(compassSysFs[COMPASS_X_ENABLE],
                                compassSysFs[COMPASS_X_INDEX],
                                compassSysFs[COMPASS_X_TYPE],
                                compassSysFs[COMPASS_X_OFFSET],
                                compassSysFs[COMPASS_X_SCALE],
                                &compassBufferScan.channels[MAG_X_CHANNEL]);
    inv_iio_buffer_scan_channel(compassSysFs[COMPASS_Y_ENABLE],
                                compassSysFs[COMPASS_Y_INDEX],
                                compassSysFs[COMPASS_Y_TYPE],
                                compassSysFs[COMPASS_Y_OFFSET],
                                compassSysFs[COMPASS_Y_SCALE],
                                &compassBufferScan.channels[MAG_Y_CHANNEL]);
    inv_iio_buffer_scan_channel(compassSysFs[COMPASS_Z_ENABLE],
                                compassSysFs[COMPASS_Z_INDEX],
                                compassSysFs[COMPASS_Z_TYPE],
                                compassSysFs[COMPASS_Z_OFFSET],
                                compassSysFs[COMPASS_Z_SCALE],
                                &compassBufferScan.channels[MAG_Z_CHANNEL]);
    inv_iio_buffer_scan_channel(compassSysFs[TIMESTAMP_ENABLE],
                                compassSysFs[TIMESTAMP_INDEX],
                                compassSysFs[TIMESTAMP_TYPE],
                                compassSysFs[TIMESTAMP_OFFSET],
                                compassSysFs[TIMESTAMP_SCALE],
                                &compassBufferScan.channels[TIMESTAMP_CHANNEL]);

    // compute buffer size and alignment
    size = 0;
    align = 0;
    for (size_t i = 0; i < ARRAY_SIZE(compassBufferScan.channels); ++i) {
        if (compassBufferScan.channels[i].is_enabled) {
            size += compassBufferScan.channels[i].size;
            if (compassBufferScan.channels[i].size > align) {
                align = compassBufferScan.channels[i].size;
            }
        }
    }
    // must be multiple of alignment
    if (size % align != 0) {
        size += align - (size % align);
    }
    compassBufferScan.size = size;

    // compute addresses
    addr = 0;
    for (size_t idx = 0; idx < CHANNELS_NB; ++idx) {
        for (size_t i = 0; i < ARRAY_SIZE(compassBufferScan.channels); ++i) {
            if (compassBufferScan.channels[i].index == idx) {
                if (compassBufferScan.channels[i].is_enabled) {
                    size = compassBufferScan.channels[i].size;
                    // handle address alignment
                    if (addr % size != 0) {
                        addr += size - (addr % size);
                    }
                    compassBufferScan.addresses[i] = addr;
                    addr += size;
                } else {
                    compassBufferScan.addresses[i] = -1;
                }
            }
        }
    }

    // print buffer scan
    LOGV_IF(PROCESS_VERBOSE, "HAL:compass buffer scan size: %zu", compassBufferScan.size);
    for (size_t i = 0; i < CHANNELS_NB; ++i) {
        LOGV_IF(PROCESS_VERBOSE, "HAL:compass buffer channel #%zu", i);
        LOGV_IF(PROCESS_VERBOSE, "\taddress: %zd", compassBufferScan.addresses[i]);
        LOGV_IF(PROCESS_VERBOSE, "\tis_enabled: %d", compassBufferScan.channels[i].is_enabled);
        LOGV_IF(PROCESS_VERBOSE, "\tindex: %u", compassBufferScan.channels[i].index);
        LOGV_IF(PROCESS_VERBOSE, "\tsize: %zu", compassBufferScan.channels[i].size);
        LOGV_IF(PROCESS_VERBOSE, "\tbits: %zu", compassBufferScan.channels[i].bits);
        LOGV_IF(PROCESS_VERBOSE, "\tshift: %zu", compassBufferScan.channels[i].shift);
        LOGV_IF(PROCESS_VERBOSE, "\tis_signed: %d", compassBufferScan.channels[i].is_signed);
        LOGV_IF(PROCESS_VERBOSE, "\tis_be: %d", compassBufferScan.channels[i].is_be);
        LOGV_IF(PROCESS_VERBOSE, "\toffset: %.6f", compassBufferScan.channels[i].offset);
        LOGV_IF(PROCESS_VERBOSE, "\tscale: %.6f", compassBufferScan.channels[i].scale);
    }

    // set buffer length
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            IIO_BUFFER_LENGTH, compassSysFs[BUFFER_LENGTH], getTimestamp());
    ret = write_sysfs_int(compassSysFs[BUFFER_LENGTH], IIO_BUFFER_LENGTH);
    LOGE_IF(ret != 0, "HAL:sysfs error setting magn buffer length");

    snprintf(iio_device_node, sizeof(iio_device_node), "/dev/iio:device%d",
             find_type_by_name(compass, "iio:device"));
    dev_fd = open(iio_device_node, O_RDONLY);
    int res = errno;
    if (dev_fd < 0) {
        LOGE("HAL:could not open '%s' iio device node in path '%s' - "
             "error '%s' (%d)",
             compass, iio_device_node, strerror(res), res);
    } else {
        LOGV_IF(PROCESS_VERBOSE,
                "HAL:iio %s, fd opened : %d", compass, dev_fd);
    }
}

CompassSensor::~CompassSensor()
{
    VFUNC_LOG;

    for (int i = 0; i < SYSFS_ATTR_NB; ++i) {
        free(compassSysFs[i]);
    }
}

int CompassSensor::isSensorPresent()
{
    VFUNC_LOG;

    if (strcmp(dev_full_name, "") == 0)
        return 0;
    else
        return 1;
}

int CompassSensor::populateSensorList(struct sensor_t *list, int len)
{
    int currentSize = sizeof(sSensorList) / sizeof(sSensorList[0]);
    if (len < currentSize) {
        LOGE("Pressure HAL:sensor list too small, len=%d", len);
        return 0;
    }
    memcpy(list, sSensorList, sizeof(*list) * currentSize);
    for (int i = 0; i < currentSize; ++i) {
        this->fillList(&list[i]);
    }

    return currentSize;
}

/**
 *  @brief        This function will enable/disable sensor.
 *  @param[in]    handle
 *                  which sensor to enable/disable.
 *  @param[in]    en
 *                  en=1, enable;
 *                  en=0, disable
 *  @return       if the operation is successful.
 */
int CompassSensor::enable(int32_t handle, int en)
{
    VFUNC_LOG;

    int val = en ? 1 : 0;
    int res;

    (void)handle;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            val, compassSysFs[BUFFER_ENABLE], getTimestamp());
    res = write_sysfs_int(compassSysFs[BUFFER_ENABLE], val);
    if (res) {
        LOGE("HAL:compass enable error %d", res);
    } else {
        mEnable = val;
        mTimestamp = 0;
    }

    return res;
}

int CompassSensor::setDelay(int32_t handle, int64_t ns)
{
    VFUNC_LOG;
    double freq;
    int freq_int;
    int res;

    (void)handle;

    if (ns == 0)
        ns = mMaxDelay;

    if (ns < mMinDelay)
        ns = mMinDelay;
    if (ns > mMaxDelay)
        ns = mMaxDelay;

    freq = 1000000000.0 / ns;
    freq_int = (int)ceil(freq);

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            freq_int, compassSysFs[COMPASS_RATE], getTimestamp());
    res = write_sysfs_int(compassSysFs[COMPASS_RATE], freq_int);
    if (res) {
        LOGE("HAL:Compass error opening compass rate file");
    } else {
        mDelay = ns;
        mTimestamp = 0;
    }

    return res;
}

void CompassSensor::getOrientationMatrix(signed char *orient)
{
    VFUNC_LOG;
    memcpy(orient, mCompassOrientation, sizeof(mCompassOrientation));
}

/**
    @brief         This function is called by SensorsMain.cpp
                   to read sensor data from the driver.
    @param[out]    data      sensor data is stored in this variable. Scaled such that
                             1 uT = 2^16
    @para[in]      timestamp data's timestamp
    @return        1, if 1   sample read, 0, if not, negative if error
 */
int CompassSensor::readSample(int *data, int64_t *timestamp) {
    VFUNC_LOG;

    char *rdata = mIIOBuffer;
    int64_t raw;
    double sample;
    struct inv_iio_buffer_channel *channel;
    ssize_t address;
#ifdef INV_HIFI_SUPPORT
    const int64_t delay_min = mDelay * 98LL / 100LL;
    const int64_t delay_max = mDelay * 102LL / 100LL;
    int64_t delta;
#endif

    ssize_t size = read(dev_fd, rdata, compassBufferScan.size);
    if (size < 0) {
        return -errno;
    }

    if (mEnable) {
        /* fill mag sample */
        for (int i = MAG_X_CHANNEL; i <= MAG_Z_CHANNEL; ++i) {
            channel = &compassBufferScan.channels[i];
            address = compassBufferScan.addresses[i];
            if (!channel->is_enabled) {
                data[i - MAG_X_CHANNEL] = 0;
            } else {
                raw = inv_iio_buffer_channel_get_data(channel, &rdata[address]);
                // apply offset + scale
                sample = inv_iio_buffer_convert_data(channel, raw);
                // sample is Gauss = 100uT, scale is 2^16 for 1 uT */
                data[i - MAG_X_CHANNEL] = sample * 100.0 * (1 << 16);
            }
        }
        /* fill timestamp sample */
        channel = &compassBufferScan.channels[TIMESTAMP_CHANNEL];
        address = compassBufferScan.addresses[TIMESTAMP_CHANNEL];
        if (!channel->is_enabled) {
            *timestamp = 0;
        } else {
            raw = inv_iio_buffer_channel_get_data(channel, &rdata[address]);
#ifdef INV_HIFI_SUPPORT
            if (mTimestamp != 0) {
                delta = raw - mTimestamp;
                if (delta > delay_max) {
                    LOGV_IF(ENG_VERBOSE, "HAL:compass: delta timestamp truncated from %" PRId64 " to %" PRId64, delta, delay_max);
                    delta = delay_max;
                } else if (delta < delay_min) {
                    LOGV_IF(ENG_VERBOSE, "HAL:compass: delta timestamp truncated from %" PRId64 " to %" PRId64, delta, delay_min);
                    delta = delay_min;
                }
                mTimestamp += delta;
            } else
#endif
            {
                mTimestamp = raw;
            }
            *timestamp = mTimestamp;
        }
    }

    return mEnable;
}

void CompassSensor::fillList(struct sensor_t *list)
{
    VFUNC_LOG;

    list->maxRange = COMPASS_RANGE(AKM9915);
    list->resolution = COMPASS_RESOLUTION(AKM9915);
    list->power = COMPASS_POWER(AKM9915);
    list->minDelay = COMPASS_MINDELAY(AKM9915);
    list->fifoReservedEventCount = 0;
    list->fifoMaxEventCount = 0;
    list->maxDelay = COMPASS_MAXDELAY(AKM9915);

    // min delay truncated to 50Hz (20000us), sufficient for HiFi and can handle timer drivers
    if (list->minDelay < 20000)
        list->minDelay = 20000;

    mMinDelay = (int64_t)list->minDelay * 1000LL;
    mMaxDelay = (int64_t)list->maxDelay * 1000LL;
}

int CompassSensor::inv_init_sysfs_attributes(void)
{
    VFUNC_LOG;

    char *sysfs_path = NULL;
    int ret;
    const char* compass = dev_full_name;

    // clear all pointers
    memset(&compassSysFs, 0, sizeof(compassSysFs));

    // get proper (in absolute/relative) IIO path & build sysfs paths
    ret = asprintf(&sysfs_path, "/sys/bus/iio/devices/iio:device%d",
                   find_type_by_name(compass, "iio:device"));
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }

    // fill sysfs attributes
    ret = asprintf(&compassSysFs[BUFFER_ENABLE], "%s/buffer/enable", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[BUFFER_LENGTH], "%s/buffer/length", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_X_ENABLE], "%s/scan_elements/in_magn_x_en", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_X_INDEX], "%s/scan_elements/in_magn_x_index", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_X_TYPE], "%s/scan_elements/in_magn_x_type", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Y_ENABLE], "%s/scan_elements/in_magn_y_en", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Y_INDEX], "%s/scan_elements/in_magn_y_index", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Y_TYPE], "%s/scan_elements/in_magn_y_type", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Z_ENABLE], "%s/scan_elements/in_magn_z_en", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Z_INDEX], "%s/scan_elements/in_magn_z_index", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Z_TYPE], "%s/scan_elements/in_magn_z_type", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[TIMESTAMP_ENABLE], "%s/scan_elements/in_timestamp_en", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[TIMESTAMP_INDEX], "%s/scan_elements/in_timestamp_index", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[TIMESTAMP_TYPE], "%s/scan_elements/in_timestamp_type", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_RATE], "%s/sampling_frequency", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_X_SCALE], "%s/in_magn_x_scale", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_X_OFFSET], "%s/in_magn_x_offset", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Y_SCALE], "%s/in_magn_y_scale", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Y_OFFSET], "%s/in_magn_y_offset", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Z_SCALE], "%s/in_magn_z_scale", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_Z_OFFSET], "%s/in_magn_z_offset", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[TIMESTAMP_SCALE], "%s/in_timestamp_scale", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[TIMESTAMP_OFFSET], "%s/in_timestamp_offset", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&compassSysFs[COMPASS_ORIENT], "%s/in_mount_matrix", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }

    free(sysfs_path);
    return 0;

error_free:
    for (int i = 0; i < SYSFS_ATTR_NB; ++i) {
        free(compassSysFs[i]);
    }
    free(sysfs_path);
    return ret;
}
