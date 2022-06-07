/*
 *
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
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/select.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>

#include "PressureSensor.IIO.primary.h"
#include "InvnSensors.h"
#include "MPLSupport.h"
#include "Log.h"
#include "ml_sysfs_helper.h"
#include "inv_iio_buffer.h"
#define MAX_SYSFS_ATTRB         (sizeof(mSysFs) / sizeof(char*))

#define PRESSURE_VENDOR(_chip)           PRESSURE_##_chip##_VENDOR
#define PRESSURE_RANGE(_chip)            PRESSURE_##_chip##_RANGE
#define PRESSURE_RESOLUTION(_chip)       PRESSURE_##_chip##_RESOLUTION
#define PRESSURE_POWER(_chip)            PRESSURE_##_chip##_POWER
#define PRESSURE_MINDELAY(_chip)         PRESSURE_##_chip##_MINDELAY
#define PRESSURE_MAXDELAY(_chip)         PRESSURE_##_chip##_MAXDELAY

static const struct sensor_t sSensorList[] = {
     {
         "Invensense Pressure", "Invensense", 1,
         SENSORS_PRESSURE_HANDLE, SENSOR_TYPE_PRESSURE,
         1100.0f, 0.001f, 0.004f, 1000,
         0, 90, "android.sensor.pressure", "", 256000,
         SENSOR_FLAG_CONTINUOUS_MODE, {}
     },
};

/******************************************************************************/

PressureSensor::PressureSensor() :
        SensorBase(NULL, NULL),
        dev_full_name{0},
        mEnable(0),
        mDelay(0)
{
    VFUNC_LOG;

    int ret;

    find_name_by_sensor_type(PRESSURE_ON_PRIMARY, "iio:device", dev_full_name);
    if (strcmp(dev_full_name, "") == 0) {
        LOGI("Pressure HAL:No sensor found\n");
        return;
    }

    if (inv_init_sysfs_attributes()) {
        LOGE("Pressure HAL:Error Instantiating\n");
        strcpy(dev_full_name, "");
        return;
    }

    if (readCalibData()) {
        LOGE("Pressure HAL:Error reading calibration data\n");
    }

    // set measurement mode
    LOGV_IF(SYSFS_VERBOSE, "Pressure HAL:sysfs:echo %d > %s (%" PRId64 ")",
            ICP101XX_MEASURE_MODE, mSysFs.mode, getTimestamp());
    ret = write_sysfs_int(mSysFs.mode, ICP101XX_MEASURE_MODE);
    LOGE_IF(ret != 0, "Pressure HAL:sysfs error setting measurement mode");

    enable(ID_PS, 0);

    LOGI("Pressure HAL:Chip %s detected", dev_full_name);
    enableIIOSysfs();
}

int PressureSensor::readCalibData()
{
    VFUNC_LOG;

    FILE *attr;
    int ret;

    attr = fopen(mSysFs.calibdata, "r");
    if (attr == NULL)
        return -1;

    ret = fscanf(attr, "%d,%d,%d,%d\n",
                 &mCalibData[0], &mCalibData[1],
                 &mCalibData[2], &mCalibData[3]);
    if (ret == 4) {
        ret = 0;
    } else {
        ret = -1;
    }

    fclose(attr);
    return ret;
}

void PressureSensor::enableIIOSysfs()
{
    VFUNC_LOG;

    char iio_device_node[MAX_CHIP_ID_LEN];
    const char* chip = dev_full_name;
    size_t size;
    int ret = 0;

    // enable pressure + temperature + timestamp into buffer
    LOGV_IF(SYSFS_VERBOSE, "Pressure HAL:sysfs:echo %d > %s (%" PRId64 ")",
            1, mSysFs.pressure_enable, getTimestamp());
    ret = write_sysfs_int(mSysFs.pressure_enable, 1);
    LOGE_IF(ret != 0, "Pressure HAL:sysfs error enabling iio buffer in_pressure");
    LOGV_IF(SYSFS_VERBOSE, "Pressure HAL:sysfs:echo %d > %s (%" PRId64 ")",
            1, mSysFs.temp_enable, getTimestamp());
    ret = write_sysfs_int(mSysFs.temp_enable, 1);
    LOGE_IF(ret != 0, "Pressure HAL:sysfs error enabling iio buffer in_temp");
    LOGV_IF(SYSFS_VERBOSE, "Pressure HAL:sysfs:echo %d > %s (%" PRId64 ")",
            1, mSysFs.timestamp_enable, getTimestamp());
    ret = write_sysfs_int(mSysFs.timestamp_enable, 1);
    LOGE_IF(ret != 0, "Pressure HAL:sysfs error enabling iio buffer in_timestamp");

    // scan iio buffer
    inv_iio_buffer_scan_channel(mSysFs.pressure_enable,
                                mSysFs.pressure_index,
                                mSysFs.pressure_type,
                                mSysFs.pressure_offset,
                                mSysFs.pressure_scale,
                                &mBufferScan.channels[PRESSURE_CHANNEL]);
    inv_iio_buffer_scan_channel(mSysFs.temp_enable,
                                mSysFs.temp_index,
                                mSysFs.temp_type,
                                mSysFs.temp_offset,
                                mSysFs.temp_scale,
                                &mBufferScan.channels[TEMP_CHANNEL]);
    inv_iio_buffer_scan_channel(mSysFs.timestamp_enable,
                                mSysFs.timestamp_index,
                                mSysFs.timestamp_type,
                                mSysFs.timestamp_offset,
                                mSysFs.timestamp_scale,
                                &mBufferScan.channels[TIMESTAMP_CHANNEL]);

    // compute buffer size
    size = 0;
    for (size_t i = 0; i < ARRAY_SIZE(mBufferScan.channels); ++i) {
        if (mBufferScan.channels[i].is_enabled) {
            size += mBufferScan.channels[i].size;
        }
    }
    mBufferScan.size = size;

    // compute addresses
    size = 0;
    for (size_t idx = 0; idx < CHANNELS_NB; ++idx) {
        for (size_t i = 0; i < ARRAY_SIZE(mBufferScan.channels); ++i) {
            if (mBufferScan.channels[i].index == idx) {
                if (mBufferScan.channels[i].is_enabled) {
                    mBufferScan.addresses[i] = size;
                    size += mBufferScan.channels[i].size;
                } else {
                    mBufferScan.addresses[i] = -1;
                }
            }
        }
    }

    // print buffer scan
    LOGV_IF(PROCESS_VERBOSE, "Pressure HAL:buffer scan size: %zu", mBufferScan.size);
    for (size_t i = 0; i < CHANNELS_NB; ++i) {
        LOGV_IF(PROCESS_VERBOSE, "Pressure HAL:buffer channel #%zu", i);
        LOGV_IF(PROCESS_VERBOSE, "\taddress: %zd", mBufferScan.addresses[i]);
        LOGV_IF(PROCESS_VERBOSE, "\tis_enabled: %d", mBufferScan.channels[i].is_enabled);
        LOGV_IF(PROCESS_VERBOSE, "\tindex: %u", mBufferScan.channels[i].index);
        LOGV_IF(PROCESS_VERBOSE, "\tsize: %zu", mBufferScan.channels[i].size);
        LOGV_IF(PROCESS_VERBOSE, "\tbits: %zu", mBufferScan.channels[i].bits);
        LOGV_IF(PROCESS_VERBOSE, "\tshift: %zu", mBufferScan.channels[i].shift);
        LOGV_IF(PROCESS_VERBOSE, "\tis_signed: %d", mBufferScan.channels[i].is_signed);
        LOGV_IF(PROCESS_VERBOSE, "\tis_be: %d", mBufferScan.channels[i].is_be);
        LOGV_IF(PROCESS_VERBOSE, "\toffset: %.6f", mBufferScan.channels[i].offset);
        LOGV_IF(PROCESS_VERBOSE, "\tscale: %.6f", mBufferScan.channels[i].scale);
    }

    // set buffer length
    LOGV_IF(SYSFS_VERBOSE, "Pressure HAL:sysfs:echo %d > %s (%" PRId64 ")",
            IIO_BUFFER_LENGTH, mSysFs.buffer_length, getTimestamp());
    ret = write_sysfs_int(mSysFs.buffer_length, IIO_BUFFER_LENGTH);
    LOGE_IF(ret != 0, "Pressure HAL:sysfs error setting pressure buffer length");

    snprintf(iio_device_node, sizeof(iio_device_node), "/dev/iio:device%d",
             find_type_by_name(chip, "iio:device"));
    dev_fd = open(iio_device_node, O_RDONLY);
    int res = errno;
    if (dev_fd < 0) {
        LOGE("Pressure HAL:could not open '%s' iio device node in path '%s' - "
             "error '%s' (%d)",
             chip, iio_device_node, strerror(res), res);
    } else {
        LOGV_IF(PROCESS_VERBOSE,
                "Pressure HAL:iio %s, fd opened : %d", chip, dev_fd);
    }
}

PressureSensor::~PressureSensor()
{
    VFUNC_LOG;

    for (size_t i = 0; i < MAX_SYSFS_ATTRB; ++i) {
        char *attr = (char *)&mSysFs + i;
        free(attr);
    }
}

void PressureSensor::computeCalibMeasures(int32_t raw_p, int32_t raw_t, double *cal_p, double *cal_t)
{
    VFUNC_LOG;

    double t;
    double LUT[3];
    double A, B, C;

    // compute calibrated pressure value
    t = (double)raw_t - 32768.0;
    LUT[0] = mLUTLower + mCalibData[0] * t * t * mQuadrFactor;
    LUT[1] = mOffsetFactor * mCalibData[3] + mCalibData[1] * t * t * mQuadrFactor;
    LUT[2] = mLUTUpper + mCalibData[2] * t * t * mQuadrFactor;
    C = (LUT[0] * LUT[1] * (mPaCalib[0] - mPaCalib[1]) +
         LUT[1] * LUT[2] * (mPaCalib[1] - mPaCalib[2]) +
         LUT[2] * LUT[0] * (mPaCalib[2] - mPaCalib[0])) /
        (LUT[2] * (mPaCalib[0] - mPaCalib[1]) +
         LUT[0] * (mPaCalib[1] - mPaCalib[2]) +
         LUT[1] * (mPaCalib[2] - mPaCalib[0]));
    A = (mPaCalib[0] * LUT[0] - mPaCalib[1] * LUT[1] - (mPaCalib[1] - mPaCalib[0]) * C) / (LUT[0] - LUT[1]);
    B = (mPaCalib[0] - A) * (LUT[0] + C);
    *cal_p = A + B / (C + (double)raw_p);

    // compute calibrated temperature value
    *cal_t = -45.0 + (175.0 / (double)(1U << 16)) * (double)raw_t;
}

int PressureSensor::isSensorPresent()
{
    VFUNC_LOG;

    if (strcmp(dev_full_name, "") == 0)
        return 0;
    else
        return 1;
}

int PressureSensor::populateSensorList(struct sensor_t *list, int len)
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
int PressureSensor::enable(int32_t handle, int en)
{
    VFUNC_LOG;

    int val = en ? 1 : 0;
    int res;

    (void)handle;

    LOGV_IF(SYSFS_VERBOSE, "Pressure HAL:sysfs:echo %d > %s (%" PRId64 ")",
            val, mSysFs.buffer_enable, getTimestamp());
    res = write_sysfs_int(mSysFs.buffer_enable, val);
    if (res) {
        LOGE("Pressure HAL:enable error %d", res);
    } else {
        mEnable = val;
    }

    return res;
}

int PressureSensor::setDelay(int32_t handle, int64_t ns)
{
    VFUNC_LOG;
    FILE *file;
    unsigned freq;
    int res;

    (void)handle;

    if (ns == 0)
        ns = mMaxDelay;

    if (ns < mMinDelay)
        ns = mMinDelay;
    if (ns > mMaxDelay)
        ns = mMaxDelay;

    freq = 1000000000.0 / ns;

    LOGV_IF(SYSFS_VERBOSE, "Pressure HAL:sysfs:echo %u > %s (%" PRId64 ")",
            freq, mSysFs.sampling_frequency, getTimestamp());
    file = fopen(mSysFs.sampling_frequency, "w");
    if (file == NULL) {
        LOGE("Pressure HAL:error opening sampling_frequency file");
        return -1;
    }
    res = fprintf(file, "%u\n", freq);
    fclose(file);
    if (res >= 0) {
        mDelay = ns;
        res = 0;
    }

    return res;
}

/**
    @brief         This function is called by sensors_mpl.cpp
                   to read sensor data from the driver.
    @param[out]    data      sensor data is stored in this variable in Pa x 100.
    @para[in]      timestamp data's timestamp
    @return        1, if 1   sample read, 0, if not, negative if error
 */
int PressureSensor::readSample(int *data, int64_t *timestamp) {
    VFUNC_LOG;

    char *rdata = mIIOBuffer;
    int32_t rawP, rawT;
    double calP, calT;
    const struct inv_iio_buffer_channel *pressure_chan = &mBufferScan.channels[PRESSURE_CHANNEL];
    const struct inv_iio_buffer_channel *temp_chan = &mBufferScan.channels[TEMP_CHANNEL];
    const struct inv_iio_buffer_channel *ts_chan = &mBufferScan.channels[TIMESTAMP_CHANNEL];
    ssize_t pressure_addr = mBufferScan.addresses[PRESSURE_CHANNEL];
    ssize_t temp_addr = mBufferScan.addresses[TEMP_CHANNEL];
    ssize_t ts_addr = mBufferScan.addresses[TIMESTAMP_CHANNEL];

    ssize_t size = read(dev_fd, rdata, mBufferScan.size);
    if (size < 0) {
        return -errno;
    }

    if (mEnable) {
        // extract pressure and temperature data
        if (!pressure_chan->is_enabled) {
            rawP = 0;
        } else {
            rawP = inv_iio_buffer_channel_get_data(pressure_chan, &rdata[pressure_addr]);
        }
        // extract temperature data
        if (!temp_chan->is_enabled) {
            rawT = 0;
        } else {
            rawT = inv_iio_buffer_channel_get_data(temp_chan, &rdata[temp_addr]);
        }
        // compute compensated value
        computeCalibMeasures(rawP, rawT, &calP, &calT);
        data[0] = calP * 100.f; // value in Pa x 100
        // fill timestamp sample
        if (!ts_chan->is_enabled) {
            *timestamp = 0;
        } else {
            *timestamp = inv_iio_buffer_channel_get_data(ts_chan, &rdata[ts_addr]);
        }
        LOGV_IF(INPUT_DATA, "Pressure HAL:data : %f %f -- %" PRId64,
                calP, calT, *timestamp);
    }

    return mEnable;
}

void PressureSensor::fillList(struct sensor_t *list)
{
    VFUNC_LOG;

    list->maxRange = PRESSURE_RANGE(ICP101XX);
    list->resolution = PRESSURE_RESOLUTION(ICP101XX);
    list->power = PRESSURE_POWER(ICP101XX);
    list->minDelay = PRESSURE_MINDELAY(ICP101XX);
    list->fifoReservedEventCount = 0;
    list->fifoMaxEventCount = 0;
    list->maxDelay = PRESSURE_MAXDELAY(ICP101XX);

    mMinDelay = (int64_t)list->minDelay * 1000LL;
    mMaxDelay = (int64_t)list->maxDelay * 1000LL;
}

int PressureSensor::inv_init_sysfs_attributes(void)
{
    VFUNC_LOG;

    char *sysfs_path = NULL;
    int ret;
    const char* chip = dev_full_name;

    // clear all pointers
    memset(&mSysFs, 0, sizeof(mSysFs));

    // get proper (in absolute/relative) IIO path & build sysfs paths
    ret = asprintf(&sysfs_path, "/sys/bus/iio/devices/iio:device%d",
                   find_type_by_name(chip, "iio:device"));
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }

    // fill sysfs attributes
    ret = asprintf(&mSysFs.buffer_enable, "%s/buffer/enable", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.buffer_length, "%s/buffer/length", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }

    ret = asprintf(&mSysFs.pressure_enable, "%s/scan_elements/in_pressure_en", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.pressure_index, "%s/scan_elements/in_pressure_index", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.pressure_type, "%s/scan_elements/in_pressure_type", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.temp_enable, "%s/scan_elements/in_temp_en", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.temp_index, "%s/scan_elements/in_temp_index", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.temp_type, "%s/scan_elements/in_temp_type", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.timestamp_enable, "%s/scan_elements/in_timestamp_en", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.timestamp_index, "%s/scan_elements/in_timestamp_index", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.timestamp_type, "%s/scan_elements/in_timestamp_type", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.sampling_frequency, "%s/sampling_frequency", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.pressure_scale, "%s/in_pressure_scale", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.pressure_offset, "%s/in_pressure_offset", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.temp_scale, "%s/in_temp_scale", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.temp_offset, "%s/in_temp_offset", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.timestamp_scale, "%s/in_timestamp_scale", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.timestamp_offset, "%s/in_timestamp_offset", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.mode, "%s/mode", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }
    ret = asprintf(&mSysFs.calibdata, "%s/calibdata", sysfs_path);
    if (ret == -1) {
        ret = -ENOMEM;
        goto error_free;
    }

    free(sysfs_path);
    return 0;

error_free:
    for (size_t i = 0; i < MAX_SYSFS_ATTRB; ++i) {
        char *attr = (char *)&mSysFs + i;
        free(attr);
    }
    free(sysfs_path);
    return ret;
}
