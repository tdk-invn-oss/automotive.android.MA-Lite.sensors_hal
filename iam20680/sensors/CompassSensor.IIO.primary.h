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

#ifndef COMPASS_SENSOR_H
#define COMPASS_SENSOR_H

#include <stdlib.h>
#include <stdint.h>

#include "InvnSensors.h"
#include "SensorBase.h"

#include "inv_iio_buffer.h"

#define MAX_CHIP_ID_LEN (20)
#define COMPASS_ON_PRIMARY "in_magn_x_raw"

#define COMPASS_MAX_SYSFS_ATTRB         (int)(sizeof(struct sysfs_attrbs) / sizeof(char*))


class CompassSensor : public SensorBase {

public:
    CompassSensor();
    virtual ~CompassSensor();

    virtual int enable(int32_t handle, int enabled);
    virtual int batch(int handle, int flags, int64_t period_ns, int64_t timeout) { (void)handle; (void)flags; (void)period_ns; (void)timeout; return 0; }
    virtual int flush(int handle) {(void)handle; return 0;}
    virtual int setDelay(int32_t handle, int64_t ns);
    // unnecessary for MPL
    virtual int readEvents(sensors_event_t *data, int count)
        { (void)data; (void)count; return 0; }

    int readSample(int *data, int64_t *timestamp);
    int providesCalibration() { return 0; }
    void getOrientationMatrix(signed char *orient);
    int getAccuracy() { return 0; }
    int isSensorPresent(void);
    int populateSensorList(struct sensor_t *list, int len);
    void fillList(struct sensor_t *list);

private:
    struct sysfs_attrbs {
       char *buffer_enable;
       char *buffer_length;
       char *compass_x_enable;
       char *compass_x_index;
       char *compass_x_type;
       char *compass_y_enable;
       char *compass_y_index;
       char *compass_y_type;
       char *compass_z_enable;
       char *compass_z_index;
       char *compass_z_type;
       char *timestamp_enable;
       char *timestamp_index;
       char *timestamp_type;
       char *compass_rate;
       char *compass_x_scale;
       char *compass_x_offset;
       char *compass_y_scale;
       char *compass_y_offset;
       char *compass_z_scale;
       char *compass_z_offset;
       char *timestamp_scale;
       char *timestamp_offset;
       char *compass_orient;
    } compassSysFs;

    char dev_full_name[MAX_CHIP_ID_LEN];

    enum scan_elements {
        MAG_X_CHANNEL,
        MAG_Y_CHANNEL,
        MAG_Z_CHANNEL,
        TIMESTAMP_CHANNEL,
        CHANNELS_NB,
    };
    struct buffer_scan {
        struct inv_iio_buffer_channel channels[CHANNELS_NB];
        ssize_t addresses[CHANNELS_NB];
        size_t size;
    } compassBufferScan;

    // implementation specific
    signed char mCompassOrientation[9];
    int mEnable;
    int64_t mDelay;
    int64_t mMinDelay;
    int64_t mMaxDelay;
    int64_t mTimestamp;
    char mIIOBuffer[CHANNELS_NB * 8 * IIO_BUFFER_LENGTH];

    void enable_iio_sysfs(void);
    int inv_init_sysfs_attributes(void);
};

/*****************************************************************************/

#endif  // COMPASS_SENSOR_PRIMARY_H
