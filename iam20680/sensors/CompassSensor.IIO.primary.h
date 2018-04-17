/*
 * Copyright (C) 2014-2017 The Android Open Source Project
 * Copyright (C) 2017-2018 InvenSense, Inc.
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

#ifndef COMPASS_SENSOR_PRIMARY_H
#define COMPASS_SENSOR_PRIMARY_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <poll.h>

#include "InvnSensors.h"
#include "SensorBase.h"
#include "inv_iio_buffer.h"

#define MAX_CHIP_ID_LEN (20)
#define COMPASS_ON_PRIMARY "in_magn_x_raw"

//COMPASS_ID_AK09911
#define COMPASS_AKM9911_RANGE           (4912.f)
#define COMPASS_AKM9911_RESOLUTION      (0.6f)
#define COMPASS_AKM9911_POWER           (2.4f)
#define COMPASS_AKM9911_MINDELAY        (10000)
#define COMPASS_AKM9911_MAXDELAY        (200000)

//COMPASS_ID_AK09916
#define COMPASS_AKM9916_RANGE           (9830.f)
#define COMPASS_AKM9916_RESOLUTION      (0.15f)
#define COMPASS_AKM9916_POWER           (10.f)
#define COMPASS_AKM9916_MINDELAY        (10000)
#define COMPASS_AKM9916_MAXDELAY        (200000)

class CompassSensor : public SensorBase {

public:
    CompassSensor();
    virtual ~CompassSensor();

    virtual int readEvents(sensors_event_t* data, int count) { (void)data; (void)count; return 0; }
    virtual int readSample(int *data, int64_t *timestamp, int len);
    virtual int getFd() const;
    virtual int enable(int32_t handle, int enabled);
    virtual int batch(int handle, int flags, int64_t period_ns, int64_t timeout) { (void)handle; (void)flags; (void)period_ns; (void)timeout; return 0; }
    virtual int flush(int handle) {(void)handle; return 0;}
    virtual int setDelay(int handle, int64_t period_ns);
    virtual void getOrientationMatrix(int8_t *orient);

    int providesCalibration() { return 0; }
    int getAccuracy() { return 0; }
    void fillList(struct sensor_t *list);

protected:
    virtual void enableIIOSysfs(void);
    virtual int initSysfsAttr(void);
private:
    struct sysfs_attrbs {
       char *buffer_enable;
       char *buffer_length;

       char *compass_enable;
       char *compass_index;
       char *compass_type;
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
       char *compass_scale;
       char *compass_offset;
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
        MAG_CHANNEL,
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
    int compass_fd;
    int mEnable;
    int64_t mDelay;
    int64_t mMinDelay;
    int64_t mMaxDelay;

    char mIIOBuffer[CHANNELS_NB * 8 * IIO_BUFFER_LENGTH];
};

/*****************************************************************************/

#endif  // COMPASS_SENSOR_PRIMARY_H
