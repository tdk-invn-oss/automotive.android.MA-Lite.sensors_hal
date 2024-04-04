/*
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

#ifndef PRESSURE_SENSOR_PRIMARY_H
#define PRESSURE_SENSOR_PRIMARY_H

#include <stdlib.h>
#include <stdint.h>

#include "InvnSensors.h"
#include "SensorBase.h"

#include "inv_iio_buffer.h"

#define MAX_CHIP_ID_LEN (20)
#define PRESSURE_ON_PRIMARY "in_pressure_raw"

//PRESSURE_ID_ICP101XX
// low noise mode
#define ICP101XX_MEASURE_MODE           3
#define PRESSURE_ICP101XX_RANGE         (1100.0f)
#define PRESSURE_ICP101XX_RESOLUTION    (0.0001f)
#define PRESSURE_ICP101XX_POWER         (0.005f)    // @1Hz
#define PRESSURE_ICP101XX_MINDELAY      (100000)
#define PRESSURE_ICP101XX_MAXDELAY      (1000000)
/* // low power mode
#define ICP101XX_MEASURE_MODE           1
#define PRESSURE_ICP101XX_RANGE         (1100.0f)
#define PRESSURE_ICP101XX_RESOLUTION    (0.0001f)
#define PRESSURE_ICP101XX_POWER         (0.0011f)   // @1Hz
#define PRESSURE_ICP101XX_MINDELAY      (5000)
#define PRESSURE_ICP101XX_MAXDELAY      (1000000)
*/

class PressureSensor : public SensorBase {

public:
    PressureSensor();
    virtual ~PressureSensor();

    virtual int readEvents(sensors_event_t* data, int count) { (void)data; (void)count; return 0; }
    virtual int readSample(int *data, int64_t *timestamp);
    virtual int enable(int32_t handle, int enabled);
    virtual int setDelay(int handle, int64_t period_ns);

    int isSensorPresent(void);
    int populateSensorList(struct sensor_t *list, int len);
    void fillList(struct sensor_t *list);
    int isIntegrated() { return (0); }

protected:
    virtual void enableIIOSysfs(void);
    virtual int inv_init_sysfs_attributes(void);
private:
    int readCalibData();
    void computeCalibMeasures(int32_t raw_p, int32_t raw_t, double *cal_p, double *cal_t);
    enum sysfs_attr {
        BUFFER_ENABLE,
        BUFFER_LENGTH,
        PRESSURE_ENABLE,
        PRESSURE_INDEX,
        PRESSURE_TYPE,
        TEMP_ENABLE,
        TEMP_INDEX,
        TEMP_TYPE,
        TIMESTAMP_ENABLE,
        TIMESTAMP_INDEX,
        TIMESTAMP_TYPE,
        SAMPLING_FREQUENCY,
        PRESSURE_SCALE,
        PRESSURE_OFFSET,
        TEMP_SCALE,
        TEMP_OFFSET,
        TIMESTAMP_SCALE,
        TIMESTAMP_OFFSET,
        MODE,
        CALIBDATA,
        SYSFS_ATTR_NB,
    };
    char *mSysFs[SYSFS_ATTR_NB];

    char dev_full_name[MAX_CHIP_ID_LEN];

    enum scan_elements {
        PRESSURE_CHANNEL,
        TEMP_CHANNEL,
        TIMESTAMP_CHANNEL,
        CHANNELS_NB,
    };
    struct buffer_scan {
        struct inv_iio_buffer_channel channels[CHANNELS_NB];
        ssize_t addresses[CHANNELS_NB];
        size_t size;
    } mBufferScan;

    // implementation specific
    int mEnable;
    int64_t mDelay;
    int64_t mMinDelay;
    int64_t mMaxDelay;

    const double mPaCalib[3] = {45000.0, 80000.0, 105000.0};
    const double mLUTLower = 3.5 * (1U << 20);
    const double mLUTUpper = 11.5 * (1U << 20);
    const double mQuadrFactor = 1.0 / 16777216.0;
    const double mOffsetFactor = 2048.0;
    int32_t mCalibData[4];

    char mIIOBuffer[CHANNELS_NB * 8 * IIO_BUFFER_LENGTH];
};

/*****************************************************************************/

#endif  // PRESSURE_SENSOR_PRIMARY_H
