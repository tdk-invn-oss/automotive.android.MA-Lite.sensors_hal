/*
 * Copyright (C) 2016-2018 InvenSense, Inc.
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

#ifndef ANDROID_MPL_SENSOR_H
#define ANDROID_MPL_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <poll.h>
#include <time.h>
#ifdef __ANDROID__
#include <utils/Vector.h>
#include <utils/String8.h>
#else
#include <vector>
#include <string>
#endif

#include "InvnSensors.h"
#include "SensorBase.h"
#include "CompassSensor.IIO.primary.h"

/*
 * Version defines
 */
#ifndef INV_SENSORS_HAL_VERSION_MAJOR
#  define INV_SENSORS_HAL_VERSION_MAJOR     0
#endif
#ifndef INV_SENSORS_HAL_VERSION_MINOR
#  define INV_SENSORS_HAL_VERSION_MINOR     0
#endif
#ifndef INV_SENSORS_HAL_VERSION_PATCH
#  define INV_SENSORS_HAL_VERSION_PATCH     0
#endif
#ifndef INV_SENSORS_HAL_VERSION_SUFFIX
#  define INV_SENSORS_HAL_VERSION_SUFFIX    "-dev"
#endif

/*****************************************************************************/
/* Sensors Enable/Disable Mask
 *****************************************************************************/
#define MAX_CHIP_ID_LEN             (20)

#define INV_THREE_AXIS_GYRO         (1LL << Gyro)
#define INV_THREE_AXIS_ACCEL        (1LL << Accelerometer)
#define INV_THREE_AXIS_COMPASS      (1LL << MagneticField)

// data header format used by kernel driver.
#define DATA_FORMAT_ACCEL           1
#define DATA_FORMAT_RAW_GYRO        2
#define DATA_FORMAT_EMPTY_MARKER    17
#define DATA_FORMAT_MARKER          18

// data size from kernel driver.
#define DATA_FORMAT_ACCEL_SZ        16
#define DATA_FORMAT_RAW_GYRO_SZ     16
#define DATA_FORMAT_EMPTY_MARKER_SZ 8
#define DATA_FORMAT_MARKER_SZ       8

// read max size from IIO
#define MAX_READ_SIZE               2048

// reserved number of events for compass
#define COMPASS_SEN_EVENT_RESV_SZ   1

#define NS_PER_SECOND               1000000000LL
#define NS_PER_SECOND_FLOAT         1000000000.f

class MPLSensor: public SensorBase
{
    typedef int (MPLSensor::*hfunc_t)(sensors_event_t*);

public:

    MPLSensor(CompassSensor *);
    virtual ~MPLSensor();

    virtual int readEvents(sensors_event_t *data, int count);
    virtual int readSample(int *data, int64_t *timestamp, int len) { (void)data; (void)timestamp; (void)len; return 0;}
    virtual int getFd() const;
    virtual int enable(int32_t handle, int enabled);
    virtual int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
    virtual int flush(int handle);
    virtual int setDelay(int handle, int64_t period_ns) { (void)handle; (void)period_ns; return 0; }
    virtual void getOrientationMatrix(int8_t *orient) { (void)orient; }

    int readCompassEvents(sensors_event_t* s, int count);
    int readMpuEvents(sensors_event_t* s, int count);
    int getCompassFd() const;
    int getPollTime();
    int populateSensorList(struct sensor_t *list, int len);

protected:
    virtual void enableIIOSysfs(void);
    virtual int initSysfsAttr(void);

private:
    /* enable/disable sensors */
    int enableGyro(int en);
    int enableAccel(int en);
    int enableCompass(int en);

    /* set sample rate */
    void setGyroRate(int64_t period_ns);
    void setAccelRate(int64_t period_ns);
    void setMagRate(int64_t period_ns);

#ifdef BATCH_MODE_SUPPORT
    void setBatchTimeout(int64_t timeout_ns);
    void updateBatchTimeout(void);
#endif

    /* data handlers */
    int rawGyroHandler(sensors_event_t *data);
    int accelHandler(sensors_event_t *data);
    int rawCompassHandler(sensors_event_t *data);
    int metaHandler(sensors_event_t *data, int flags); // for flush complete

#ifdef __ANDROID__
    void getHandle(int32_t handle, int &what, android::String8 &sname);
#else
    void getHandle(int32_t handle, int &what, std::string &sname);
#endif
    void setDeviceProperties();
    void getSensorsOrientation(void);
    void writeSysfs(int data, char *sysfs);
    void writeRateSysfs(int64_t period_ns, char *sysfs_rate);
    typedef int (*get_sensor_data_func)(float *values, int8_t *accuracy, int64_t *timestamp, int mode);

    CompassSensor *mCompassSensor;
    pthread_mutex_t mHALMutex;
    bool mChipDetected;
    char mChipId[MAX_CHIP_ID_LEN];
    uint32_t mNumSensors;
    uint64_t mEnabled;
    int mIIOfd;
    char mIIOReadBuffer[MAX_READ_SIZE];
    int mIIOReadSize;
    int mPollTime;
    int64_t mDelays[TotalNumSensors];
    int64_t mEnabledTime[TotalNumSensors];
#ifdef BATCH_MODE_SUPPORT
    uint64_t mBatchEnabled;
    int64_t mBatchTimeouts[TotalNumSensors];
    int64_t mBatchTimeoutInMs;
#endif
    char mSysfsPath[MAX_SYSFS_NAME_LEN];
    char *sysfs_names_ptr;
#ifdef __ANDROID__
    android::Vector<int> mFlushSensorEnabledVector;
#else
    std::vector<int> mFlushSensorEnabledVector;
#endif
    sensors_event_t mPendingEvents[TotalNumSensors];
    hfunc_t mHandlers[TotalNumSensors];

    /* mount matrix */
    signed char mGyroOrientationMatrix[9];
    signed char mAccelOrientationMatrix[9];
    signed char mCompassOrientationMatrix[9];

    /* sensor data */
    int mCachedGyroData[3];
    int mCachedAccelData[3];
    int mCachedCompassData[3];

    /* timestamp */
    int64_t mGyroSensorTimestamp;
    int64_t mAccelSensorTimestamp;
    int64_t mCompassTimestamp;
    int64_t mGyroSensorPrevTimestamp;
    int64_t mAccelSensorPrevTimestamp;
    int64_t mCompassPrevTimestamp;

    /* sysfs entries */
    struct sysfs_attrbs {
       char *chip_enable;
       char *self_test;
       char *gyro_enable;
       char *gyro_fsr;
       char *gyro_sf;
       char *gyro_orient;
       char *gyro_fifo_enable;
       char *gyro_rate;
       char *gyro_wake_fifo_enable;
       char *gyro_wake_rate;
       char *accel_enable;
       char *accel_fsr;
       char *accel_orient;
       char *accel_fifo_enable;
       char *accel_rate;
       char *accel_wake_fifo_enable;
       char *accel_wake_rate;
       char *in_timestamp_en;
       char *in_timestamp_index;
       char *in_timestamp_type;
       char *buffer_length;
       char *in_accel_x_offset;
       char *in_accel_y_offset;
       char *in_accel_z_offset;
       char *in_gyro_x_offset;
       char *in_gyro_y_offset;
       char *in_gyro_z_offset;
       char *batchmode_timeout;
       char *flush_batch;
   } mpu;
};

extern "C" {
    void setCallbackObject(MPLSensor*);
    MPLSensor *getCallbackObject();
}

#endif  // ANDROID_MPL_SENSOR_H

