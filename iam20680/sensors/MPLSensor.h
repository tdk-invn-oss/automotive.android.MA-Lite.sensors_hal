/*
 * Copyright (C) 2016-2020 InvenSense, Inc.
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
#include <vector>
#include <string>

#include "InvnSensors.h"
#include "SensorBase.h"
#include "CompassSensor.IIO.primary.h"
#include "PressureSensor.IIO.primary.h"

/*
 * Version defines
 */
#ifdef INV_VERSION_MAJOR
#  define INV_SENSORS_HAL_VERSION_MAJOR     INV_VERSION_MAJOR
#else
#  define INV_SENSORS_HAL_VERSION_MAJOR     0
#endif
#ifdef INV_VERSION_MINOR
#  define INV_SENSORS_HAL_VERSION_MINOR     INV_VERSION_MINOR
#else
#  define INV_SENSORS_HAL_VERSION_MINOR     0
#endif
#ifdef INV_VERSION_PATCH
#  define INV_SENSORS_HAL_VERSION_PATCH     INV_VERSION_PATCH
#else
#  define INV_SENSORS_HAL_VERSION_PATCH     0
#endif
#ifdef INV_VERSION_SUFFIX
#  define INV_SENSORS_HAL_VERSION_SUFFIX    INV_VERSION_SUFFIX
#else
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
#define DATA_FORMAT_ACCEL_SZ        24
#define DATA_FORMAT_RAW_GYRO_SZ     24
#define DATA_FORMAT_EMPTY_MARKER_SZ 8
#define DATA_FORMAT_MARKER_SZ       8

// read max size from IIO
#define MAX_READ_SIZE               2048

// reserved the number of events for compass
#define COMPASS_SEN_EVENT_RESV_SZ   5

// reserved the number of events for pressure
#define PRESSURE_SEN_EVENT_RESV_SZ  5

#define NS_PER_SECOND               1000000000LL
#define NS_PER_SECOND_FLOAT         1000000000.f

class MPLSensor: public SensorBase
{
    typedef int (MPLSensor::*hfunc_t)(sensors_event_t*);

public:

    MPLSensor(CompassSensor *, PressureSensor *);
    virtual ~MPLSensor();

    virtual int readEvents(sensors_event_t *data, int count);
    virtual int getFd() const;
    virtual int enable(int32_t handle, int enabled);
    virtual int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
    virtual int flush(int handle);
    virtual int setDelay(int handle, int64_t period_ns) { (void)handle; (void)period_ns; return 0; }
    virtual void getOrientationMatrix(int8_t *orient) { (void)orient; }

    int readCompassEvents(sensors_event_t* s, int count);
    int readPressureEvents(sensors_event_t* s, int count);
    int readMpuEvents(sensors_event_t* s, int count);
    int getCompassFd() const;
    int getPollTime();
    int populateSensorList(struct sensor_t *list, int len);

protected:
	CompassSensor *mCompassSensor;
    PressureSensor *mPressureSensor;
	
    virtual void enable_iio_sysfs(void);
    virtual int inv_init_sysfs_attributes(void);
	struct sensor_t mCurrentSensorList[ID_NUMBER];

private:
    /* enable/disable sensors */
    int enableGyro(int en);
    int enableAccel(int en);
    int enableCompass(int en);
    int enablePressure(int en);

    /* set sample rate */
    void setGyroRate(uint64_t period_ns);
    void setAccelRate(uint64_t period_ns);
    void setMagRate(uint64_t period_ns);
    void setPressureRate(uint64_t period_ns);

    void setBatchTimeout(int64_t timeout_ns);
    void updateBatchTimeout(void);

    /* data handlers */
    int gyroHandler(sensors_event_t *data);
    int accelHandler(sensors_event_t *data);
    int compassHandler(sensors_event_t *data);
    int psHandler(sensors_event_t *data);
    int metaHandler(int sensor, sensors_event_t *data, int flags); // for flush complete
    int additionalInfoSensorPlacement(int handle, unsigned int seq, sensors_event_t* event);
    int additionalInfoInternalTemperature(int handle, unsigned int seq, sensors_event_t *event);
    int additionalInfoHandler(int handle, sensors_event_t *data, int count);
    int periodicAdditionalInfoHandler(int handle, sensors_event_t *data, int count);

    void getHandle(int32_t handle, int &what, std::string &sname);
    void inv_set_device_properties();
    void inv_get_sensors_orientation(void);
    void inv_write_sysfs(uint32_t delay, char *sysfs_rate);
    typedef int (*get_sensor_data_func)(float *values, int8_t *accuracy, int64_t *timestamp, int mode);
    void fillAccel(const char* accel, struct sensor_t *list);
    void fillGyro(const char* gyro, struct sensor_t *list);
    int inv_read_temperature(int *temperature, int64_t *ts);

    pthread_mutex_t mHALMutex;
    bool mChipDetected;
    char chip_ID[MAX_CHIP_ID_LEN];
    uint32_t mNumSensors;
    uint64_t mEnabled;
    int iio_fd;
    int chip_temperature_fd;
    char mIIOReadBuffer[MAX_READ_SIZE];
    int mIIOReadSize;
    int mPollTime;
    int64_t mDelays[TotalNumSensors];
    int64_t mEnabledTime[TotalNumSensors];
    uint64_t mBatchEnabled;
    int64_t mBatchTimeouts[TotalNumSensors];
    int64_t mBatchTimeoutInMs;
    char mSysfsPath[MAX_SYSFS_NAME_LEN];
    char *sysfs_names_ptr;
    std::vector<int> mFlushSensorEnabledVector;
    sensors_event_t mPendingEvents[TotalNumSensors];
    hfunc_t mHandlers[TotalNumSensors];
    std::vector<int> mAdditionalInfoEnabledVector;
    float mGyroLocation[3];
    float mAccelLocation[3];
    float mCompassLocation[3];
    float mPressureLocation[3];

    /* mount matrix */
    signed char mGyroOrientationMatrix[9];
    signed char mAccelOrientationMatrix[9];
    signed char mCompassOrientationMatrix[9];

    /* sensor data */
    int mCachedGyroData[3];
    int mCachedAccelData[3];
    int mCachedCompassData[3];
    int mCachedPressureData;

    /* timestamp */
    int64_t mGyroSensorTimestamp;
    int64_t mAccelSensorTimestamp;
    int64_t mCompassTimestamp;
    int64_t mPressureTimestamp;
    int64_t mChipTemperatureTimestamp[TotalNumSensors];
    int64_t mGyroSensorPrevTimestamp;
    int64_t mAccelSensorPrevTimestamp;
    int64_t mCompassPrevTimestamp;
    int64_t mPressurePrevTimestamp;

    /* fsr */
    int mGyroFsrDps;
    int mAccelFsrGee;

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
       char *scan_el_en;
       char *scan_el_index;
       char *scan_el_type;

       char *buffer_length;
       char *in_accel_x_offset;
       char *in_accel_y_offset;
       char *in_accel_z_offset;
       char *in_gyro_x_offset;
       char *in_gyro_y_offset;
       char *in_gyro_z_offset;
       char *batchmode_timeout;
       char *flush_batch;
       char *high_res_mode;
       char *chip_temperature;
   } mpu;
};

extern "C" {
    void setCallbackObject(MPLSensor*);
    MPLSensor *getCallbackObject();
}

#endif  // ANDROID_MPL_SENSOR_H

