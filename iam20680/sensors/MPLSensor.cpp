/*
 * Copyright (C) 2014-2017 InvenSense, Inc.
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
#include <errno.h>
#include <math.h>
#include <float.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <sys/select.h>
#include <sys/syscall.h>
#include <dlfcn.h>
#include <pthread.h>
#ifdef __ANDROID__
#include <utils/Vector.h>
#include <utils/String8.h>
#else
#include <vector>
#include <string>
#endif
#include <string.h>

#include "MPLSensor.h"
#include "MPLSupport.h"

#include "log.h"
#include "ml_sysfs_helper.h"
#include "ml_sensor_parsing.h"

#define MAX_SYSFS_ATTRB (sizeof(struct sysfs_attrbs) / sizeof(char*))

#define ACCEL_FSR       8.0f    // 8g
#define ACCEL_FSR_SYSFS 2       // 0:2g, 1:4g, 2:8g, 3:16g

/*******************************************************************************
 * MPLSensor class implementation
 ******************************************************************************/

static struct sensor_t sRawSensorList[] =
{
    {"Invensense Gyroscope Uncalibrated", "Invensense", 1,
     SENSORS_RAW_GYROSCOPE_HANDLE,
     SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, 2000.0f * M_PI / 180.0f, 2000.0f * M_PI / (180.0f * 32768.0f), 3.0f, 5000, 0, 0,
     "android.sensor.gyroscope_uncalibrated", "", 250000, SENSOR_FLAG_CONTINUOUS_MODE, {}},
    {"Invensense Accelerometer", "Invensense", 1,
     SENSORS_ACCELERATION_HANDLE,
     SENSOR_TYPE_ACCELEROMETER, GRAVITY_EARTH * ACCEL_FSR, GRAVITY_EARTH * ACCEL_FSR / 32768.0f, 0.4f, 5000, 0, 0,
     "android.sensor.accelerometer", "", 250000, SENSOR_FLAG_CONTINUOUS_MODE, {}},
#ifdef COMPASS_SUPPORT
    {"Invensense Magnetometer Uncalibrated", "Invensense", 1,
     SENSORS_RAW_MAGNETIC_FIELD_HANDLE,
     SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED, 10240.0f, 1.0f, 0.5f, 10000, 0, 0,
     "android.sensor.magnetic_field_uncalibrated", "", 250000, SENSOR_FLAG_CONTINUOUS_MODE, {}},
#endif
};

struct sensor_t *currentSensorList;

MPLSensor::MPLSensor(CompassSensor *compass) :
    mEnabled(0),
    mPollTime(-1),
    mGyroSensorPrevTimestamp(0),
    mAccelSensorPrevTimestamp(0),
    mCompassPrevTimestamp(0)
{

    VFUNC_LOG;

    int i;

    mCompassSensor = compass;

    LOGV_IF(PROCESS_VERBOSE,
            "HAL:MPLSensor constructor : NumSensors = %d", TotalNumSensors);

    pthread_mutex_init(&mHALMutex, NULL);
    memset(mGyroOrientationMatrix, 0, sizeof(mGyroOrientationMatrix));
    memset(mAccelOrientationMatrix, 0, sizeof(mAccelOrientationMatrix));
    memset(mCompassOrientationMatrix, 0, sizeof(mCompassOrientationMatrix));
#ifdef __ANDROID__
    mFlushSensorEnabledVector.setCapacity(TotalNumSensors);
#else
    mFlushSensorEnabledVector.resize(TotalNumSensors);
#endif
    memset(mEnabledTime, 0, sizeof(mEnabledTime));

    /* setup sysfs paths */
    initSysfsAttr();

    /* get chip name */
    if (inv_get_chip_name(mChipId) != INV_SUCCESS) {
        LOGE("HAL:ERR Failed to get chip ID\n");
        mChipDetected = false;
    } else {
        LOGI("HAL:Chip ID = %s\n", mChipId);
        mChipDetected = true;
    }

    /* print software version string */
    LOGI("HAL:InvenSense Sensors HAL version MA-%d.%d.%d%s\n",
         INV_SENSORS_HAL_VERSION_MAJOR, INV_SENSORS_HAL_VERSION_MINOR,
         INV_SENSORS_HAL_VERSION_PATCH, INV_SENSORS_HAL_VERSION_SUFFIX);

    /* enable iio */
    enableIIOSysfs();

    /* setup orientation matrix */
    setDeviceProperties();

    /* initialize sensor data */
    memset(mPendingEvents, 0, sizeof(mPendingEvents));
    mPendingEvents[RawGyro].version = sizeof(sensors_event_t);
    mPendingEvents[RawGyro].sensor = ID_RG;
    mPendingEvents[RawGyro].type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
    mPendingEvents[RawGyro].gyro.status = SENSOR_STATUS_UNRELIABLE;
    mPendingEvents[Accelerometer].version = sizeof(sensors_event_t);
    mPendingEvents[Accelerometer].sensor = ID_A;
    mPendingEvents[Accelerometer].type = SENSOR_TYPE_ACCELEROMETER;
    mPendingEvents[Accelerometer].acceleration.status
        = SENSOR_STATUS_UNRELIABLE;
    mPendingEvents[RawMagneticField].version = sizeof(sensors_event_t);
    mPendingEvents[RawMagneticField].sensor = ID_RM;
    mPendingEvents[RawMagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    mPendingEvents[RawMagneticField].magnetic.status =
        SENSOR_STATUS_UNRELIABLE;

    /* Event Handlers */
    mHandlers[RawGyro] = &MPLSensor::rawGyroHandler;
    mHandlers[Accelerometer] = &MPLSensor::accelHandler;
    mHandlers[RawMagneticField] = &MPLSensor::rawCompassHandler;

    /* initialize delays to reasonable values */
    for (i = 0; i < TotalNumSensors; i++) {
        mDelays[i] = NS_PER_SECOND;
    }

    /* disable all sensors */
    enableGyro(0);
    enableAccel(0);
    enableCompass(0);

    /* set accel FSR */
    writeSysfs(ACCEL_FSR_SYSFS, mpu.accel_fsr);
}

void MPLSensor::enableIIOSysfs(void)
{
    VFUNC_LOG;

    char iio_device_node[MAX_CHIP_ID_LEN];
    FILE *tempFp = NULL;
    int err;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo 1 > %s (%" PRId64 ")",
            mpu.in_timestamp_en, getTimestamp());
    tempFp = fopen(mpu.in_timestamp_en, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open timestamp enable");
    } else {
        err = fprintf(tempFp, "%d", 1);
        if (err < 0) {
            LOGE("HAL:could not write timestamp enable, %d", err);
        }
        err = fclose(tempFp);
        if (err) {
            LOGE("HAL:could not close write timestamp enable, %d", err);
        }
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            IIO_BUFFER_LENGTH, mpu.buffer_length, getTimestamp());
    tempFp = fopen(mpu.buffer_length, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open buffer length");
    } else {
        err = fprintf(tempFp, "%d", IIO_BUFFER_LENGTH);
        if (err < 0) {
            LOGE("HAL:could not write buffer length, %d", err);
        }
        err = fclose(tempFp);
        if (err) {
            LOGE("HAL:could not close write buffer length, %d", err);
        }
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            1, mpu.chip_enable, getTimestamp());
    tempFp = fopen(mpu.chip_enable, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open chip enable");
    } else {
        if ((err = fprintf(tempFp, "%d", 1)) < 0) {
            LOGE("HAL:could not write chip enable, %d", err);
         } else if ((err = fclose(tempFp)) < 0) {
            LOGE("HAL:could not close chip enable, %d", err);
        }
    }

    inv_get_iio_device_node(iio_device_node);
    mIIOfd = open(iio_device_node, O_RDONLY);
    if (mIIOfd < 0) {
        LOGE("HAL:could not open iio device node");
    } else {
        LOGV_IF(PROCESS_VERBOSE, "HAL:iio opened : %d", mIIOfd);
    }
}

void MPLSensor::setDeviceProperties(void)
{
    VFUNC_LOG;

    /* gyro/accel mount matrix */
    getSensorsOrientation();
    if (mCompassSensor) {
        /* compass mount matrix */
        mCompassSensor->getOrientationMatrix(mCompassOrientationMatrix);
    }
}

void MPLSensor::getSensorsOrientation(void)
{
    VFUNC_LOG;

    FILE *fptr;

    // get gyro orientation
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%" PRId64 ")", mpu.gyro_orient, getTimestamp());
    fptr = fopen(mpu.gyro_orient, "r");
    if (fptr != NULL) {
        int om[9];
        if (fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
                    &om[6], &om[7], &om[8]) < 0 || fclose(fptr) < 0) {
            LOGE("HAL:Could not read gyro mounting matrix");
        } else {
            LOGV_IF(PROCESS_VERBOSE,
                    "HAL:gyro mounting matrix: "
                    "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                    om[0], om[1], om[2], om[3], om[4], om[5], om[6], om[7], om[8]);

            mGyroOrientationMatrix[0] = om[0];
            mGyroOrientationMatrix[1] = om[1];
            mGyroOrientationMatrix[2] = om[2];
            mGyroOrientationMatrix[3] = om[3];
            mGyroOrientationMatrix[4] = om[4];
            mGyroOrientationMatrix[5] = om[5];
            mGyroOrientationMatrix[6] = om[6];
            mGyroOrientationMatrix[7] = om[7];
            mGyroOrientationMatrix[8] = om[8];
        }
    }

    // get accel orientation
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%" PRId64 ")", mpu.accel_orient, getTimestamp());
    fptr = fopen(mpu.accel_orient, "r");
    if (fptr != NULL) {
        int om[9];
        if (fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
                    &om[6], &om[7], &om[8]) < 0 || fclose(fptr) < 0) {
            LOGE("HAL:could not read accel mounting matrix");
        } else {
            LOGV_IF(PROCESS_VERBOSE,
                    "HAL:accel mounting matrix: "
                    "%+d %+d %+d %+d %+d %+d %+d %+d %+d",
                    om[0], om[1], om[2], om[3], om[4], om[5], om[6], om[7], om[8]);

            mAccelOrientationMatrix[0] = om[0];
            mAccelOrientationMatrix[1] = om[1];
            mAccelOrientationMatrix[2] = om[2];
            mAccelOrientationMatrix[3] = om[3];
            mAccelOrientationMatrix[4] = om[4];
            mAccelOrientationMatrix[5] = om[5];
            mAccelOrientationMatrix[6] = om[6];
            mAccelOrientationMatrix[7] = om[7];
            mAccelOrientationMatrix[8] = om[8];
        }
    }
}

MPLSensor::~MPLSensor()
{
    VFUNC_LOG;

    if (mIIOfd > 0)
        close(mIIOfd);
}

void MPLSensor::writeSysfs(int data, char *sysfs)
{
    int fd;
    int res;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            data, sysfs, getTimestamp());
    fd = open(sysfs, O_RDWR);
    if (fd < 0) {
        LOGE("HAL:%s failed to open sysfs", sysfs);
    } else {
        res = write_attribute_sensor(fd, data);
        if (res < 0) {
            LOGE("HAL:%s failed to write sysfs", sysfs);
        }
        close(fd);
    }
}

void MPLSensor::writeRateSysfs(int64_t period_ns, char *sysfs_rate)
{
    writeSysfs(NS_PER_SECOND_FLOAT / period_ns, sysfs_rate);
}

void MPLSensor::setGyroRate(int64_t period_ns)
{
    writeRateSysfs(period_ns, mpu.gyro_rate);
}

void MPLSensor::setAccelRate(int64_t period_ns)
{
    writeRateSysfs(period_ns, mpu.accel_rate);
}

void MPLSensor::setMagRate(int64_t period_ns)
{
    if (mCompassSensor)
        mCompassSensor->setDelay(ID_RM, period_ns);
}

int MPLSensor::enableGyro(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            en, mpu.gyro_fifo_enable, getTimestamp());
    res += write_sysfs_int(mpu.gyro_fifo_enable, en);

    return res;
}

int MPLSensor::enableAccel(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            en, mpu.accel_fifo_enable, getTimestamp());
    res += write_sysfs_int(mpu.accel_fifo_enable, en);

    return res;
}

int MPLSensor::enableCompass(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mCompassSensor)
        res = mCompassSensor->enable(ID_RM, en);

    return res;
}

int MPLSensor::enable(int32_t handle, int en)
{
    VFUNC_LOG;

#ifdef __ANDROID__
    android::String8 sname;
#else
    std::string sname;
#endif
    int what;
    int err = 0;

    /* exit if no chip is connected */
    if (!mChipDetected)
        return -EINVAL;

    getHandle(handle, what, sname);
    if (what < 0) {
        LOGV_IF(PROCESS_VERBOSE, "HAL:can't find handle %d",handle);
        return -EINVAL;
    }

    if (mEnabled == 0) {
        // reset buffer
        LOGV_IF(PROCESS_VERBOSE, "HAL:reset parsing buffer (size = %d)", inv_sensor_parsing_get_size());
        inv_sensor_parsing_reset();
    }

    LOGV_IF(PROCESS_VERBOSE, "HAL:handle = %d en = %d", handle, en);

    uint64_t newState = en ? 1 : 0;

    LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
#ifdef __ANDROID__
            sname.string(),
#else
            sname.c_str(),
#endif
            handle,
            ((mEnabled & (1LL << what)) ? "en" : "dis"),
            (((newState) << what) ? "en" : "dis"));
    LOGV_IF(PROCESS_VERBOSE, "HAL:%s sensor state change what=%d",
#ifdef __ANDROID__
            sname.string(),
#else
            sname.c_str(),
#endif
            what);

    if (((newState) << what) != (mEnabled & (1LL << what))) {
        uint64_t flags = newState;

        mEnabled &= ~(1LL << what);
        mEnabled |= (uint64_t(flags) << what);

        switch (what) {
            case RawGyro:
                enableGyro(en);
                break;
            case Accelerometer:
                enableAccel(en);
                break;
            case RawMagneticField:
                enableCompass(en);
                break;
        }
        if (en)
            mEnabledTime[what] = getTimestamp();
        else
            mEnabledTime[what] = 0;
    }
    return err;
}

/*  these handlers transform mpl data into one of the Android sensor types */
int MPLSensor::rawGyroHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    int update = 0;
    int data[3];
    int i;
    float scale = 1.f / 16.4f * 0.0174532925f; // 2000dps

    /* convert to body frame */
    for (i = 0; i < 3 ; i++) {
        data[i] = mCachedGyroData[0] * mGyroOrientationMatrix[i * 3] +
                  mCachedGyroData[1] * mGyroOrientationMatrix[i * 3 + 1] +
                  mCachedGyroData[2] * mGyroOrientationMatrix[i * 3 + 2];
    }

    for (i = 0; i < 3 ; i++) {
        s->uncalibrated_gyro.uncalib[i] = (float)data[i] * scale;
        s->uncalibrated_gyro.bias[i] = 0;
    }

    s->timestamp = mGyroSensorTimestamp;
    s->gyro.status = SENSOR_STATUS_UNRELIABLE;

    /* timestamp check */
    if ((mGyroSensorTimestamp > mGyroSensorPrevTimestamp) &&
        (mGyroSensorTimestamp > mEnabledTime[RawGyro])) {
        update = 1;
    }

    mGyroSensorPrevTimestamp = mGyroSensorTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:raw gyro data : %+f %+f %+f -- %" PRId64 " - %d",
        s->uncalibrated_gyro.uncalib[0], s->uncalibrated_gyro.uncalib[1], s->uncalibrated_gyro.uncalib[2],
        s->timestamp, update);

    return update;
}

int MPLSensor::accelHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    int update = 0;
    int data[3];
    int i;
    float scale = 1.f / (32768.0f / ACCEL_FSR) * 9.80665f;

    /* convert to body frame */
    for (i = 0; i < 3 ; i++) {
        data[i] = mCachedAccelData[0] * mAccelOrientationMatrix[i * 3] +
                  mCachedAccelData[1] * mAccelOrientationMatrix[i * 3 + 1] +
                  mCachedAccelData[2] * mAccelOrientationMatrix[i * 3 + 2];
    }
    for (i = 0; i < 3 ; i++) {
        s->acceleration.v[i] = (float)data[i] * scale;
    }
    s->timestamp = mAccelSensorTimestamp;
    s->acceleration.status = SENSOR_STATUS_UNRELIABLE;

    /*timestamp check */
    if ((mAccelSensorTimestamp > mAccelSensorPrevTimestamp) &&
        (mAccelSensorTimestamp > mEnabledTime[Accelerometer])) {
        update = 1;
    }

    mAccelSensorPrevTimestamp = mAccelSensorTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:accel data : %+f %+f %+f -- %" PRId64 " - %d",
        s->acceleration.v[0], s->acceleration.v[1], s->acceleration.v[2],
        s->timestamp, update);

    return update;
}

int MPLSensor::rawCompassHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    int update = 0;
    int data[3];
    int i;
    float scale = 1.f / (1 << 16); // 1uT for 2^16

    /* convert to body frame */
    for (i = 0; i < 3 ; i++) {
        data[i] = (mCachedCompassData[0]) * mCompassOrientationMatrix[i * 3] +
                  (mCachedCompassData[1]) * mCompassOrientationMatrix[i * 3 + 1] +
                  (mCachedCompassData[2]) * mCompassOrientationMatrix[i * 3 + 2];
    }

    for (i = 0; i < 3 ; i++) {
        s->uncalibrated_magnetic.uncalib[i] = (float)data[i] * scale;
        s->uncalibrated_magnetic.bias[i] = 0;
    }

    s->timestamp = mCompassTimestamp;
    s->magnetic.status = SENSOR_STATUS_UNRELIABLE;

    /* timestamp check */
    if ((mCompassTimestamp > mCompassPrevTimestamp) &&
        (mCompassTimestamp > mEnabledTime[RawMagneticField])) {
        update = 1;
    }

    mCompassPrevTimestamp = mCompassTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:raw compass data: %+f %+f %+f %d -- %" PRId64 " - %d",
        s->uncalibrated_magnetic.uncalib[0], s->uncalibrated_magnetic.uncalib[1], s->uncalibrated_magnetic.uncalib[2],
        s->magnetic.status, s->timestamp, update);

    return update;
}

int MPLSensor::metaHandler(sensors_event_t* s, int flags)
{
    VHANDLER_LOG;
    int update = 1;

    /* initalize SENSOR_TYPE_META_DATA */
    s->version = META_DATA_VERSION;
    s->sensor = 0;
    s->reserved0 = 0;
    s->timestamp = 0LL;

    switch(flags) {
        case META_DATA_FLUSH_COMPLETE:
            s->type = SENSOR_TYPE_META_DATA;
            s->meta_data.what = flags;
            s->meta_data.sensor = mFlushSensorEnabledVector[0];

            pthread_mutex_lock(&mHALMutex);
#ifdef __ANDROID__
            mFlushSensorEnabledVector.removeAt(0);
#else
            mFlushSensorEnabledVector.erase(mFlushSensorEnabledVector.begin());
#endif
            pthread_mutex_unlock(&mHALMutex);
            LOGV_IF(HANDLER_DATA,
                    "HAL:flush complete data: type=%d what=%d, "
                    "sensor=%d - %" PRId64 " - %d",
                    s->type, s->meta_data.what, s->meta_data.sensor,
                    s->timestamp, update);
            break;

        default:
            LOGW("HAL: Meta flags not supported");
            break;
    }

    return update;
}

#ifdef __ANDROID__
void MPLSensor::getHandle(int32_t handle, int &what, android::String8 &sname)
#else
void MPLSensor::getHandle(int32_t handle, int &what, std::string &sname)
#endif
{
    VFUNC_LOG;

    what = -1;

    if (handle >= ID_NUMBER) {
        LOGV_IF(PROCESS_VERBOSE, "HAL:handle over = %d",handle);
        return;
    }
    switch (handle) {
        case ID_RG:
            what = RawGyro;
            sname = "RawGyro";
            break;
        case ID_A:
            what = Accelerometer;
            sname = "Accelerometer";
            break;
        case ID_RM:
            what = RawMagneticField;
            sname = "RawMagneticField";
            break;
        default:
            what = handle;
            sname = "Others";
            break;
    }
    LOGI_IF(PROCESS_VERBOSE, "HAL:getHandle - what=%d, sname=%s",
            what,
#ifdef __ANDROID__
            sname.string()
#else
            sname.c_str()
#endif
            );
    return;
}

int MPLSensor::readEvents(sensors_event_t* data, int count)
{
    VHANDLER_LOG;

    int numEventReceived = 0;

    // handle flush complete event
#ifdef __ANDROID__
    if(!mFlushSensorEnabledVector.isEmpty()) {
#else
    if(!mFlushSensorEnabledVector.empty()) {
#endif
        sensors_event_t temp;
        int sendEvent = metaHandler(&temp, META_DATA_FLUSH_COMPLETE);
        if(sendEvent == 1 && count > 0) {
            *data++ = temp;
            count--;
            numEventReceived++;
        }
    }

    for (int i = 0; i < ID_NUMBER; i++) {
        int update = 0;
        if (mEnabled & (1LL << i)) {
            update = CALL_MEMBER_FN(this, mHandlers[i])(mPendingEvents + i);
            if (update && (count > 0)) {
                *data++ = mPendingEvents[i];
                count--;
                numEventReceived++;
            }
        }
    }

    return numEventReceived;
}

// collect data for MPL (but NOT sensor service currently), from driver layer
void MPLSensor::buildMpuEvent(void)
{
    VHANDLER_LOG;

    unsigned short header;
    char rdata[32];
    int rsize;
    int sensor;
    char outBuffer[MAX_READ_SIZE];
    size_t nbyte = BYTES_PER_SENSOR;

    if (mEnabled == 0) {
        /* no sensor is enabled. read out all leftover */
        rsize = read(mIIOfd, outBuffer, MAX_READ_SIZE);
        inv_sensor_parsing_reset(); // reset buffer
        return;
    }

    rsize = read(mIIOfd, outBuffer, nbyte);
    header = inv_sensor_parsing(outBuffer, rdata, rsize);

    switch (header) {
        case DATA_FORMAT_MARKER:
            sensor = *((int *) (rdata + 4));
            mFlushSensorEnabledVector.push_back(sensor);
            LOGV_IF(INPUT_DATA, "HAL:MARKER DETECTED what:%d", sensor);
            break;
        case DATA_FORMAT_EMPTY_MARKER:
            sensor = *((int *) (rdata + 4));
            mFlushSensorEnabledVector.push_back(sensor);
            LOGV_IF(INPUT_DATA, "HAL:EMPTY MARKER DETECTED what:%d", sensor);
            break;
        case DATA_FORMAT_RAW_GYRO:
            mCachedGyroData[0] = *((short *) (rdata + 2));
            mCachedGyroData[1] = *((short *) (rdata + 4));
            mCachedGyroData[2] = *((short *) (rdata + 6));
            mGyroSensorTimestamp = *((long long*) (rdata + 8));
            LOGV_IF(INPUT_DATA, "HAL:RAW GYRO DETECTED:0x%x : %d %d %d -- %" PRId64,
                    header,
                    mCachedGyroData[0], mCachedGyroData[1], mCachedGyroData[2],
                    mGyroSensorTimestamp);
            break;
        case DATA_FORMAT_ACCEL:
            mCachedAccelData[0] = *((short *) (rdata + 2));
            mCachedAccelData[1] = *((short *) (rdata + 4));
            mCachedAccelData[2] = *((short *) (rdata + 6));
            mAccelSensorTimestamp = *((long long*) (rdata + 8));
            LOGV_IF(INPUT_DATA, "HAL:ACCEL DETECTED:0x%x : %d %d %d -- %" PRId64,
                    header,
                    mCachedAccelData[0], mCachedAccelData[1], mCachedAccelData[2],
                    mAccelSensorTimestamp);
            break;
        default:
            /* set timestamp 0 for all sensors not to send any data
             * to framework by checking timestamp in handlers */
            mGyroSensorTimestamp = 0;
            mAccelSensorTimestamp = 0;
            break;
    }
}

void MPLSensor::buildCompassEvent(void)
{
    VHANDLER_LOG;

    if (mCompassSensor)
        mCompassSensor->readSample(mCachedCompassData, &mCompassTimestamp, 3);
}

int MPLSensor::getFd(void) const
{
    VFUNC_LOG;
    LOGV_IF(PROCESS_VERBOSE, "getFd returning %d", mIIOfd);
    return mIIOfd;
}

int MPLSensor::getCompassFd(void) const
{
    VFUNC_LOG;
    int fd = 0;
    if (mCompassSensor)
        fd = mCompassSensor->getFd();
    LOGV_IF(PROCESS_VERBOSE, "getCompassFd returning %d", fd);
    return fd;
}

int MPLSensor::getPollTime(void)
{
    VFUNC_LOG;
    return mPollTime;
}

/** fill in the sensor list based on which sensors are configured.
 *  return the number of configured sensors.
 *  parameter list must point to a memory region of at least 7*sizeof(sensor_t)
 *  parameter len gives the length of the buffer pointed to by list
 */
int MPLSensor::populateSensorList(struct sensor_t *list, int len)
{
    VFUNC_LOG;

    int listSize;

    currentSensorList = sRawSensorList;
    listSize = sizeof(sRawSensorList);
    LOGI("The sensor list for raw data only is used");

    if(len < (int)((listSize / sizeof(sensor_t)) * sizeof(sensor_t))) {
        LOGE("HAL:sensor list too small, not populating.");
        return -(listSize / sizeof(sensor_t));
    }

    mNumSensors = listSize / sizeof(sensor_t);

    /* fill in the base values */
    memcpy(list, currentSensorList, sizeof (struct sensor_t) * mNumSensors);
#ifdef COMPASS_SUPPORT
    if (mCompassSensor)
        mCompassSensor->fillList(&list[ID_RM]);
#endif

    return mNumSensors;
}

int MPLSensor::initSysfsAttr(void)
{
    VFUNC_LOG;

    unsigned char i = 0;
    char sysfs_path[MAX_SYSFS_NAME_LEN];
    char *sptr;
    char **dptr;

    memset(sysfs_path, 0, sizeof(sysfs_path));

    sysfs_names_ptr =
        (char*)malloc(sizeof(char[MAX_SYSFS_ATTRB][MAX_SYSFS_NAME_LEN]));
    sptr = sysfs_names_ptr;
    if (sptr != NULL) {
        dptr = (char**)&mpu;
        do {
            *dptr++ = sptr;
            memset(sptr, 0, sizeof(char));
            sptr += sizeof(char[MAX_SYSFS_NAME_LEN]);
        } while (++i < MAX_SYSFS_ATTRB);
    } else {
        LOGE("HAL:couldn't alloc mem for sysfs paths");
        return -1;
    }

    // get absolute IIO path & build MPU's sysfs paths
    inv_get_sysfs_path(sysfs_path);

    memcpy(mSysfsPath, sysfs_path, sizeof(sysfs_path));
    sprintf(mpu.chip_enable, "%s%s", sysfs_path, "/buffer/enable");
    sprintf(mpu.buffer_length, "%s%s", sysfs_path, "/buffer/length");

    sprintf(mpu.in_timestamp_en, "%s%s", sysfs_path,
            "/scan_elements/in_timestamp_en");
    sprintf(mpu.in_timestamp_index, "%s%s", sysfs_path,
            "/scan_elements/in_timestamp_index");
    sprintf(mpu.in_timestamp_type, "%s%s", sysfs_path,
            "/scan_elements/in_timestamp_type");

    sprintf(mpu.self_test, "%s%s", sysfs_path, "/misc_self_test");

    /* gyro sysfs */
    sprintf(mpu.gyro_orient, "%s%s", sysfs_path, "/info_anglvel_matrix");
    sprintf(mpu.gyro_fifo_enable, "%s%s", sysfs_path, "/in_anglvel_enable");
    sprintf(mpu.gyro_fsr, "%s%s", sysfs_path, "/in_anglvel_scale");
    sprintf(mpu.gyro_sf, "%s%s", sysfs_path, "/info_gyro_sf");
    sprintf(mpu.gyro_rate, "%s%s", sysfs_path, "/in_anglvel_rate");
    sprintf(mpu.gyro_wake_fifo_enable, "%s%s", sysfs_path, "/in_anglvel_wake_enable");
    sprintf(mpu.gyro_wake_rate, "%s%s", sysfs_path, "/in_anglvel_wake_rate");

    /* accel sysfs */
    sprintf(mpu.accel_orient, "%s%s", sysfs_path, "/info_accel_matrix");
    sprintf(mpu.accel_fifo_enable, "%s%s", sysfs_path, "/in_accel_enable");
    sprintf(mpu.accel_rate, "%s%s", sysfs_path, "/in_accel_rate");
    sprintf(mpu.accel_fsr, "%s%s", sysfs_path, "/in_accel_scale");
    sprintf(mpu.accel_wake_fifo_enable, "%s%s", sysfs_path, "/in_accel_wake_enable");
    sprintf(mpu.accel_wake_rate, "%s%s", sysfs_path, "/in_accel_wake_rate");

    /* accel offset */
    sprintf(mpu.in_accel_x_offset, "%s%s", sysfs_path, "/in_accel_x_offset");
    sprintf(mpu.in_accel_y_offset, "%s%s", sysfs_path, "/in_accel_y_offset");
    sprintf(mpu.in_accel_z_offset, "%s%s", sysfs_path, "/in_accel_z_offset");

    /* gyro offset */
    sprintf(mpu.in_gyro_x_offset, "%s%s", sysfs_path, "/in_anglvel_x_offset");
    sprintf(mpu.in_gyro_y_offset, "%s%s", sysfs_path, "/in_anglvel_y_offset");
    sprintf(mpu.in_gyro_z_offset, "%s%s", sysfs_path, "/in_anglvel_z_offset");

    /* batch and flush */
    sprintf(mpu.batchmode_timeout, "%s%s", sysfs_path,
            "/misc_batchmode_timeout");
    sprintf(mpu.flush_batch, "%s%s", sysfs_path,
            "/misc_flush_batch");

    return 0;
}

int MPLSensor::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
    VFUNC_LOG;

    int period_ns_int;
    int i, list_index;
    bool dryRun = false;
#ifdef __ANDROID__
    android::String8 sname;
#else
    std::string sname;
#endif
    int what = -1;

    /* exit if no chip is connected */
    if (!mChipDetected)
        return -EINVAL;

    period_ns_int = (NS_PER_SECOND + (period_ns - 1))/ period_ns;
    period_ns = NS_PER_SECOND / period_ns_int;

    LOGI_IF(PROCESS_VERBOSE,
            "HAL:batch called - handle=%d, flags=%d, period=%" PRId64 ", timeout=%" PRId64,
            handle, flags, period_ns, timeout);

    if(flags & SENSORS_BATCH_DRY_RUN) {
        dryRun = true;
        LOGI_IF(PROCESS_VERBOSE,
                "HAL:batch - dry run mode is set (%d)", SENSORS_BATCH_DRY_RUN);
    }

    if (flags & SENSORS_BATCH_WAKE_UPON_FIFO_FULL) {
        LOGE("HAL: batch SENSORS_BATCH_WAKE_UPON_FIFO_FULL is not supported");
        return -EINVAL;
    }

    getHandle(handle, what, sname);
    if(what < 0) {
        LOGE("HAL:batch sensors %d not found", handle);
        return -EINVAL;
    }

    LOGV_IF(PROCESS_VERBOSE,
            "HAL:batch : %" PRId64 " ns, (%.2f Hz) timeout=%" PRId64, period_ns, NS_PER_SECOND_FLOAT / period_ns, timeout);

    int size = mNumSensors;
    list_index = -1;
    for (i = 0; i < size; i++) {
        if (handle == currentSensorList[i].handle) {
            list_index = i;
            break;
        }
    }
    if (period_ns > currentSensorList[list_index].maxDelay * 1000)
        period_ns = currentSensorList[list_index].maxDelay * 1000;

    if (period_ns < currentSensorList[list_index].minDelay * 1000)
        period_ns = currentSensorList[list_index].minDelay * 1000;

#if 1
    if (size > 0) {
        if (currentSensorList[list_index].fifoMaxEventCount != 0) {
            LOGV_IF(PROCESS_VERBOSE, "HAL: batch - select sensor (handle %d)", list_index);
        } else if (timeout > 0) {
            LOGE("sensor (handle %d) is not supported in batch mode", list_index);
            return -EINVAL;
        }
    }
#endif

    if(dryRun == true) {
        return 0;
    }

    switch (what) {
        case RawGyro:
            setGyroRate(period_ns);
            break;
        case Accelerometer:
            setAccelRate(period_ns);
            break;
        case RawMagneticField:
            setMagRate(period_ns);
            break;
    }
    return 0;
}

int MPLSensor::flush(int handle)
{
    VFUNC_LOG;

#ifdef __ANDROID__
    android::String8 sname;
#else
    std::string sname;
#endif
    int what = -1;

    /* exit if no chip is connected */
    if (!mChipDetected)
        return -EINVAL;

    getHandle(handle, what, sname);
    if (what < 0) {
        LOGE("HAL:flush - what=%d is invalid", what);
        return -EINVAL;
    }

    LOGV_IF(PROCESS_VERBOSE, "HAL: flush - select sensor %s (handle %d)",
#ifdef __ANDROID__
            sname.string(),
#else
            sname.c_str(),
#endif
            handle);

    /*write sysfs */
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            handle, mpu.flush_batch, getTimestamp());

    if (write_sysfs_int(mpu.flush_batch, handle) < 0) {
        LOGE("HAL:ERR can't write flush_batch");
    }

    return 0;
}
