/*
 * Copyright (C) 2014-2020 InvenSense, Inc.
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
#include <vector>
#include <string>
#include <string.h>

#include "MPLSensor.h"
#include "MPLSupport.h"
#include "sensor_params.h"

#include "Log.h"
#include "ml_sysfs_helper.h"

#define MAX_SYSFS_ATTRB (sizeof(struct sysfs_attrbs) / sizeof(char*))

/* Set default accel and gyro FSR (use enhanced FSR if available) */
#ifdef ACCEL_ENHANCED_FSR_SUPPORT
#  define DEFAULT_ACCEL_FSR        32.0f     // 32g
#  define DEFAULT_ACCEL_FSR_SYSFS  4         // 0:2g, 1:4g, 2:8g, 3:16g, 4:32g
#else
#  ifdef INV_HIFI_ACCEL_16G
#    define DEFAULT_ACCEL_FSR        16.0f   // 16g
#    define DEFAULT_ACCEL_FSR_SYSFS  3       // 0:2g, 1:4g, 2:8g, 3:16g, 4:32g
#  else
#    define DEFAULT_ACCEL_FSR        8.0f    // 8g
#    define DEFAULT_ACCEL_FSR_SYSFS  2       // 0:2g, 1:4g, 2:8g, 3:16g, 4:32g
#  endif
#endif

#ifdef GYRO_ENHANCED_FSR_SUPPORT
#  define DEFAULT_GYRO_FSR         4000.0f   // 4000dps
#  define DEFAULT_GYRO_FSR_SYSFS   4         // 0:250dps, 1:500dps, 2:1000dps, 3:2000dps, 4:4000dps
#else
#  ifdef INV_GYRO_250DPS
#    define DEFAULT_GYRO_FSR         250.0f // 250dps
#    define DEFAULT_GYRO_FSR_SYSFS   0       // 0:250dps, 1:500dps, 2:1000dps, 3:2000dps, 4:4000dps
#  else
#    define DEFAULT_GYRO_FSR         2000.0f // 2000dps
#    define DEFAULT_GYRO_FSR_SYSFS   3       // 0:250dps, 1:500dps, 2:1000dps, 3:2000dps, 4:4000dps
#  endif
#endif

/* Force fixed full FSR for FIFO high resolution */
#ifdef FIFO_HIGH_RES_ENABLE

#ifdef ACCEL_ENHANCED_FSR_SUPPORT
#define ACCEL_FSR        32.0f
#define ACCEL_FSR_SYSFS  4
#else
#define ACCEL_FSR        16.0f
#define ACCEL_FSR_SYSFS  3
#endif

#ifdef GYRO_ENHANCED_FSR_SUPPORT
#define GYRO_FSR        4000.0f
#define GYRO_FSR_SYSFS  4
#else
#define GYRO_FSR        2000.0f
#define GYRO_FSR_SYSFS  3
#endif

/* Set default FSR otherwise */
#else /* FIFO_HIGH_RES_ENABLE */

#define ACCEL_FSR        DEFAULT_ACCEL_FSR
#define ACCEL_FSR_SYSFS  DEFAULT_ACCEL_FSR_SYSFS
#define GYRO_FSR         DEFAULT_GYRO_FSR
#define GYRO_FSR_SYSFS   DEFAULT_GYRO_FSR_SYSFS

#endif /* FIFO_HIGH_RES_ENABLE */

#ifdef ODR_SMPLRT_DIV
#define MAX_DELAY_US    250000 // for ICM2xxxx
#else
#define MAX_DELAY_US    320000 // for ICM4xxxx
#endif

#ifdef FIFO_HIGH_RES_ENABLE
#define MAX_LSB_DATA    524288.0f   // 2^19
#else
#define MAX_LSB_DATA    32768.0f    // 2^15
#endif

/* Set Chip temperature reporting period */
#define INV_CHIP_TEMPERATURE_REPORT_PERIOD_MS    100

/*******************************************************************************
 * MPLSensor class implementation
 ******************************************************************************/
static struct sensor_t sRawSensorList[] =
{
    {
        .name = "Invensense Gyroscope",
        .vendor = "Invensense",
        .version = 1,
        .handle = SENSORS_GYROSCOPE_HANDLE,
        .type = SENSOR_TYPE_GYROSCOPE,
        .maxRange = GYRO_FSR * M_PI / 180.0f,
        .resolution = GYRO_FSR * M_PI / (180.0f * MAX_LSB_DATA),
        .power = 3.0f,
        .minDelay = 5000,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = SENSOR_STRING_TYPE_GYROSCOPE,
        .requiredPermission = "",
        .maxDelay = MAX_DELAY_US,
        .flags = SENSOR_FLAG_CONTINUOUS_MODE | SENSOR_FLAG_ADDITIONAL_INFO,
        .reserved = {},
    },
    {
        .name = "Invensense Accelerometer",
        .vendor = "Invensense",
        .version = 1,
        .handle = SENSORS_ACCELERATION_HANDLE,
        .type = SENSOR_TYPE_ACCELEROMETER,
        .maxRange = GRAVITY_EARTH * ACCEL_FSR,
        .resolution = GRAVITY_EARTH * ACCEL_FSR / MAX_LSB_DATA,
        .power = 0.4f,
        .minDelay = 5000,
        .fifoReservedEventCount = 0,
        .fifoMaxEventCount = 0,
        .stringType = SENSOR_STRING_TYPE_ACCELEROMETER,
        .requiredPermission = "",
        .maxDelay = MAX_DELAY_US,
        .flags = SENSOR_FLAG_CONTINUOUS_MODE | SENSOR_FLAG_ADDITIONAL_INFO,
        .reserved = {}
    },
};

MPLSensor::MPLSensor(CompassSensor *compass, PressureSensor *pressure)
    : SensorBase(NULL, NULL),
    mEnabled(0),
    iio_fd(-1),
    chip_temperature_fd(-1),
    mIIOReadSize(0),
    mPollTime(-1),
    mGyroSensorPrevTimestamp(0),
    mAccelSensorPrevTimestamp(0),
    mCompassPrevTimestamp(0),
    mPressurePrevTimestamp(0)
{
    VFUNC_LOG;

    mCompassSensor = compass;
    mPressureSensor = pressure;

    LOGV_IF(EXTRA_VERBOSE,
            "HAL:MPLSensor constructor : NumSensors = %d", TotalNumSensors);

    pthread_mutex_init(&mHALMutex, NULL);
    memset(mGyroOrientationMatrix, 0, sizeof(mGyroOrientationMatrix));
    memset(mAccelOrientationMatrix, 0, sizeof(mAccelOrientationMatrix));
    memset(mCompassOrientationMatrix, 0, sizeof(mCompassOrientationMatrix));
    memset(mGyroLocation, 0, sizeof(mGyroLocation));
    memset(mAccelLocation, 0, sizeof(mAccelLocation));
    memset(mCompassLocation, 0, sizeof(mCompassLocation));
    memset(mPressureLocation, 0, sizeof(mPressureLocation));
    mFlushSensorEnabledVector.reserve(TotalNumSensors);
    memset(mEnabledTime, 0, sizeof(mEnabledTime));
    mBatchEnabled = 0;
    for (int i = 0; i < TotalNumSensors; i++)
        mBatchTimeouts[i] = 100000000000LL;
    mBatchTimeoutInMs = 0;
    memset(mChipTemperatureTimestamp, 0, sizeof(mChipTemperatureTimestamp));

    /* setup sysfs paths */
    inv_init_sysfs_attributes();

    /* get chip name */
    if (inv_get_chip_name(chip_ID) != INV_SUCCESS) {
        LOGE("HAL:ERR Failed to get chip ID\n");
        mChipDetected = false;
    } else {
        LOGV_IF(PROCESS_VERBOSE, "HAL:Chip ID= %s\n", chip_ID);
        mChipDetected = true;
    }

    /* print software version string */
    LOGI("InvenSense MA-Lite Sensors HAL version %d.%d.%d%s\n",
         INV_SENSORS_HAL_VERSION_MAJOR, INV_SENSORS_HAL_VERSION_MINOR,
         INV_SENSORS_HAL_VERSION_PATCH, INV_SENSORS_HAL_VERSION_SUFFIX);

    /* enable iio */
    enable_iio_sysfs();

    /* setup orientation matrix */
    inv_set_device_properties();

    /* open temperature fd */
    chip_temperature_fd = open(mpu.chip_temperature, O_RDONLY);
    if (chip_temperature_fd == -1) {
        LOGE("HAL: could not open temperature node [%s], error %d", mpu.chip_temperature, errno);
    }

    /* initialize sensor data */
    memset(mPendingEvents, 0, sizeof(mPendingEvents));
    mPendingEvents[Gyro].version = sizeof(sensors_event_t);
    mPendingEvents[Gyro].sensor = ID_G;
    mPendingEvents[Gyro].type = SENSOR_TYPE_GYROSCOPE;
    mPendingEvents[Gyro].gyro.status = SENSOR_STATUS_UNRELIABLE;
    mPendingEvents[Accelerometer].version = sizeof(sensors_event_t);
    mPendingEvents[Accelerometer].sensor = ID_A;
    mPendingEvents[Accelerometer].type = SENSOR_TYPE_ACCELEROMETER;
    mPendingEvents[Accelerometer].acceleration.status
        = SENSOR_STATUS_UNRELIABLE;
    mPendingEvents[MagneticField].version = sizeof(sensors_event_t);
    mPendingEvents[MagneticField].sensor = ID_M;
    mPendingEvents[MagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD;
    mPendingEvents[MagneticField].magnetic.status =
        SENSOR_STATUS_UNRELIABLE;
    mPendingEvents[Pressure].version = sizeof(sensors_event_t);
    mPendingEvents[Pressure].sensor = ID_PS;
    mPendingEvents[Pressure].type = SENSOR_TYPE_PRESSURE;

    /* Event Handlers */
    mHandlers[Gyro] = &MPLSensor::gyroHandler;
    mHandlers[Accelerometer] = &MPLSensor::accelHandler;
    mHandlers[MagneticField] = &MPLSensor::compassHandler;
    mHandlers[Pressure] = &MPLSensor::psHandler;

    /* initialize delays to reasonable values */
    for (int i = 0; i < TotalNumSensors; i++) {
        mDelays[i] = NS_PER_SECOND;
    }

    /* disable all sensors */
    enableGyro(0);
    enableAccel(0);
    enableCompass(0);
    enablePressure(0);

    /* FIFO high resolution mode */
    /* This needs to be set before setting FSR */
#ifdef FIFO_HIGH_RES_ENABLE
    write_sysfs_int(mpu.high_res_mode, 1);
    LOGI("HAL:FIFO High resolution enabled");
#else
    write_sysfs_int(mpu.high_res_mode, 0);
#endif

    /* set accel FSR */
    write_sysfs_int(mpu.accel_fsr, ACCEL_FSR_SYSFS);
    read_sysfs_int(mpu.accel_fsr, &mAccelFsrGee); /* read actual fsr */

    /* set gyro FSR */
    write_sysfs_int(mpu.gyro_fsr, GYRO_FSR_SYSFS);
    read_sysfs_int(mpu.gyro_fsr, &mGyroFsrDps); /* read actual fsr */

    /* reset batch timeout */
    setBatchTimeout(0);
}

void MPLSensor::enable_iio_sysfs(void)
{
    VFUNC_LOG;

    char iio_device_node[MAX_CHIP_ID_LEN];
    FILE *tempFp = NULL;

    // turn off chip in case
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            0, mpu.chip_enable, (long long)getTimestamp());
    tempFp = fopen(mpu.chip_enable, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open chip enable");
    } else {
        if (fprintf(tempFp, "%d", 0) < 0) {
            LOGE("HAL:could not write chip enable");
        }
        fclose(tempFp);
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            1, mpu.scan_el_en, (long long)getTimestamp());
    tempFp = fopen(mpu.scan_el_en, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open scan element enable");
    } else {
        if (fprintf(tempFp, "%d", 1) < 0) {
            LOGE("HAL:could not write scan element enable");
        }
        fclose(tempFp);
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            IIO_BUFFER_LENGTH, mpu.buffer_length, (long long)getTimestamp());
    tempFp = fopen(mpu.buffer_length, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open buffer length");
    } else {
        if (fprintf(tempFp, "%d", IIO_BUFFER_LENGTH) < 0) {
            LOGE("HAL:could not write buffer length");
        }
        fclose(tempFp);
    }

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            1, mpu.chip_enable, (long long)getTimestamp());
    tempFp = fopen(mpu.chip_enable, "w");
    if (tempFp == NULL) {
        LOGE("HAL:could not open chip enable");
    } else {
        if (fprintf(tempFp, "%d", 1) < 0) {
            LOGE("HAL:could not write chip enable");
        }
        fclose(tempFp);
    }

    inv_get_iio_device_node(iio_device_node);
    iio_fd = open(iio_device_node, O_RDONLY);
    if (iio_fd < 0) {
        LOGE("HAL:could not open iio device node");
    } else {
        LOGV_IF(ENG_VERBOSE, "HAL:iio iio_fd opened : %d", iio_fd);
    }
}

void MPLSensor::inv_set_device_properties(void)
{
    VFUNC_LOG;

    /* gyro/accel mount matrix */
    inv_get_sensors_orientation();
	
    if (mCompassSensor) {
        /* compass mount matrix */
        mCompassSensor->getOrientationMatrix(mCompassOrientationMatrix);
    }
}

void MPLSensor::inv_get_sensors_orientation(void)
{
    VFUNC_LOG;

    FILE *fptr;

    // get gyro orientation
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.gyro_orient, (long long)getTimestamp());
    fptr = fopen(mpu.gyro_orient, "r");
    if (fptr != NULL) {
        int om[9];
        if (fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
                    &om[6], &om[7], &om[8]) < 0) {
            LOGE("HAL:Could not read gyro mounting matrix");
        } else {
            LOGV_IF(EXTRA_VERBOSE,
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
        fclose(fptr);
    }

    // get accel orientation
    LOGV_IF(SYSFS_VERBOSE,
            "HAL:sysfs:cat %s (%lld)", mpu.accel_orient, (long long)getTimestamp());
    fptr = fopen(mpu.accel_orient, "r");
    if (fptr != NULL) {
        int om[9];
        if (fscanf(fptr, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    &om[0], &om[1], &om[2], &om[3], &om[4], &om[5],
                    &om[6], &om[7], &om[8]) < 0) {
            LOGE("HAL:could not read accel mounting matrix");
        } else {
            LOGV_IF(EXTRA_VERBOSE,
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
        fclose(fptr);
    }

    // Fill sensors location here if needed
    // mGyroLocation = {0, 0, 0};
    // mAccelLocation = {0, 0, 0};
    // mCompassLocation = {0, 0, 0};
    // mPressureLocation = {0, 0, 0};
}

MPLSensor::~MPLSensor()
{
    VFUNC_LOG;

    /* Close open fds */
    if (iio_fd >= 0) {
        close(iio_fd);
    }
}

void MPLSensor::inv_write_sysfs(uint32_t delay, char *sysfs_rate)
{
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %.0f > %s (%lld)",
        NS_PER_SECOND_FLOAT / delay, sysfs_rate, (long long)getTimestamp());
    write_sysfs_int(sysfs_rate, NS_PER_SECOND_FLOAT / delay);
	
}

void MPLSensor::setGyroRate(uint64_t delay)
{
    inv_write_sysfs(delay, mpu.gyro_rate);
}

void MPLSensor::setAccelRate(uint64_t delay)
{
    inv_write_sysfs(delay, mpu.accel_rate);
}

void MPLSensor::setMagRate(uint64_t delay)
{
    if (mCompassSensor)
        mCompassSensor->setDelay(ID_M, delay);
}

void MPLSensor::setPressureRate(uint64_t delay)
{
    if (mPressureSensor)
        mPressureSensor->setDelay(ID_PS, delay);
}

int MPLSensor::enableGyro(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.gyro_fifo_enable, (long long)getTimestamp());
    res += write_sysfs_int(mpu.gyro_fifo_enable, en);

    return res;
}

int MPLSensor::enableAccel(int en)
{
    VFUNC_LOG;

    int res = 0;

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            en, mpu.accel_fifo_enable, (long long)getTimestamp());
    res += write_sysfs_int(mpu.accel_fifo_enable, en);

    return res;
}

int MPLSensor::enableCompass(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mCompassSensor)
        res = mCompassSensor->enable(ID_M, en);

    return res;
}

int MPLSensor::enablePressure(int en)
{
    VFUNC_LOG;

    int res = 0;

    if (mPressureSensor)
        res = mPressureSensor->enable(ID_PS, en);

    return res;
}

int MPLSensor::enable(int32_t handle, int en)
{
    VFUNC_LOG;

    std::string sname;
    int what = -1;
    int err = 0;

    /* exit if no chip is connected */
    if (!mChipDetected)
        return -EINVAL;

    getHandle(handle, what, sname);
    if (what < 0) {
        LOGV_IF(ENG_VERBOSE, "HAL:can't find handle %d",handle);
        return -EINVAL;
    }
    if (!en)
        mBatchEnabled &= ~(1LL << what);
    if (mEnabled == 0) {
        // reset buffer
        mIIOReadSize = 0;
    }

    LOGV_IF(PROCESS_VERBOSE, "HAL:handle = %d en = %d", handle, en);

    uint64_t newState = en ? 1 : 0;

    LOGV_IF(PROCESS_VERBOSE, "HAL:enable - sensor %s (handle %d) %s -> %s",
            sname.c_str(),
            handle,
            ((mEnabled & (1LL << what)) ? "en" : "dis"),
            (((newState) << what) ? "en" : "dis"));
    LOGV_IF(PROCESS_VERBOSE, "HAL:%s sensor state change what=%d",
            sname.c_str(),
            what);

    if (en) {
        pthread_mutex_lock(&mHALMutex);
        mAdditionalInfoEnabledVector.push_back(handle);
        pthread_mutex_unlock(&mHALMutex);
    }

    if (((newState) << what) != (mEnabled & (1LL << what))) {
        uint64_t flags = newState;

        mEnabled &= ~(1LL << what);
        mEnabled |= (uint64_t(flags) << what);

        switch (what) {
            case Gyro:
                enableGyro(en);
                break;
            case Accelerometer:
                enableAccel(en);
                break;
            case MagneticField:
                enableCompass(en);
                break;
            case Pressure:
                enablePressure(en);
                break;
        }
        if (en)
            mEnabledTime[what] = getTimestamp();
        else
            mEnabledTime[what] = 0;
    }

    updateBatchTimeout();

    return err;
}

void MPLSensor::setBatchTimeout(int64_t timeout_ns)
{
    int timeout_ms = (int)(timeout_ns / 1000000LL);

    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%" PRId64 ")",
            timeout_ms, mpu.batchmode_timeout, getTimestamp());
    write_sysfs_int(mpu.batchmode_timeout, timeout_ms);
    mBatchTimeoutInMs = timeout_ms;
}

void MPLSensor::updateBatchTimeout(void)
{
    int64_t batchingTimeout = 100000000000LL;
    int64_t ns = 0;

    if (mBatchEnabled) {
        for (uint32_t i = 0; i < TotalNumSensors; i++) {
            if (mEnabled & (1LL << i)) {
                if (mBatchEnabled & (1LL << i))
                    ns = mBatchTimeouts[i];
                else
                    ns = 0;
                batchingTimeout = (ns < batchingTimeout) ? ns : batchingTimeout;
            }
        }
    } else {
        batchingTimeout = 0;
    }
    if (mBatchTimeoutInMs != batchingTimeout) {
        setBatchTimeout(batchingTimeout);
    }
}

/*  these handlers transform mpl data into one of the Android sensor types */
int MPLSensor::gyroHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    int update = 0;
    int data[3];
    const float scale = (float)mGyroFsrDps / MAX_LSB_DATA * M_PI / 180;

    /* convert to body frame */
    for (unsigned int i = 0; i < 3; i++) {
        data[i] = mCachedGyroData[0] * mGyroOrientationMatrix[i * 3] +
                  mCachedGyroData[1] * mGyroOrientationMatrix[i * 3 + 1] +
                  mCachedGyroData[2] * mGyroOrientationMatrix[i * 3 + 2];
        s->gyro.v[i] = (float)data[i] * scale;
    }

    s->timestamp = mGyroSensorTimestamp;
    s->gyro.status = SENSOR_STATUS_UNRELIABLE;

    /* timestamp check */
    if ((mGyroSensorTimestamp > mGyroSensorPrevTimestamp) &&
        (mGyroSensorTimestamp > mEnabledTime[Gyro])) {
        update = 1;
    }

    mGyroSensorPrevTimestamp = mGyroSensorTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:gyro data : %+f %+f %+f -- %" PRId64 " - %d",
        s->gyro.x, s->gyro.y, s->gyro.z, s->timestamp, update);

    return update;
}

int MPLSensor::accelHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    int update = 0;
    int data[3];
    const float scale = 1.f / (MAX_LSB_DATA / (float)mAccelFsrGee) * 9.80665f;

    /* convert to body frame */
    for (unsigned int i = 0; i < 3; i++) {
        data[i] = mCachedAccelData[0] * mAccelOrientationMatrix[i * 3] +
                  mCachedAccelData[1] * mAccelOrientationMatrix[i * 3 + 1] +
                  mCachedAccelData[2] * mAccelOrientationMatrix[i * 3 + 2];
        s->acceleration.v[i] = (float)data[i] * scale;
    }

    s->timestamp = mAccelSensorTimestamp;
    s->acceleration.status = SENSOR_STATUS_UNRELIABLE;

    /* timestamp check */
    if ((mAccelSensorTimestamp > mAccelSensorPrevTimestamp) &&
        (mAccelSensorTimestamp > mEnabledTime[Accelerometer])) {
        update = 1;
    }

    mAccelSensorPrevTimestamp = mAccelSensorTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:accel data : %+f %+f %+f -- %" PRId64 " - %d",
        s->acceleration.x, s->acceleration.y, s->acceleration.z,
        s->timestamp, update);

    return update;
}

int MPLSensor::compassHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    int update = 0;
    int data[3];
    const float scale = 1.f / (1 << 16); // 1uT for 2^16

    /* convert to body frame */
    for (unsigned int i = 0; i < 3 ; i++) {
        data[i] = (mCachedCompassData[0]) * mCompassOrientationMatrix[i * 3] +
                  (mCachedCompassData[1]) * mCompassOrientationMatrix[i * 3 + 1] +
                  (mCachedCompassData[2]) * mCompassOrientationMatrix[i * 3 + 2];
        s->magnetic.v[i] = (float)data[i] * scale;
    }

    s->timestamp = mCompassTimestamp;
    s->magnetic.status = SENSOR_STATUS_UNRELIABLE;

    /* timestamp check */
    if ((mCompassTimestamp > mCompassPrevTimestamp) &&
        (mCompassTimestamp > mEnabledTime[MagneticField])) {
        update = 1;
    }

    mCompassPrevTimestamp = mCompassTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:raw compass data: %+f %+f %+f %d -- %" PRId64 " - %d",
        s->magnetic.x, s->magnetic.y, s->magnetic.z,
        s->magnetic.status, s->timestamp, update);

    return update;
}

int MPLSensor::psHandler(sensors_event_t* s)
{
    VHANDLER_LOG;

    int update = 0;

    s->pressure = mCachedPressureData / 100.f / 100.f; /* hPa */
    s->timestamp = mPressureTimestamp;

    /* timestamp check */
    if ((mPressureTimestamp > mPressurePrevTimestamp) &&
        (mPressureTimestamp > mEnabledTime[Pressure])) {
        update = 1;
    }

    mPressurePrevTimestamp = mPressureTimestamp;

    LOGV_IF(HANDLER_DATA, "HAL:pressure data: %+f -- %" PRId64 " - %d",
        s->pressure,
        s->timestamp, update);

    return update;
}

int MPLSensor::metaHandler(int sensor, sensors_event_t* s, int flags)
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
            s->meta_data.sensor = sensor;

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

// Sensor Placement event
int MPLSensor::additionalInfoSensorPlacement(int handle, unsigned int seq, sensors_event_t* event)
{
    static const signed char identityOrientation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float *location;
    const signed char *orient;

    if (handle < 0 || handle >= TotalNumSensors) {
        return -1;
    }

    switch (handle) {
    case Gyro:
        orient = mGyroOrientationMatrix;
        location = mGyroLocation;
        break;
    case MagneticField:
        orient = mCompassOrientationMatrix;
        location = mCompassLocation;
        break;
    case Pressure:
        orient = identityOrientation;
        location = mPressureLocation;
        break;
    // use accelerometer orientation by default
    case Accelerometer:
    default:
        orient = mAccelOrientationMatrix;
        location = mAccelLocation;
        break;
    }

    memset(event, 0, sizeof(*event));
    event->version = sizeof(sensors_event_t);
    event->sensor = handle;
    event->type = SENSOR_TYPE_ADDITIONAL_INFO;
    event->timestamp = seq;
    event->additional_info.type = AINFO_SENSOR_PLACEMENT;
    // inverse orientation matrix, Android system to sensor system
    for (int i = 0; i < 3; ++i) {
        float *data = &event->additional_info.data_float[i * 4];
        data[0] = orient[i];
        data[1] = orient[i + 3];
        data[2] = orient[i + 6];
        data[3] = location[i];
    }

    return 1;
}

// Internal Temperature event
int MPLSensor::additionalInfoInternalTemperature(int handle, unsigned int seq, sensors_event_t *event)
{
    int temperature;
    int64_t temp_ts;
    int ret;

    switch (handle) {
    case Gyro:
    case Accelerometer:
        // Push temperature payload
        ret = inv_read_temperature(&temperature, &temp_ts);
        if (ret < 0) {
            return 0;
        }
        mChipTemperatureTimestamp[handle] = temp_ts;
        memset(event, 0, sizeof(*event));
        event->version = sizeof(sensors_event_t);
        event->sensor = handle;
        event->type = SENSOR_TYPE_ADDITIONAL_INFO;
        event->timestamp = seq;
        event->additional_info.type = AINFO_INTERNAL_TEMPERATURE;
        event->additional_info.data_float[0] = (float)temperature / 100.f;
        ret = 1;
        break;
    default:
        ret = 0;
        break;
    }

    return ret;
}

int MPLSensor::additionalInfoHandler(int handle, sensors_event_t* data, int count)
{
    VHANDLER_LOG;

    sensors_event_t marker, event;
    int numEventReceived = 0;
    unsigned int seq = 0;
    int ret;

    // minimum 3 events: begin - sensor placement - end
    if (count < 3)
        return -1;

    // initalize marker event
    memset(&marker, 0, sizeof(marker));
    marker.version = sizeof(sensors_event_t);
    marker.sensor = handle;
    marker.type = SENSOR_TYPE_ADDITIONAL_INFO;

    // Push begin frame
    marker.timestamp = seq++;
    marker.additional_info.type = AINFO_BEGIN;
    *data++ = marker;
    numEventReceived++;

    // Sensor placement
    ret = additionalInfoSensorPlacement(handle, seq, &event);
    if (ret == 1) {
        *data++ = event;
        numEventReceived++;
        seq++;
    }

    if (count >= 4) {
            // Internal Temperature
            ret = additionalInfoInternalTemperature(handle, seq, &event);
            if (ret == 1) {
                *data++ = event;
                numEventReceived++;
                seq++;
            }
    }

    // Push end frame
    marker.timestamp = seq++;
    marker.additional_info.type = AINFO_END;
    *data++ = marker;
    numEventReceived++;

    LOGV_IF(HANDLER_DATA, "HAL:additionalInfo %d data", numEventReceived);

    return numEventReceived;
}

int MPLSensor::periodicAdditionalInfoHandler(int handle, sensors_event_t* data, int count)
{
    VHANDLER_LOG;

    const int64_t timestamp = getTimestamp();
    sensors_event_t marker, events[1];
    unsigned int ind = 0;
    unsigned int maxEvents;
    int numEvents = 0;
    int ret;

    // we need at least 3 free data events for returning additional info frames
    if (count < 3) {
        return -1;
    }
    maxEvents = count - 2;

    // forge internal temperature frame
    if ((timestamp - mChipTemperatureTimestamp[handle]) > ((INV_CHIP_TEMPERATURE_REPORT_PERIOD_MS - 1) * 1000000LL)) {
        ret = additionalInfoInternalTemperature(handle, ind + 1, &events[ind]);
        if (ret == 1) {
            ++ind;
        }
    }

    // return immediately if there is no data frame
    if (ind == 0) {
        return 0;
    }

    // initalize marker event
    memset(&marker, 0, sizeof(marker));
    marker.version = sizeof(sensors_event_t);
    marker.sensor = handle;
    marker.type = SENSOR_TYPE_ADDITIONAL_INFO;

    // Push begin frame
    marker.timestamp = 0;
    marker.additional_info.type = AINFO_BEGIN;
    *data++ = marker;
    numEvents++;

    // Push data frames
    for (unsigned int i = 0; i < ind && i < maxEvents; ++i) {
        *data++ = events[i];
        numEvents++;
    }

    // Push end frame
    marker.timestamp = ind + 1;
    marker.additional_info.type = AINFO_END;
    *data++ = marker;
    numEvents++;

    return numEvents;
}

void MPLSensor::getHandle(int32_t handle, int &what, std::string &sname)
{
    VFUNC_LOG;

    what = -1;

    if (handle >= ID_NUMBER) {
        LOGV_IF(PROCESS_VERBOSE, "HAL:handle over = %d",handle);
        return;
    }
    switch (handle) {
        case ID_G:
            what = Gyro;
            sname = "Gyro";
            break;
        case ID_A:
            what = Accelerometer;
            sname = "Accelerometer";
            break;
        case ID_M:
            what = MagneticField;
            sname = "MagneticField";
            break;
        case ID_PS:
            what = Pressure;
            sname = "Pressure";
            break;
        default:
            what = handle;
            sname = "Others";
            break;
    }
    LOGI_IF(PROCESS_VERBOSE, "HAL:getHandle - what=%d, sname=%s",
            what,
            sname.c_str()
            );
    return;
}

/**
 *  Should be called after reading at least one of gyro
 *  compass or accel data. (Also okay for handling all of them).
 *  @returns 0, if successful, error number if not.
 */
int MPLSensor::readEvents(sensors_event_t* data, int count)
{
    VHANDLER_LOG;

    int numEventReceived = 0;

    // send additional info on enable
    if (!mAdditionalInfoEnabledVector.empty()) {
        int sendEvent = additionalInfoHandler(mAdditionalInfoEnabledVector[0], data, count);
        if (sendEvent > 0) {
            data += sendEvent;
            count -= sendEvent;
            numEventReceived += sendEvent;
        }
        pthread_mutex_lock(&mHALMutex);
        mAdditionalInfoEnabledVector.erase(mAdditionalInfoEnabledVector.begin());
        pthread_mutex_unlock(&mHALMutex);
    }

    // handle flush complete event
    if (!mFlushSensorEnabledVector.empty()) {
        int sensor = mFlushSensorEnabledVector[0];
        sensors_event_t temp;
        int sendEvent = metaHandler(sensor, &temp, META_DATA_FLUSH_COMPLETE);
        if(sendEvent == 1 && count > 0) {
            *data++ = temp;
            count--;
            numEventReceived++;
        }
        sendEvent = additionalInfoHandler(sensor, data, count);
        if (sendEvent > 0) {
            data += sendEvent;
            count -= sendEvent;
            numEventReceived += sendEvent;
        } else {
            // save sensor to send info later
            pthread_mutex_lock(&mHALMutex);
            mAdditionalInfoEnabledVector.push_back(sensor);
            pthread_mutex_unlock(&mHALMutex);
        }
        pthread_mutex_lock(&mHALMutex);
        mFlushSensorEnabledVector.erase(mFlushSensorEnabledVector.begin());
        pthread_mutex_unlock(&mHALMutex);
    }

    for (int i = 0; i < ID_NUMBER; i++) {
        int update = 0;
        if (mEnabled & (1LL << i)) {
            update = CALL_MEMBER_FN(this, mHandlers[i])(mPendingEvents + i);
            if (update && (count > 0)) {
                *data++ = mPendingEvents[i];
                count--;
                numEventReceived++;
                int sendEvent = periodicAdditionalInfoHandler(i, data, count);
                if (sendEvent > 0) {
                    data += sendEvent;
                    count -= sendEvent;
                    numEventReceived += sendEvent;
                }
            }
        }
    }

    return numEventReceived;
}

int MPLSensor::readMpuEvents(sensors_event_t* s, int count)
{
    VHANDLER_LOG;

    unsigned short header;
    char *rdata;
    int rsize;
    int sensor;
    int ptr = 0;
    int numEventReceived = 0;
    int left_over = 0;
    bool data_found;

    if (mEnabled == 0) {
        /* no sensor is enabled. read out all leftover */
        rsize = read(iio_fd, mIIOReadBuffer, sizeof(mIIOReadBuffer));
        mIIOReadSize = 0;
        return 0;
    }

    if (mCompassSensor)
        count -= COMPASS_SEN_EVENT_RESV_SZ;

    if (mPressureSensor)
        count -= PRESSURE_SEN_EVENT_RESV_SZ;

    /* read as much data as possible allowed with either
     * smaller, the buffer from upper layer or local buffer */
    int nbytes = sizeof(mIIOReadBuffer) - mIIOReadSize;
    /* assume that gyro and accel data packet size are the same
     * and larger than marker packet */
    int packet_size = DATA_FORMAT_RAW_GYRO_SZ;
    if (nbytes > count * packet_size) {
        nbytes = count * packet_size;
    }
    rsize = read(iio_fd, &mIIOReadBuffer[mIIOReadSize], nbytes);
    LOGV_IF(PROCESS_VERBOSE, "HAL: nbytes=%d rsize=%d", nbytes, rsize);
    if (rsize < 0) {
        LOGE("HAL:failed to read IIO.  nbytes=%d rsize=%d", nbytes, rsize);
        return 0;
    }
    if (rsize == 0) {
        LOGI("HAL:no data from IIO.");
        return 0;
    }

    mIIOReadSize += rsize;

    while (ptr < mIIOReadSize) {
        rdata = &mIIOReadBuffer[ptr];
        header = *(unsigned short*)rdata;
        data_found = false;
        switch (header) {
            case DATA_FORMAT_MARKER:
                if ((mIIOReadSize - ptr) < DATA_FORMAT_MARKER_SZ) {
                    left_over = mIIOReadSize - ptr;
                    break;
                }
                sensor = *((int *) (rdata + 4));
                mFlushSensorEnabledVector.push_back(sensor);
                LOGV_IF(INPUT_DATA, "HAL:MARKER DETECTED what:%d", sensor);
                ptr += DATA_FORMAT_MARKER_SZ;
                data_found = true;
                break;
            case DATA_FORMAT_EMPTY_MARKER:
                if ((mIIOReadSize - ptr) < DATA_FORMAT_EMPTY_MARKER_SZ) {
                    left_over = mIIOReadSize - ptr;
                    break;
                }
                sensor = *((int *) (rdata + 4));
                mFlushSensorEnabledVector.push_back(sensor);
                LOGV_IF(INPUT_DATA, "HAL:EMPTY MARKER DETECTED what:%d", sensor);
                ptr += DATA_FORMAT_EMPTY_MARKER_SZ;
                data_found = true;
                break;
            case DATA_FORMAT_RAW_GYRO:
                if ((mIIOReadSize - ptr) < DATA_FORMAT_RAW_GYRO_SZ) {
                    left_over = mIIOReadSize - ptr;
                    break;
                }
                mCachedGyroData[0] = *((int *) (rdata + 4));
                mCachedGyroData[1] = *((int *) (rdata + 8));
                mCachedGyroData[2] = *((int *) (rdata + 12));
                mGyroSensorTimestamp = *((long long*) (rdata + 16));
                LOGV_IF(INPUT_DATA, "HAL:RAW GYRO DETECTED:0x%x : %d %d %d -- %" PRId64,
                        header,
                        mCachedGyroData[0], mCachedGyroData[1], mCachedGyroData[2],
                        mGyroSensorTimestamp);
                ptr += DATA_FORMAT_RAW_GYRO_SZ;
                data_found = true;
                break;
            case DATA_FORMAT_ACCEL:
                if ((mIIOReadSize - ptr) < DATA_FORMAT_ACCEL_SZ) {
                    left_over = mIIOReadSize - ptr;
                    break;
                }
                mCachedAccelData[0] = *((int *) (rdata + 4));
                mCachedAccelData[1] = *((int *) (rdata + 8));
                mCachedAccelData[2] = *((int *) (rdata + 12));
                mAccelSensorTimestamp = *((long long*) (rdata +16));
                LOGV_IF(INPUT_DATA, "HAL:ACCEL DETECTED:0x%x : %d %d %d -- %" PRId64,
                        header,
                        mCachedAccelData[0], mCachedAccelData[1], mCachedAccelData[2],
                        mAccelSensorTimestamp);
                ptr += DATA_FORMAT_ACCEL_SZ;
                data_found = true;
                break;
            default:
                LOGW("HAL:no header.");
                ptr++;
                data_found = false;
                break;
        }

        if (data_found) {
            int num = readEvents(&s[numEventReceived], count);
            if (num > 0) {
                count -= num;
                numEventReceived += num;
                if (count == 0)
                    break;
                if (count < 0) {
                    LOGW("HAL:sensor_event_t buffer overflow");
                    break;
                }
            }
        }
        if (left_over) {
            break;
        }
    }

    if (left_over > 0) {
        LOGV_IF(PROCESS_VERBOSE, "HAL: leftover mIIOReadSize=%d ptr=%d",
                mIIOReadSize, ptr);
        memmove(mIIOReadBuffer, &mIIOReadBuffer[ptr], left_over);
        mIIOReadSize = left_over;
    } else {
        mIIOReadSize = 0;
    }

    return numEventReceived;
}

int MPLSensor::readCompassEvents(sensors_event_t* s, int count)
{
    VHANDLER_LOG;

    int numEventReceived = 0;

    if (count > COMPASS_SEN_EVENT_RESV_SZ)
        count = COMPASS_SEN_EVENT_RESV_SZ;

    if (mCompassSensor) {
        mCompassSensor->readSample(mCachedCompassData, &mCompassTimestamp);
        int num = readEvents(&s[numEventReceived], count);
        if (num > 0) {
            count -= num;
            numEventReceived += num;
            if (count < 0)
                LOGW("HAL:sensor_event_t buffer overflow");
        }
    }
    return numEventReceived;
}

int MPLSensor::readPressureEvents(sensors_event_t* s, int count)
{
    VHANDLER_LOG;

    int numEventReceived = 0;

    if (count > PRESSURE_SEN_EVENT_RESV_SZ)
        count = PRESSURE_SEN_EVENT_RESV_SZ;

    if (mPressureSensor) {
        mPressureSensor->readSample(&mCachedPressureData, &mPressureTimestamp);
        int num = readEvents(&s[numEventReceived], count);
        if (num > 0) {
            count -= num;
            numEventReceived += num;
            if (count < 0)
                LOGW("HAL:sensor_event_t buffer overflow");
        }
    }
    return numEventReceived;
}

int MPLSensor::inv_read_temperature(int *temperature, int64_t *ts)
{
    VHANDLER_LOG;

    int count = 0;
    char raw_buf[40];
    int raw = 0;
    long long timestamp = 0;

    memset(raw_buf, 0, sizeof(raw_buf));
    count = read_attribute_sensor(chip_temperature_fd, raw_buf,
            sizeof(raw_buf));
    if (count < 0) {
        LOGE("HAL:error reading gyro temperature");
        return -1;
    }

    count = sscanf(raw_buf, "%d %lld", &raw, &timestamp);
    if (count < 0) {
        LOGW("HAL:error parsing gyro temperature count=%d", count);
        return -1;
    }

    LOGV_IF(ENG_VERBOSE && INPUT_DATA,
            "HAL:temperature raw = %d, timestamp = %lld, count = %d",
            raw, timestamp, count);
    *temperature = raw; // degrees Celsius scaled by 100
    *ts = timestamp;

    return 0;
}

int MPLSensor::getFd(void) const
{
    VFUNC_LOG;
    LOGV_IF(PROCESS_VERBOSE, "getFd returning %d", iio_fd);
    return iio_fd;
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

int MPLSensor::populateSensorList(struct sensor_t *list, int len)
{
    VFUNC_LOG;

    int listSize;
    int maxNumSensors;

    /* base sensor list */
    listSize = sizeof(sRawSensorList);
    memcpy(mCurrentSensorList, sRawSensorList, listSize);
    mNumSensors = listSize / sizeof(sensor_t);

    maxNumSensors = sizeof(mCurrentSensorList) / sizeof(sensor_t);

    /* add compass if there is */
    if (mCompassSensor) {
        if (mCompassSensor->isSensorPresent()) {
            mNumSensors += mCompassSensor->populateSensorList(mCurrentSensorList + mNumSensors,
                    maxNumSensors - mNumSensors);
            LOGI("HAL:compass is added to a list");
        }
    }

    /* add pressure if there is */
    if (mPressureSensor) {
       if (mPressureSensor->isSensorPresent()) {
            mNumSensors += mPressureSensor->populateSensorList(mCurrentSensorList + mNumSensors,
                    maxNumSensors - mNumSensors);
            LOGI("HAL:pressure is added to a list");
       }
    }

    if (len < (int)(sizeof(struct sensor_t) * mNumSensors)) {
        LOGE("HAL:sensor list too small, not populating.");
        return -ENOMEM;
    }

    LOGI("HAL:Update sensor information");
    fillAccel(chip_ID, mCurrentSensorList);
    fillGyro(chip_ID, mCurrentSensorList);

    /* copy the list */
    memcpy(list, mCurrentSensorList, sizeof(struct sensor_t) * mNumSensors);

    return mNumSensors;
}

/* fill accel metadata */
void MPLSensor::fillAccel(const char* accel, struct sensor_t *list)
{
    VFUNC_LOG;

    unsigned int i;

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_ACCELERATION_HANDLE){
            if (strcmp(accel, "ICM20648") == 0) {
                list[i].power = ACCEL_ICM20648_POWER;
                list[i].minDelay = ACCEL_ICM20648_MINDELAY;
                list[i].maxDelay = ACCEL_ICM20648_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM20648);
			} else if (strcmp(accel, "ICM20602") == 0) {
                list[i].power = ACCEL_ICM20602_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_ICM20602_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_ICM20602_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_ICM20602_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM20602);
            } else if (strcmp(accel, "ICM20690") == 0) {
                list[i].power = ACCEL_ICM20690_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_ICM20690_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_ICM20690_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_ICM20690_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM20690);
            } else if (strcmp(accel, "IAM20680") == 0) {
                list[i].power = ACCEL_IAM20680_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_IAM20680_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_IAM20680_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_IAM20680_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_IAM20680);
            } else if (strcmp(accel, "ICM42600") == 0) {
                list[i].power = ACCEL_ICM42600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_ICM42600_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_ICM42600_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_ICM42600_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM42600);
            } else if (strcmp(accel, "ICM43600") == 0) {
                list[i].power = ACCEL_ICM43600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = ACCEL_ICM43600_MINDELAY_HIFI;
#else
                list[i].minDelay = ACCEL_ICM43600_MINDELAY;
#endif
                list[i].maxDelay = ACCEL_ICM43600_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM43600);
            }
        }
    }
}

/* fill gyro metadata */
void MPLSensor::fillGyro(const char* gyro, struct sensor_t *list)
{
    VFUNC_LOG;

    unsigned int i;

    for (i = 0; i < mNumSensors; i++) {
        if (list[i].handle == SENSORS_GYROSCOPE_HANDLE){
            if (strcmp(gyro, "ICM20648") == 0) {
                list[i].power = GYRO_ICM20648_POWER;
                list[i].minDelay = GYRO_ICM20648_MINDELAY;
                list[i].maxDelay = GYRO_ICM20648_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM20648);
            } else if (strcmp(gyro, "ICM20602") == 0) {
                list[i].power = GYRO_ICM20602_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_ICM20602_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_ICM20602_MINDELAY;
#endif
                list[i].maxDelay = GYRO_ICM20602_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM20602);
            } else if (strcmp(gyro, "ICM20690") == 0) {
                list[i].power = GYRO_ICM20690_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_ICM20690_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_ICM20690_MINDELAY;
#endif
                list[i].maxDelay = GYRO_ICM20690_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM20690);
            } else if (strcmp(gyro, "IAM20680") == 0) {
                list[i].power = GYRO_IAM20680_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_IAM20680_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_IAM20680_MINDELAY;
#endif
                list[i].maxDelay = GYRO_IAM20680_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_IAM20680);
            } else if (strcmp(gyro, "ICM42600") == 0) {
                list[i].power = GYRO_ICM42600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_ICM42600_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_ICM42600_MINDELAY;
#endif
                list[i].maxDelay = GYRO_ICM42600_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM42600);
            } else if (strcmp(gyro, "ICM43600") == 0) {
                list[i].power = GYRO_ICM43600_POWER;
#ifdef INV_HIFI_HIGH_ODR
                list[i].minDelay = GYRO_ICM43600_MINDELAY_HIFI;
#else
                list[i].minDelay = GYRO_ICM43600_MINDELAY;
#endif
                list[i].maxDelay = GYRO_ICM43600_MAXDELAY;
                list[i].fifoMaxEventCount = FIFO_SIZE_COMPUTE(FIFO_SIZE_ICM43600);
            }
        }
    }
}

int MPLSensor::inv_init_sysfs_attributes(void)
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

    sprintf(mpu.scan_el_en, "%s%s", sysfs_path,
            "/scan_elements/in_accel_en");
    sprintf(mpu.scan_el_index, "%s%s", sysfs_path,
            "/scan_elements/in_accel_index");
    sprintf(mpu.scan_el_type, "%s%s", sysfs_path,
            "/scan_elements/in_accel_type");

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

    /* FIFO high resolution mode */
    sprintf(mpu.high_res_mode, "%s%s", sysfs_path, "/in_high_res_mode");

    /* chip temperature fd */
    sprintf(mpu.chip_temperature, "%s%s", sysfs_path, "/out_temperature");

    return 0;
}

/* precondition: framework disallows this case, ie enable continuous sensor, */
/* and enable batch sensor */
/* if one sensor is in continuous mode, HAL disallows enabling batch for this sensor */
/* or any other sensors */
int MPLSensor::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
    VFUNC_LOG;

    uint32_t i;
    int list_index = 0;
    std::string sname;
    int what = -1;

    /* exit if no chip is connected */
    if (!mChipDetected)
        return -EINVAL;

    LOGI_IF(PROCESS_VERBOSE,
            "HAL:batch called - handle=%d, flags=%d, period=%" PRId64 ", timeout=%" PRId64,
            handle, flags, period_ns, timeout);

    /* check if the handle is valid */
    getHandle(handle, what, sname);
    if(what < 0) {
        LOGE("HAL:batch sensors %d not found", handle);
        return -EINVAL;
    }

    /* check if we can support issuing interrupt before FIFO fills-up */
    /* in a given timeout.                                          */
    if (flags & SENSORS_BATCH_WAKE_UPON_FIFO_FULL) {
        LOGE("HAL: batch SENSORS_BATCH_WAKE_UPON_FIFO_FULL is not supported");
        return -EINVAL;
    }

    /* find sensor_t struct for this sensor */
    for (i = 0; i < mNumSensors; i++) {
        if (handle == mCurrentSensorList[i].handle) {
            list_index = i;
            break;
        }
    }

    if (period_ns != mCurrentSensorList[list_index].maxDelay * 1000) {
        /* Round up in Hz when requested frequency has fractional digit.
         * Note: not round up if requested frequency is the same as maxDelay */
        int rate_hz;
        rate_hz = (NS_PER_SECOND + (period_ns / 2)) / period_ns;
        period_ns = NS_PER_SECOND / rate_hz;
    }

    if (period_ns > mCurrentSensorList[list_index].maxDelay * 1000)
        period_ns = mCurrentSensorList[list_index].maxDelay * 1000;
    if (period_ns < mCurrentSensorList[list_index].minDelay * 1000)
        period_ns = mCurrentSensorList[list_index].minDelay * 1000;

    /* just stream with no error return, if the sensor does not support batch mode */
    if (mCurrentSensorList[list_index].fifoMaxEventCount != 0) {
        LOGV_IF(PROCESS_VERBOSE, "HAL: batch - select sensor (handle %d)", list_index);
    } else if (timeout > 0) {
        LOGV_IF(PROCESS_VERBOSE, "HAL: sensor (handle %d) does not support batch mode", list_index);
        timeout = 0;
    }

    /* return from here when dry run */
    if (flags & SENSORS_BATCH_DRY_RUN) {
        return 0;
    }

    if (timeout == 0) {
        mBatchEnabled &= ~(1LL << what);
        mBatchTimeouts[what] = 100000000000LL;
    } else {
        mBatchEnabled |= (1LL << what);
        mBatchTimeouts[what] = timeout;
    }
    updateBatchTimeout();

    switch (what) {
        case Gyro:
            setGyroRate(period_ns);
            break;
        case Accelerometer:
            setAccelRate(period_ns);
            break;
        case MagneticField:
            setMagRate(period_ns);
            break;
        case Pressure:
            setPressureRate(period_ns);
            break;
    }
    return 0;
}

int MPLSensor::flush(int handle)
{
    VFUNC_LOG;

    std::string sname;
    int what = -1;

    if (!mChipDetected)
        return -EINVAL;

    getHandle(handle, what, sname);
    if (what < 0) {
        LOGE("HAL:flush - what=%d is invalid", what);
        return -EINVAL;
    }

    if (!(mEnabled & (1LL << what))) {
        return -EINVAL;
    }

    LOGV_IF(PROCESS_VERBOSE, "HAL: flush - select sensor %s (handle %d)", sname.c_str(), handle);

    /*write sysfs */
    LOGV_IF(SYSFS_VERBOSE, "HAL:sysfs:echo %d > %s (%lld)",
            handle, mpu.flush_batch, (long long)getTimestamp());

    if (write_sysfs_int(mpu.flush_batch, handle) < 0) {
        LOGE("HAL:ERR can't write flush_batch");
    }

    return 0;
}
