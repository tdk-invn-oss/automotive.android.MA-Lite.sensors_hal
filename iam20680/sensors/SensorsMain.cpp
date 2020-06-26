/*
 * Copyright (C) 2014-2019 InvenSense, Inc.
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

#define FUNC_LOG LOGV("%s", __PRETTY_FUNCTION__)

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "Log.h"

#include "InvnSensors.h"
#include "MPLSensor.h"

/*****************************************************************************/
/* The SENSORS Module */

#define LOCAL_SENSORS (TotalNumSensors)

static struct sensor_t sSensorList[LOCAL_SENSORS];
static int sensors = (sizeof(sSensorList) / sizeof(sensor_t));

static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device);

static int sensors__get_sensors_list(struct sensors_module_t* module,
                                     struct sensor_t const** list)
{
    (void) module;
    *list = sSensorList;
    return sensors;
}

static struct hw_module_methods_t sensors_module_methods = {
    .open = open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = SENSORS_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id =  SENSORS_HARDWARE_MODULE_ID,
        .name = "Invensense module",
        .author = "Invensense Inc.",
        .methods = &sensors_module_methods,
        .dso = NULL,
        .reserved = {0}
    },
    .get_sensors_list = sensors__get_sensors_list,
    .set_operation_mode = NULL,
};

struct sensors_poll_context_t {
    sensors_poll_device_1_t device; // must be first

    sensors_poll_context_t();
    ~sensors_poll_context_t();
    int activate(int handle, int enabled);
    int pollEvents(sensors_event_t* data, int count);
    int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
    int flush(int handle);

private:
    enum {
        mpl = 0,
        compass,
        numFds,
    };

    struct pollfd mPollFds[numFds];
    SensorBase *mSensor;
    CompassSensor *mCompassSensor;
};

/******************************************************************************/

sensors_poll_context_t::sensors_poll_context_t() {
    VFUNC_LOG;

#ifdef COMPASS_SUPPORT
    mCompassSensor = new CompassSensor();
#else
    mCompassSensor = NULL;
#endif
    MPLSensor *mplSensor = new MPLSensor(mCompassSensor);

    // populate the sensor list
    sensors =
            mplSensor->populateSensorList(sSensorList, sizeof(sSensorList));

    mSensor = mplSensor;
    mPollFds[mpl].fd = mSensor->getFd();
    mPollFds[mpl].events = POLLIN;
    mPollFds[mpl].revents = 0;

    if (mCompassSensor) {
        mPollFds[compass].fd = mCompassSensor->getFd();
        mPollFds[compass].events = POLLIN;
        mPollFds[compass].revents = 0;
    }
}

sensors_poll_context_t::~sensors_poll_context_t() {
    FUNC_LOG;
    int i, num;

    if (mCompassSensor == NULL)
        num = numFds - 1;
    else
        num = numFds;

    delete mSensor;
    if (mCompassSensor)
        delete mCompassSensor;

    for (i = 0; i < num; i++) {
        close(mPollFds[i].fd);
    }
}

int sensors_poll_context_t::activate(int handle, int enabled) {
    FUNC_LOG;

    int err;
    err = mSensor->enable(handle, enabled);
    return err;
}

int sensors_poll_context_t::pollEvents(sensors_event_t *data, int count)
{
    VHANDLER_LOG;

    int nbEvents = 0;
    int nb, polltime = -1;
    int num;

    if (mCompassSensor == NULL)
        num = numFds - 1;
    else
        num = numFds;

    // look for new events
    do {
        nb = poll(mPollFds, num, polltime);
        LOGI_IF(0, "poll nb=%d, count=%d, pt=%d", nb, count, polltime);
        if (nb < 0)
            return -errno;
        if (nb > 0) {
            for (int i = 0; count && i < num; i++) {
                if (mPollFds[i].revents & (POLLIN | POLLPRI)) {
                    nb = 0;
                    if (i == mpl) {
                        nb = ((MPLSensor*) mSensor)->readMpuEvents(data, count);
                        mPollFds[i].revents = 0;
                    } else if (i == compass) {
                        nb = ((MPLSensor*) mSensor)->readCompassEvents(data, count);
                        mPollFds[i].revents = 0;
                    }
                    if (nb > 0) {
                        count -= nb;
                        nbEvents += nb;
                        data += nb;
                    }
                }
            }
        }
    } while (nbEvents == 0);

    return nbEvents;
}

int sensors_poll_context_t::batch(int handle, int flags, int64_t period_ns,
                                  int64_t timeout)
{
    FUNC_LOG;
    return mSensor->batch(handle, flags, period_ns, timeout);
}

int sensors_poll_context_t::flush(int handle)
{
    FUNC_LOG;
    return mSensor->flush(handle);
}


/******************************************************************************/

static int poll__close(struct hw_device_t *dev)
{
    FUNC_LOG;
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    if (ctx) {
        delete ctx;
    }
    return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
                          int handle, int enabled)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->activate(handle, enabled);
}

static int poll__poll(struct sensors_poll_device_t *dev,
                      sensors_event_t* data, int count)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->pollEvents(data, count);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
                      int handle, int64_t period_ns)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->batch(handle, 0, period_ns, 0);
}

static int poll__batch(struct sensors_poll_device_1 *dev,
                      int handle, int flags, int64_t period_ns, int64_t timeout)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->batch(handle, flags, period_ns, timeout);
}

static int poll__flush(struct sensors_poll_device_1 *dev,
                      int handle)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->flush(handle);
}

/******************************************************************************/

/** Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* id,
                        struct hw_device_t** device)
{
    FUNC_LOG;
    int status = -EINVAL;
    (void) id;

    sensors_poll_context_t *dev = new sensors_poll_context_t();

    memset(&dev->device, 0, sizeof(sensors_poll_device_1));

    dev->device.common.tag          = HARDWARE_DEVICE_TAG;
    dev->device.common.version      = SENSORS_DEVICE_API_VERSION_1_3;
    dev->device.common.module       = const_cast<hw_module_t*>(module);
    dev->device.common.close        = poll__close;
    dev->device.activate            = poll__activate;
    dev->device.poll                = poll__poll;
    dev->device.setDelay            = poll__setDelay;
    dev->device.batch               = poll__batch;
    dev->device.flush               = poll__flush;
    *device = &dev->device.common;
    status = 0;

    return status;
}
