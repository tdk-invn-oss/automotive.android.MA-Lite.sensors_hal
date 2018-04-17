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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#ifdef __ANDROID__
#include <cutils/properties.h>
#include <utils/SystemClock.h>
#else
#include <time.h>
#endif

#include "SensorBase.h"

/*****************************************************************************/

bool SensorBase::PROCESS_VERBOSE = false;
bool SensorBase::SYSFS_VERBOSE = false;

bool SensorBase::FUNC_ENTRY = false;
bool SensorBase::HANDLER_ENTRY = false;
bool SensorBase::INPUT_DATA = false;
bool SensorBase::HANDLER_DATA = false;

SensorBase::SensorBase(void)
{
#ifdef __ANDROID__
    char value[PROPERTY_VALUE_MAX];
    property_get("invn.hal.verbose.basic", value, "0");
    if (atoi(value)) {
        PROCESS_VERBOSE = true;
    }
    property_get("invn.hal.verbose.sysfs", value, "0");
    if (atoi(value)) {
        SYSFS_VERBOSE = true;
    }
    property_get("invn.hal.entry.function", value, "0");
    if (atoi(value)) {
        FUNC_ENTRY = true;
    }
    property_get("invn.hal.entry.handler", value, "0");
    if (atoi(value)) {
        HANDLER_ENTRY = true;
    }
    property_get("invn.hal.data.input", value, "0");
    if (atoi(value)) {
        INPUT_DATA = true;
    }
    property_get("invn.hal.data.handler", value, "0");
    if (atoi(value)) {
        HANDLER_DATA = true;
    }
#endif
}

SensorBase::~SensorBase()
{
}

int SensorBase::readEvents(sensors_event_t* data, int count)
{
    (void)data; (void)count;
    return 0;
}

int SensorBase::readSample(int *data, int64_t *timestamp, int len)
{
    (void)data; (void)timestamp; (void)len;
    return 0;
}

int SensorBase::getFd(void) const
{
    return 0;
}

int SensorBase::enable(int32_t handle, int enabled)
{
    (void)handle; (void)enabled;
    return 0;
}

int SensorBase::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
    (void)handle; (void)flags; (void)period_ns; (void)timeout;
    return 0;
}

int SensorBase::flush(int handle)
{
    (void)handle;
    return 0;
}

int SensorBase::setDelay(int handle, int64_t period_ns)
{
    (void)handle; (void)period_ns;
    return 0;
}

void SensorBase::getOrientationMatrix(int8_t *orient)
{
    (void)orient;
}

int64_t SensorBase::getTimestamp(void)
{
    int64_t ts;

#ifdef __ANDROID__
    ts = android::elapsedRealtimeNano();
#else
    struct timespec res;

    clock_gettime(CLOCK_BOOTTIME, &res);
    ts = (int64_t)res.tv_sec * 1000000000LL + (int64_t)res.tv_nsec;
#endif

    return ts;
}

void SensorBase::enableIIOSysfs(void)
{
}

int SensorBase::initSysfsAttr(void)
{
    return 0;
}

