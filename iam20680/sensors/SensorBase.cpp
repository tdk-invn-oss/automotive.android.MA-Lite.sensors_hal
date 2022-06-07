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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/select.h>
#include <time.h>
#include <linux/input.h>

#include <cutils/properties.h>
#ifdef __ANDROID__
#include <utils/SystemClock.h>
#endif

#include "SensorBase.h"

/*****************************************************************************/

#if 0
bool SensorBase::PROCESS_VERBOSE = true;
bool SensorBase::EXTRA_VERBOSE = true;
bool SensorBase::SYSFS_VERBOSE = true;

bool SensorBase::FUNC_ENTRY = true;
bool SensorBase::HANDLER_ENTRY = true;
bool SensorBase::ENG_VERBOSE = true;
bool SensorBase::INPUT_DATA = true;
bool SensorBase::HANDLER_DATA = true;
#else
bool SensorBase::PROCESS_VERBOSE = false;
bool SensorBase::EXTRA_VERBOSE = false;
bool SensorBase::SYSFS_VERBOSE = false;

bool SensorBase::FUNC_ENTRY = false;
bool SensorBase::HANDLER_ENTRY = false;
bool SensorBase::ENG_VERBOSE = false;
bool SensorBase::INPUT_DATA = false;
bool SensorBase::HANDLER_DATA = false;
#endif

SensorBase::SensorBase(const char* dev_name,
        const char* data_name)
    : dev_name(dev_name),
    data_name(data_name),
    dev_fd(-1),
    data_fd(-1)
{
    if (data_name) {
        data_fd = openInput(data_name);
    }

    char value[PROPERTY_VALUE_MAX];
    property_get("vendor.invn.hal.verbose.basic", value, "0");
    if (atoi(value)) {
        PROCESS_VERBOSE = true;
    }
    property_get("vendor.invn.hal.verbose.extra", value, "0");
    if (atoi(value)) {
        EXTRA_VERBOSE = true;
    }
    property_get("vendor.invn.hal.verbose.sysfs", value, "0");
    if (atoi(value)) {
        SYSFS_VERBOSE = true;
    }
    property_get("vendor.invn.hal.verbose.engineering", value, "0");
    if (atoi(value)) {
        ENG_VERBOSE = true;
    }
    property_get("vendor.invn.hal.entry.function", value, "0");
    if (atoi(value)) {
        FUNC_ENTRY = true;
    }
    property_get("vendor.invn.hal.entry.handler", value, "0");
    if (atoi(value)) {
        HANDLER_ENTRY = true;
    }
    property_get("vendor.invn.hal.data.input", value, "0");
    if (atoi(value)) {
        INPUT_DATA = true;
    }
    property_get("vendor.invn.hal.data.handler", value, "0");
    if (atoi(value)) {
        HANDLER_DATA = true;
    }
}

SensorBase::~SensorBase()
{
    if (data_fd >= 0) {
        close(data_fd);
    }
    if (dev_fd >= 0) {
        close(dev_fd);
    }
}

int SensorBase::open_device()
{
    if (dev_fd<0 && dev_name) {
        dev_fd = open(dev_name, O_RDONLY);
        LOGE_IF(dev_fd<0, "Couldn't open %s (%s)", dev_name, strerror(errno));
    }
    return 0;
}

int SensorBase::close_device()
{
    if (dev_fd >= 0) {
        close(dev_fd);
        dev_fd = -1;
    }
    return 0;
}

int SensorBase::getFd() const
{
    if (!data_name) {
        return dev_fd;
    }
    return data_fd;
}

int64_t SensorBase::getTimestamp()
{
#ifdef __ANDROID__
    return android::elapsedRealtimeNano();
#else
    struct timespec t;
    clock_gettime(CLOCK_BOOTTIME, &t);
    return int64_t(t.tv_sec) * 1000000000LL + t.tv_nsec;
#endif
}

int SensorBase::openInput(const char *inputName)
{
    int fd = -1;
    const char *dirname = "/dev/input";
    char devname[PATH_MAX];
    char *filename;
    DIR *dir;
    struct dirent *de;
    dir = opendir(dirname);
    if(dir == NULL)
        return -1;
    strcpy(devname, dirname);
    filename = devname + strlen(devname);
    *filename++ = '/';
    while((de = readdir(dir))) {
        if(de->d_name[0] == '.' &&
                (de->d_name[1] == '\0' ||
                        (de->d_name[1] == '.' && de->d_name[2] == '\0')))
            continue;
        strcpy(filename, de->d_name);
        fd = open(devname, O_RDONLY);
        LOGV_IF(EXTRA_VERBOSE, "path open %s", devname);
        LOGI("path open %s", devname);
        if (fd >= 0) {
            char name[80];
            if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
                name[0] = '\0';
            }
            LOGV_IF(EXTRA_VERBOSE, "name read %s", name);
            if (!strcmp(name, inputName)) {
                strcpy(input_name, filename);
                break;
            } else {
                close(fd);
                fd = -1;
            }
        }
    }
    closedir(dir);
    LOGE_IF(fd < 0, "couldn't find '%s' input device", inputName);
    return fd;
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

void SensorBase::getOrientationMatrix(int8_t *orient)
{
    (void)orient;
}

int SensorBase::isSensorPresent(void)
{
    return 0;
}

int SensorBase::populateSensorList(struct sensor_t *list, int len)
{
    (void)list; (void)len;
    return 0;
}

