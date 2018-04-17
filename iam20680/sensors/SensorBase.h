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

#ifndef ANDROID_SENSOR_BASE_H
#define ANDROID_SENSOR_BASE_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include "log.h"

#define FUNC_LOG \
            LOGV("%s", __PRETTY_FUNCTION__)
#define VFUNC_LOG \
            LOGV_IF(SensorBase::FUNC_ENTRY, \
                    "Entering function '%s'", __PRETTY_FUNCTION__)
#define VHANDLER_LOG \
            LOGV_IF(SensorBase::HANDLER_ENTRY, \
                    "Entering handler '%s'", __PRETTY_FUNCTION__)
#define CALL_MEMBER_FN(pobject, ptrToMember) ((pobject)->*(ptrToMember))

#define MAX_SYSFS_NAME_LEN  (100)
#define IIO_BUFFER_LENGTH   (32768)

/*****************************************************************************/

struct sensors_event_t;

class SensorBase {
public:
    static bool PROCESS_VERBOSE;   /* process log messages */
    static bool SYSFS_VERBOSE;     /* log sysfs interactions as cat/echo for
                                      repro purpose on a shell */
    static bool FUNC_ENTRY;        /* log entry in all one-time functions */
    static bool HANDLER_ENTRY;     /* log entry in all handler functions */
    static bool INPUT_DATA;        /* log the data input from the events */
    static bool HANDLER_DATA;      /* log the data fetched from the handlers */

public:
    SensorBase(void);
    virtual ~SensorBase();
    virtual int readEvents(sensors_event_t* data, int count);
    virtual int readSample(int *data, int64_t *timestamp, int len);
    virtual int getFd(void) const;
    virtual int enable(int32_t handle, int enabled);
    virtual int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
    virtual int flush(int handle);
    virtual int setDelay(int handle, int64_t period_ns);
    virtual void getOrientationMatrix(int8_t *orient);

protected:
    int64_t getTimestamp(void);
    virtual void enableIIOSysfs(void);
    virtual int initSysfsAttr(void);

};

/*****************************************************************************/

#endif  // ANDROID_SENSOR_BASE_H
