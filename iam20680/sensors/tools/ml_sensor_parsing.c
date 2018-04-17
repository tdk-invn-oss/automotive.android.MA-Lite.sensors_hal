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

/**
 * @defgroup ML_STORED_DATA
 *
 * @{
 *      @file     ml_stored_data.c
 *      @brief    functions for reading and writing stored data sets.
 *                Typically, these functions process stored calibration data.
 */

#undef LOG_NDEBUG
#define LOG_NDEBUG 1 /* Use 0 to turn on LOGV output */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#undef LOG_TAG
#define LOG_TAG "MPL-sensor_parsing"
#include "log.h"

#include "ml_sensor_parsing.h"

static char left_over_buffer[1024];
static int left_over_size;

unsigned short inv_sensor_parsing(char *in, char *out, int read_size)
{
    unsigned short hdr;
    char *dptr;
    int found_sensor, sensor_size;
    char tmp[32];

    if (!read_size) {
        LOGV("Not read size\n");
        return 0;
    }
    if (read_size > 32) {
        LOGV("read size > 32 size %d\n", read_size);
        return 0;
    }
    if (read_size < 0) {
        left_over_size = 0;
        LOGV("read size < 0 size %d\n", read_size);
        return 0;
    }
    memcpy(&left_over_buffer[left_over_size], in, read_size);
    left_over_size += read_size;
    dptr = left_over_buffer;

    hdr = *((unsigned short *)(dptr));
    found_sensor = false;
    sensor_size = 0;

    LOGV("parsing HDR [%04X] hdr = [%04X] size [%d]\n",hdr & (~1), hdr, left_over_size);

    switch (hdr) {
        case PRESSURE_HDR:
        case PRESSURE_WAKE_HDR:
        case STEP_COUNTER_HDR:
        case STEP_COUNTER_WAKE_HDR:
        case ACCEL_HDR:
        case ACCEL_WAKE_HDR:
        case GYRO_HDR:
        case GYRO_WAKE_HDR:
        case PEDQUAT_HDR:
        case ALS_HDR:
        case ALS_WAKE_HDR:
        case STEP_DETECTOR_HDR:
        case STEP_DETECTOR_WAKE_HDR:
            if (left_over_size >= 16) {
                found_sensor = true;
                sensor_size = 16;
                LOGV("HDR [%04X] founded [16]\n",hdr);
            } else
            {
                LOGV("HDR [%04X] left over size < 16\n", hdr);
            }
            break;
        case COMPASS_HDR:
        case COMPASS_WAKE_HDR:
        case GYRO_CALIB_HDR:
        case GYRO_CALIB_WAKE_HDR:
        case COMPASS_CALIB_HDR:
        case COMPASS_CALIB_WAKE_HDR:
        case GEOMAG_HDR:
        case GEOMAG_WAKE_HDR:
        case SIXQUAT_HDR:
        case SIXQUAT_WAKE_HDR:
        case NINEQUAT_HDR:
        case NINEQUAT_WAKE_HDR:
        case LPQ_HDR:
            if (left_over_size >= 24) {
                found_sensor = true;
                sensor_size = 24;
                LOGV("HDR [%04X] founded [32]\n",hdr);
            } else
            {
                LOGV("HDR [%04X] left over size < 24\n", hdr);
            }
            break;
	case EIS_GYROSCOPE_HDR:
            if (left_over_size >= 32) {
                found_sensor = true;
                sensor_size = 32;
                LOGV("HDR [%04X] founded [32]\n",hdr);
            } else
            {
                LOGV("HDR [%04X] left over size < 24\n", hdr);
            }
            break;
        case EIS_AUTHENTICATION_HDR:
            if (left_over_size >= 8) {
                found_sensor = true;
                sensor_size = 8;
                LOGV("HDR [%04X] founded [32]\n",hdr);
            } else
            {
                LOGV("HDR [%04X] left over size < 24\n", hdr);
            }
            break;
        case ACCEL_ACCURACY_HDR:
        case GYRO_ACCURACY_HDR:
        case COMPASS_ACCURACY_HDR:
        case EMPTY_MARKER:
        case END_MARKER:
            found_sensor = true;
            sensor_size = 8;
            break;
        default:
            left_over_size = 0;
            LOGV("header default error= %04X  %04X", hdr, hdr & (~1));
            break;
    }
    if (found_sensor) {
        memcpy(out, left_over_buffer, sensor_size);
        left_over_size -= sensor_size;
        if (left_over_size) {
            memcpy(tmp, &left_over_buffer[sensor_size], left_over_size);
            memcpy(left_over_buffer, tmp, left_over_size);
        }
        return hdr;
    } else {
        LOGV("HAL Cannot find sensor in parsing function\n");
        return 0;
    }
}

void inv_sensor_parsing_reset(void)
{
    left_over_size = 0;
}

int inv_sensor_parsing_get_size(void)
{
    return left_over_size;
}
/**
 *  @}
 */
