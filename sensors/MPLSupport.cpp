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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "Log.h"
#include "ml_sysfs_helper.h"
#include "SensorBase.h"
#include "MPLSupport.h"

int inv_read_data(char *fname, int *data)
{
    VFUNC_LOG;

    char buf[sizeof(int) * 4];
    int count, fd;

    fd = open(fname, O_RDONLY);
    if(fd < 0) {
        LOGE("HAL:Error opening %s", fname);
        return -1;
    }
    memset(buf, 0, sizeof(buf));
    count = read_attribute_sensor(fd, buf, sizeof(buf));
    if(count < 1) {
        close(fd);
        return -1;
    } else {
        count = sscanf(buf, "%d", data);
        if(count)
            LOGV_IF(SensorBase::EXTRA_VERBOSE, "HAL:Data= %d", *data);
    }
    close(fd);

    return 0;
}

/* This one DOES NOT close FDs for you */
int read_attribute_sensor(int fd, char* data, unsigned int size)
{
    VFUNC_LOG;

    int count = 0;
    if (fd > 0) {
        count = pread(fd, data, size, 0);
        if(count < 1) {
            LOGE("HAL:read fails with error code=%s", strerror(count));
        }
    }

    return count;
}

/**
 *  @brief  Enable a sensor through the sysfs file descriptor
 *          provided.
 *  @note   this function one closes FD after the write
 *  @param  fd
 *              the file descriptor to write into
 *  @param  en
 *              the value to write, typically 1 or 0
 *  @return the errno whenever applicable.
 */
int enable_sysfs_sensor(int fd, int en)
{
    VFUNC_LOG;

    int nb;
    int err = 0;

    char c = en ? '1' : '0';
    nb = write(fd, &c, 1);

    if (nb <= 0) {
        err = errno;
        LOGE("HAL:enable_sysfs_sensor - write %c returned %d (%s / %d)",
             c, nb, strerror(err), err);
    }
    close(fd);


    return -err;
}

/* This one closes FDs for you */
int write_attribute_sensor(int fd, int data)
{
    VFUNC_LOG;

    int num_b = 0;

    if (fd >= 0) {
        char buf[80];
        sprintf(buf, "%d", data);
        num_b = write(fd, buf, strlen(buf) + 1);
        if (num_b <= 0) {
            int err = errno;
            LOGE("HAL:write fd %d returned '%s' (%d)", fd, strerror(err), err);
        } else {
            LOGV_IF(SensorBase::EXTRA_VERBOSE, "HAL:fd=%d write attribute to %d", fd, data);
        }
        close(fd);
    }

    return num_b;
}

int write_attribute_sensor(int fd, char* data)
{
    VFUNC_LOG;

    int num_b = 0;

    if (fd >= 0) {
        num_b = write(fd, data, strlen(data) + 1);
        if (num_b <= 0) {
            int err = errno;
            LOGE("HAL:write fd %d returned '%s' (%d)", fd, strerror(err), err);
        } else {
            LOGV_IF(SensorBase::EXTRA_VERBOSE, "HAL:fd=%d write attribute to %s", fd, data);
        }
        close(fd);
    }

    return num_b;
}

/* This one DOES NOT close FDs for you */
int write_attribute_sensor_continuous(int fd, int data)
{
    VFUNC_LOG;

    int num_b = 0;

    if (fd >= 0) {
        char buf[80];
        sprintf(buf, "%d", data);
        num_b = write(fd, buf, strlen(buf) + 1);
        if (num_b <= 0) {
            int err = errno;
            LOGE("HAL:write fd %d returned '%s' (%d)", fd, strerror(err), err);
        } else {
            LOGV_IF(SensorBase::EXTRA_VERBOSE, "HAL:fd=%d write attribute to %d", fd, data);
        }
    }

    return num_b;
}

int read_sysfs_int(const char *filename, int *var)
{
    int res = 0;
    int ret;
    FILE *sysfsfp;

    sysfsfp = fopen(filename, "r");
    if (sysfsfp == NULL) {
        return -errno;
    }
    ret = fscanf(sysfsfp, "%d\n", var);
    if (ret != 1) {
        res = -errno;
        LOGE("HAL:ERR open file %s to read with error %d", filename, res);
    }
    fclose(sysfsfp);

    return res;
}

int read_sysfs_int_array(const char *filename, int *var)
{
    int res = 0;
    int ret;
    FILE *sysfsfp;

    sysfsfp = fopen(filename, "r");
    if (sysfsfp == NULL) {
        return -errno;
    }
    ret = fscanf(sysfsfp, "%d, %d, %d\n", &var[0], &var[1], &var[2]);
    if (ret < 1) {
        res = -errno;
        LOGE("HAL:ERR open file %s to read with error %d", filename, res);
    }
    fclose(sysfsfp);

    return res;
}

int read_sysfs_int64(const char *filename, int64_t *var)
{
    int res = 0;
    int ret;
    FILE *sysfsfp;

    sysfsfp = fopen(filename, "r");
    if (sysfsfp == NULL) {
        return -errno;
    }
    ret = fscanf(sysfsfp, "%lld\n", (long long*)var);
    if (ret != 1) {
        res = -errno;
        LOGE("HAL:ERR open file %s to read with error %d", filename, res);
    }
    fclose(sysfsfp);

    return res;
}

void convert_int_to_hex_char(int* quat, unsigned char* hex, int numElement)
{
    int bytePosition = 0;
    for (int index = 0; index < numElement; index++) {
        for (int i = 0; i < 4; i++) {
            hex[bytePosition] = (int) ((quat[index] >> (4-1-i) * 8) & 0xFF);
            //LOGI("e%d quat[%d]: %x", index, bytePosition, hex[bytePosition]);
            bytePosition++;
        }
    }
    return;
}

int write_sysfs_int(const char *filename, int var)
{
    int res = 0;
    int ret;
    FILE *sysfsfp;

    sysfsfp = fopen(filename, "w");
    if (sysfsfp == NULL) {
        return -errno;
    }
    ret = fprintf(sysfsfp, "%d\n", var);
    if (ret < 0) {
        res = -errno;
        LOGE("HAL:ERR open file %s to write with error %d", filename, res);
    }
    fclose(sysfsfp);

    return res;
}

int write_sysfs_intint(const char *filename, int64_t var)
{
    int res = 0;
    int ret;
    FILE *sysfsfp;

    sysfsfp = fopen(filename, "w");
    if (sysfsfp == NULL) {
        return -errno;
    }
    ret = fprintf(sysfsfp, "%lld\n", (long long)var);
    if (ret < 0) {
        res = -errno;
        LOGE("HAL:ERR open file %s to write with error %d", filename, res);
    }
    fclose(sysfsfp);

    return res;
}

int fill_dev_full_name_by_prefix(const char* dev_prefix,
                                 char *dev_full_name, int len)
{
    char cand_name[20];
    int prefix_len = strlen(dev_prefix);
    strncpy(cand_name, dev_prefix, sizeof(cand_name) / sizeof(cand_name[0]));

    // try adding a number, 0-9
    for(int cand_postfix = 0; cand_postfix < 10; cand_postfix++) {
        snprintf(&cand_name[prefix_len],
                 sizeof(cand_name) / sizeof(cand_name[0]),
                 "%d", cand_postfix);
        int dev_num = find_type_by_name(cand_name, "iio:device");
        if (dev_num != -ENODEV) {
            strncpy(dev_full_name, cand_name, len);
            return 0;
        }
    }
    // try adding a small letter, a-z
    for(char cand_postfix = 'a'; cand_postfix <= 'z'; cand_postfix++) {
        snprintf(&cand_name[prefix_len],
                 sizeof(cand_name) / sizeof(cand_name[0]),
                 "%c", cand_postfix);
        int dev_num = find_type_by_name(cand_name, "iio:device");
        if (dev_num != -ENODEV) {
            strncpy(dev_full_name, cand_name, len);
            return 0;
        }
    }
    // try adding a capital letter, A-Z
    for(char cand_postfix = 'A'; cand_postfix <= 'Z'; cand_postfix++) {
        snprintf(&cand_name[prefix_len],
                 sizeof(cand_name) / sizeof(cand_name[0]),
                 "%c", cand_postfix);
        int dev_num = find_type_by_name(cand_name, "iio:device");
        if (dev_num != -ENODEV) {
            strncpy(dev_full_name, cand_name, len);
            return 0;
        }
    }
    return 1;
}

int read_sysfs_dir(bool fileMode, const char *sysfs_path)
{
    VFUNC_LOG;

    int res = 0;
    static char full_path[257];
    int fd;
    char buf[sizeof(int) *4];
    int data;

    DIR *dp;
    struct dirent *ep;

    dp = opendir (sysfs_path);

    if(dp && fileMode)
        LOGV_IF(0,"HAL DEBUG: file mode");

    if (dp != NULL)
    {
        LOGI("******************** System Sysfs Dump ***************************");
        LOGV_IF(0,"HAL DEBUG: opened directory %s", sysfs_path);
        while ((ep = readdir (dp))) {
            if(ep != NULL) {
                LOGV_IF(0,"file name %s", ep->d_name);
                if(!strcmp(ep->d_name, ".") || !strcmp(ep->d_name, "..") ||
                         !strcmp(ep->d_name, "uevent") || !strcmp(ep->d_name, "dev") ||
                         !strcmp(ep->d_name, "self_test"))
                    continue;
                snprintf(full_path, sizeof(full_path), "%s/%s", sysfs_path, ep->d_name);
                LOGV_IF(0,"HAL DEBUG: reading %s", full_path);
                fd = open(full_path, O_RDONLY);
                if (fd > -1) {
                    memset(buf, 0, sizeof(buf));
                    res = read_attribute_sensor(fd, buf, sizeof(buf));
                    close(fd);
                    if (res > 0) {
                        res = sscanf(buf, "%d", &data);
                        if (res)
                            LOGI("HAL DEBUG:sysfs:cat %s = %d", full_path, data);
                    } else {
                         LOGV_IF(0,"HAL DEBUG: error reading %s", full_path);
                    }
                } else {
                    LOGV_IF(0,"HAL DEBUG: error opening %s", full_path);
                }
                close(fd);
            }
        }
        closedir(dp);
    } else{
        LOGI("HAL DEBUG: could not open directory %s", sysfs_path);
    }

    return res;
}

int inv_float_to_q16(float *fdata, int *ldata)
{

    if (!fdata || !ldata)
        return -1;
    ldata[0] = (int)(fdata[0] * 65536.f);
    ldata[1] = (int)(fdata[1] * 65536.f);
    ldata[2] = (int)(fdata[2] * 65536.f);
    return 0;
}

int inv_int_to_q16(int *fdata, int *ldata)
{

    if (!fdata || !ldata)
        return -1;
    ldata[0] = (fdata[1] * 65536.f);
    ldata[1] = (fdata[2] * 65536.f);
    ldata[2] = (fdata[3] * 65536.f);
    return 0;
}

int inv_float_to_round(float *fdata, int *ldata)
{

    if (!fdata || !ldata)
            return -1;
    ldata[0] = (int)fdata[0];
    ldata[1] = (int)fdata[1];
    ldata[2] = (int)fdata[2];
    return 0;
}

int inv_float_to_round2(float *fdata, short *ldata)
{

    if (!fdata || !ldata)
        return -1;
    ldata[0] = (short)fdata[0];
    ldata[1] = (short)fdata[1];
    ldata[2] = (short)fdata[2];
    return 0;
}

int inv_int_to_float(int *ldata, float *fdata)
{

    if (!ldata || !fdata)
        return -1;
    fdata[0] = (float)ldata[0];
    fdata[1] = (float)ldata[1];
    fdata[2] = (float)ldata[2];
    return 0;
}

void inv_calculate_bias(float *cal, float *raw, float *bias)
{
    if (!cal || !raw)
        return;

    bias[0] = raw[0] - cal[0];
    bias[1] = raw[1] - cal[1];
    bias[2] = raw[2] - cal[2];
    return ;
}
