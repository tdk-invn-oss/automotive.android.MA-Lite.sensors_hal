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

#ifndef ANDROID_MPL_SUPPORT_H
#define ANDROID_MPL_SUPPORT_H

#include <stdint.h>

int inv_read_data(char *fname, int *data);
int read_attribute_sensor(int fd, char* data, unsigned int size);
int enable_sysfs_sensor(int fd, int en);
int write_attribute_sensor(int fd, int data);
int write_attribute_sensor(int fd, char* data);
int write_attribute_sensor_continuous(int fd, int data);
int read_sysfs_int64(const char*, int64_t*);
int read_sysfs_int(const char*, int*);
int read_sysfs_int_array(const char*, int*);
int write_sysfs_int(const char*, int);
int write_sysfs_intint(const char*, long long);
int fill_dev_full_name_by_prefix(const char* dev_prefix,
                                 char* dev_full_name, int len);

int read_sysfs_dir(bool fileMode, const char *sysfs_path);

void convert_int_to_hex_char(int* quat, unsigned char* hex, int numElement);
int inv_float_to_q16(float *fdata, int *ldata);
int inv_int_to_q16(int *fdata, int *ldata);
int inv_float_to_round(float *fdata, int *ldata);
int inv_float_to_round2(float *fdata, short *sdata);
int inv_int_to_float(int*ldata, float *fdata);

void inv_calculate_bias(float *cal, float *raw, float *bias);

#endif //  ANDROID_MPL_SUPPORT_H
