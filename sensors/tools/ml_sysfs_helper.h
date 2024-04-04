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

#ifndef MLDMP_SYSFS_HELPER_H__
#define MLDMP_SYSFS_HELPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mltypes.h"

int find_type_by_name(const char *name, const char *type);
int find_name_by_sensor_type(const char *sensor_type, const char *type, char *sensor_name);
inv_error_t inv_get_sysfs_path(char *name);
inv_error_t inv_get_sysfs_abs_path(char *name);
inv_error_t inv_get_dmpfile(char *name);
inv_error_t inv_get_chip_name(char *name);
inv_error_t inv_get_sysfs_key(unsigned char *key);
inv_error_t inv_get_handler_number(const char *name, int *num);
inv_error_t inv_get_input_number(const char *name, int *num);
inv_error_t inv_get_iio_trigger_path(const char *name);
inv_error_t inv_get_iio_device_node(const char *name);
//inv_error_t inv_get_soft_iron_matrix(int *in_orient, int *soft_iron);
//inv_error_t inv_get_compass_sens( int *compassSens);

#ifdef __cplusplus
}
#endif
#endif	/* MLDMP_SYSFS_HELPER_H__ */
