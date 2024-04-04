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

#undef MPL_LOG_NDEBUG
#define MPL_LOG_NDEBUG 1 /* Use 0 to turn on MPL_LOGV output */
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MLLITE"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ml_sysfs_helper.h"
#include <dirent.h>
#include <ctype.h>
#include "log.h"

#define MPU_SYSFS_ABS_PATH "/sys/class/invensense/mpu"

enum PROC_SYSFS_CMD {
	CMD_GET_SYSFS_PATH,
	CMD_GET_DMP_PATH,
	CMD_GET_CHIP_NAME,
	CMD_GET_SYSFS_KEY,
	CMD_GET_TRIGGER_PATH,
	CMD_GET_DEVICE_NODE
};
static char sysfs_path[128];
static char *chip_name[] = {
    "ITG3500",
    "MPU6050",
    "MPU9150",
    "MPU3050",
    "MPU6500",
    "MPU9250",
    "MPU6XXX",
    "MPU9350",
    "MPU6515",
    "MPU7400",
    "ICM20728",
    "ICM20628",
    "ICM20645",
    "ICM10340",
    "ICM20648",
    "ICM20608D",
    "ICM20609I",
    "ICM20602",
    "ICM20690",
    "IAM20680",
    "ICM42600",
    "ICM42686",
    "ICM43600",
    "ICM45600",
};
static int chip_ind;
static int initialized =0;
static int status = 0;
static int iio_initialized = 0;
static int iio_dev_num = 0;

#define IIO_MAX_NAME_LENGTH 30

#define FORMAT_SCAN_ELEMENTS_DIR "%s/scan_elements"
#define FORMAT_TYPE_FILE "%s_type"

#define CHIP_NUM ARRAY_SIZE(chip_name)

static const char *iio_dir = "/sys/bus/iio/devices/";

/**
 * find_type_by_name() - function to match top level types by name
 * @name: top level type instance name
 * @type: the type of top level instance being sort
 *
 * Typical types this is used for are device and trigger.
 **/
int find_type_by_name(const char *name, const char *type)
{
	const struct dirent *ent;
	int number, numstrlen;

	FILE *nameFile;
	DIR *dp;
	char thisname[IIO_MAX_NAME_LENGTH];
	char *filename;
	size_t filename_sz;
	int ret;
	int status = -ENODEV;

	dp = opendir(iio_dir);
	if (dp == NULL) {
		MPL_LOGE("No industrialio devices available");
		return -ENODEV;
	}

	while (ent = readdir(dp), ent != NULL) {
		if (strcmp(ent->d_name, ".") != 0 &&
			strcmp(ent->d_name, "..") != 0 &&
			strlen(ent->d_name) > strlen(type) &&
			strncmp(ent->d_name, type, strlen(type)) == 0) {
			numstrlen = sscanf(ent->d_name + strlen(type),
					   "%d",
					   &number);
			/* verify the next character is not a colon */
			if (strncmp(ent->d_name + strlen(type) + numstrlen,
					":",
					1) != 0) {
				filename_sz = strlen(iio_dir)
						+ strlen(type)
						+ numstrlen
						+ 6;
				filename = malloc(filename_sz);
				if (filename == NULL) {
					status = -ENOMEM;
					goto exit_closedir;
				}
				snprintf(filename, filename_sz, "%s%s%d/name",
					iio_dir,
					type,
					number);
				nameFile = fopen(filename, "r");
				free(filename);
				if (!nameFile)
					continue;
				ret = fscanf(nameFile, "%29s", thisname);
				fclose(nameFile);
				if (ret == 1 && strcmp(name, thisname) == 0) {
					status = number;
					goto exit_closedir;
				}
			}
		}
	}

exit_closedir:
	closedir(dp);
	return status;
}

/* mode 0: search for which chip in the system and fill sysfs path
   mode 1: return event number
 */
static int parsing_proc_input(int mode, char *name){
	const char input[] = "/proc/bus/input/devices";
	static char line[4096];
	char d;
	char tmp[100];
	FILE *fp;
	int i, j, result, find_flag;
	int event_number = -1;
	int input_number = -1;

	if(NULL == (fp = fopen(input, "rt")) ){
		return -1;
	}
	result = 1;
	find_flag = 0;
	while(result != 0 && find_flag < 2){
		i = 0;
		d = 0;
		memset(line, 0, 100);
		while(d != '\n'){
			result = fread(&d, 1, 1, fp);
			if(result == 0){
				line[0] = 0;
				break;
			}
			sprintf(&line[i], "%c", d);
			i ++;
		}
		if(line[0] == 'N'){
			i = 1;
			while(line[i] != '"'){
				i++;
			}
			i++;
			j = 0;
			find_flag = 0;
			if (mode == 0){
				while(j < (int)CHIP_NUM){
					if(!memcmp(&line[i], chip_name[j], strlen(chip_name[j]))){
						find_flag = 1;
						chip_ind = j;
					}
					j++;
				}
			} else if (mode  != 0){
				if(!memcmp(&line[i], name, strlen(name))){
					find_flag = 1;
				}
			}
		}
		if(find_flag){
			if(mode == 0){
				if(line[0] == 'S'){
					memset(tmp, 0, 100);
					i =1;
					while(line[i] != '=') i++;
					i++;
					j = 0;
					while(line[i] != '\n'){
						tmp[j] = line[i];
						i ++; j++;
					}
					snprintf(sysfs_path, sizeof(sysfs_path), "%s%s", "/sys", tmp);
					find_flag++;
				}
			} else if(mode == 1){
				if(line[0] == 'H') {
					i = 2;
					while(line[i] != '=') i++;
					while(line[i] != 't') i++;
					i++;
					event_number = 0;
					while(line[i] != '\n'){
						if(line[i] >= '0' && line[i] <= '9')
							event_number = event_number*10 + line[i]-0x30;
						i ++;
					}
					find_flag ++;
				}
			} else if (mode == 2) {
				if(line[0] == 'S'){
					memset(tmp, 0, 100);
					i =1;
					while(line[i] != '=') i++;
					i++;
					j = 0;
					while(line[i] != '\n'){
						tmp[j] = line[i];
						i ++; j++;
					}
					input_number = 0;
					if(tmp[j-2] >= '0' && tmp[j-2] <= '9')
						input_number += (tmp[j-2]-0x30)*10;
					if(tmp[j-1] >= '0' && tmp[j-1] <= '9')
						input_number += (tmp[j-1]-0x30);
					find_flag++;
				}
			}
		}
	}
	fclose(fp);
	if(find_flag == 0){
		return -1;
	}
	if(0 == mode)
		status = 1;
	if (mode == 1)
		return event_number;
	if (mode == 2)
		return input_number;
	return 0;

}
static void init_iio() {
	int i, j;
	char iio_chip[10];
	int dev_num;
	for(j=0; j< (int)CHIP_NUM; j++) {
		for (i=0; i < (int)strlen(chip_name[j]); i++) {
			iio_chip[i] = tolower(chip_name[j][i]);
		}
		iio_chip[strlen(chip_name[j])] = '\0';
		dev_num = find_type_by_name(iio_chip, "iio:device");
		if(dev_num >= 0) {
			iio_initialized = 1;
			iio_dev_num = dev_num;
			chip_ind = j;
		}
	}
}

static int process_sysfs_request(enum PROC_SYSFS_CMD cmd, char *data)
{
	static char key_path[256];
	FILE *fp;
	int i, result, ret;
	if(initialized == 0){
		parsing_proc_input(0, NULL);
		initialized = 1;
	}
	if(initialized && status == 0) {
		init_iio();
		if (iio_initialized == 0)
			return -1;
	}

	memset(key_path, 0, 100);
	switch(cmd){
	case CMD_GET_SYSFS_PATH:
		if (iio_initialized == 1)
			sprintf(data, "/sys/bus/iio/devices/iio:device%d", iio_dev_num);
		else
			sprintf(data, "%s%s", sysfs_path, "/device/invensense/mpu");
		break;
	case CMD_GET_DMP_PATH:
		if (iio_initialized == 1)
			sprintf(data, "/sys/bus/iio/devices/iio:device%d/misc_dmp_firmware", iio_dev_num);
		else
			sprintf(data, "%s%s", sysfs_path, "/device/invensense/mpu/dmp_firmware");
		break;
	case CMD_GET_CHIP_NAME:
		sprintf(data, "%s", chip_name[chip_ind]);
		break;
	case CMD_GET_TRIGGER_PATH:
		sprintf(data, "/sys/bus/iio/devices/trigger%d", iio_dev_num);
		break;
	case CMD_GET_DEVICE_NODE:
		sprintf(data, "/dev/iio:device%d", iio_dev_num);
		break;
	case CMD_GET_SYSFS_KEY:
		memset(key_path, 0, 100);
		if (iio_initialized == 1)
			snprintf(key_path, sizeof(key_path), "/sys/bus/iio/devices/iio:device%d/key", iio_dev_num);
		else
			snprintf(key_path, sizeof(key_path), "%s%s", sysfs_path, "/device/invensense/mpu/key");

		if((fp = fopen(key_path, "rt")) == NULL)
			return -1;
		for(i=0;i<16;i++){
			ret = fscanf(fp, "%02x", &result);
			if (ret != 1)
				result = 0;
			data[i] = (char)result;
		}
		fclose(fp);
		break;
	default:
		break;
	}
	return 0;
}

int find_name_by_sensor_type(const char *sensor_type, const char *type, char *sensor_name)
{
    const struct dirent *ent;
    int number, numstrlen;

    FILE *nameFile;
    DIR *dp;
    char *filename;
    size_t filename_sz;
    int ret;
    int status = -ENODEV;

    dp = opendir(iio_dir);
    if (dp == NULL) {
        MPL_LOGE("No industrialio devices available");
        return -ENODEV;
    }

    while (ent = readdir(dp), ent != NULL) {
        if (strcmp(ent->d_name, ".") != 0 &&
            strcmp(ent->d_name, "..") != 0 &&
            strlen(ent->d_name) > strlen(type) &&
            strncmp(ent->d_name, type, strlen(type)) == 0) {
            numstrlen = sscanf(ent->d_name + strlen(type),
                       "%d",
                       &number);
            /* verify the next character is not a colon */
            if (strncmp(ent->d_name + strlen(type) + numstrlen,
                    ":",
                    1) != 0) {
                filename_sz = strlen(iio_dir)
                        + strlen(type)
                        + numstrlen
                        + 6
                        + strlen(sensor_type);
                filename = malloc(filename_sz);
                if (filename == NULL) {
                    status = -ENOMEM;
                    goto exit_closedir;
                }
                snprintf(filename, filename_sz, "%s%s%d/%s",
                    iio_dir,
                    type,
                    number,
                    sensor_type);
                nameFile = fopen(filename, "r");
                MPL_LOGI("sensor type path: %s\n", filename);
                free(filename);
                //fscanf(nameFile, "%s", thisname);
                //if (strcmp(name, thisname) == 0) {
                if(nameFile == NULL) {
                    MPL_LOGI("keeps searching");
                    continue;
                } else{
                    MPL_LOGI("found directory");
                    fclose(nameFile);
                }
                filename_sz = strlen(iio_dir)
                        + strlen(type)
                        + numstrlen
                        + 6;
                filename = malloc(filename_sz);
                snprintf(filename, filename_sz, "%s%s%d/name",
                    iio_dir,
                    type,
                    number);
                    nameFile = fopen(filename, "r");
                    MPL_LOGI("name path: %s\n", filename);
                    free(filename);
                    if (!nameFile)
                        continue;
                    ret = fscanf(nameFile, "%s", sensor_name);
		    fclose(nameFile);
                    if (ret != 1)
                        continue;
                    MPL_LOGI("name found: %s now test for mpuxxxx", sensor_name);
                    if( !strncmp("mpu",sensor_name, 3) ||
				!strncmp("icm", sensor_name, 3) ) {
                        char secondaryFileName[200];
                    snprintf(secondaryFileName, sizeof(secondaryFileName), "%s%s%d/info_secondary_name",
                        iio_dir,
                        type,
                        number);
                        nameFile = fopen(secondaryFileName, "r");
                        MPL_LOGI("name path: %s\n", secondaryFileName);
                        if(!nameFile)
                            continue;
                        ret = fscanf(nameFile, "%s", sensor_name);
                        fclose(nameFile);
                        if (ret != 1)
                            continue;
                        MPL_LOGI("secondary name found: %s\n", sensor_name);
                    }
		    status = 0;
                    goto exit_closedir;
                //}
            }
        }
    }

exit_closedir:
    closedir(dp);
    return status;
}

/**
 *  @brief  return sysfs key. if the key is not available
 *          return false. So the return value must be checked
 *          to make sure the path is valid.
 *  @unsigned char *name: This should be array big enough to hold the key
 *           It should be zeroed before calling this function.
 *           Or it could have unpredicable result.
 */
inv_error_t inv_get_sysfs_key(unsigned char *key)
{
	if (process_sysfs_request(CMD_GET_SYSFS_KEY, (char*)key) < 0)
		return INV_ERROR_NOT_OPENED;
	else
		return INV_SUCCESS;
}

/**
 *  @brief  return the sysfs path. If the path is not
 *          found yet. return false. So the return value must be checked
 *          to make sure the path is valid.
 *  @unsigned char *name: This should be array big enough to hold the sysfs
 *           path. It should be zeroed before calling this function.
 *           Or it could have unpredicable result.
 */
inv_error_t inv_get_sysfs_path(char *name)
{
	if (process_sysfs_request(CMD_GET_SYSFS_PATH, name) < 0)
		return INV_ERROR_NOT_OPENED;
	else
		return INV_SUCCESS;
}

inv_error_t inv_get_sysfs_abs_path(char *name)
{
    strcpy(name, MPU_SYSFS_ABS_PATH);
    return INV_SUCCESS;
}

/**
 *  @brief  return the dmp file path. If the path is not
 *          found yet. return false. So the return value must be checked
 *          to make sure the path is valid.
 *  @unsigned char *name: This should be array big enough to hold the dmp file
 *           path. It should be zeroed before calling this function.
 *           Or it could have unpredicable result.
 */
inv_error_t inv_get_dmpfile(char *name)
{
   	if (process_sysfs_request(CMD_GET_DMP_PATH, name) < 0)
		return INV_ERROR_NOT_OPENED;
	else
		return INV_SUCCESS;
}
/**
 *  @brief  return the chip name. If the chip is not
 *          found yet. return false. So the return value must be checked
 *          to make sure the path is valid.
 *  @unsigned char *name: This should be array big enough to hold the chip name
 *           path(8 bytes). It should be zeroed before calling this function.
 *           Or it could have unpredicable result.
 */
inv_error_t inv_get_chip_name(char *name)
{
   	if (process_sysfs_request(CMD_GET_CHIP_NAME, name) < 0)
		return INV_ERROR_NOT_OPENED;
	else
		return INV_SUCCESS;
}
/**
 *  @brief  return event handler number. If the handler number is not found
 *          return false. the return value must be checked
 *          to make sure the path is valid.
 *  @unsigned char *name: This should be array big enough to hold the chip name
 *           path(8 bytes). It should be zeroed before calling this function.
 *           Or it could have unpredicable result.
 *  @int *num: event number store
 */
inv_error_t  inv_get_handler_number(const char *name, int *num)
{
	initialized = 0;
	if ((*num = parsing_proc_input(1, (char *)name)) < 0)
		return INV_ERROR_NOT_OPENED;
	else
		return INV_SUCCESS;
}

/**
 *  @brief  return input number. If the handler number is not found
 *          return false. the return value must be checked
 *          to make sure the path is valid.
 *  @unsigned char *name: This should be array big enough to hold the chip name
 *           path(8 bytes). It should be zeroed before calling this function.
 *           Or it could have unpredicable result.
 *  @int *num: input number store
 */
inv_error_t  inv_get_input_number(const char *name, int *num)
{
	initialized = 0;
	if ((*num = parsing_proc_input(2, (char *)name)) < 0)
		return INV_ERROR_NOT_OPENED;
	else {
		return INV_SUCCESS;
	}
}

/**
 *  @brief  return iio trigger name. If iio is not initialized, return false.
 *          So the return must be checked to make sure the numeber is valid.
 *  @unsigned char *name: This should be array big enough to hold the trigger
 *           name. It should be zeroed before calling this function.
 *           Or it could have unpredicable result.
 */
inv_error_t inv_get_iio_trigger_path(const char *name)
{
	if (process_sysfs_request(CMD_GET_TRIGGER_PATH, (char *)name) < 0)
		return INV_ERROR_NOT_OPENED;
	else
		return INV_SUCCESS;
}

/**
 *  @brief  return iio device node. If iio is not initialized, return false.
 *          So the return must be checked to make sure the numeber is valid.
 *  @unsigned char *name: This should be array big enough to hold the device
 *           node. It should be zeroed before calling this function.
 *           Or it could have unpredicable result.
 */
inv_error_t inv_get_iio_device_node(const char *name)
{
	if (process_sysfs_request(CMD_GET_DEVICE_NODE, (char *)name) < 0)
		return INV_ERROR_NOT_OPENED;
	else
		return INV_SUCCESS;
}

/**
 *  @brief  return soft iron matrix based on the sysfs value.
 *  @unsigned char *name: This should be array big enough to hold the device
 *           node. It should be zeroed before calling this function.
 *           Or it could have unpredicable result.
 */
#if 0
inv_error_t inv_get_soft_iron_matrix(int *in_orient, int *soft_iron)
{
	static char final_path[256];
	char soft_path[100], mag_name[100];
	int sens[3], scale, shift, i, ret;
	FILE *fp;

    (void)in_orient;

	if (process_sysfs_request(CMD_GET_SYSFS_PATH, (char *)soft_path) < 0)
		return INV_ERROR_NOT_OPENED;
	snprintf(final_path, sizeof(final_path), "%s/in_magn_sensitivity_x", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%d\n", &sens[0]);
	if (ret != 1)
		sens[0] = 0;
	fclose(fp);

	snprintf(final_path, sizeof(final_path), "%s/in_magn_sensitivity_y", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%d\n", &sens[1]);
	if (ret != 1)
		sens[1] = 0;
	fclose(fp);
	snprintf(final_path, sizeof(final_path), "%s/in_magn_sensitivity_z", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%d\n", &sens[2]);
	if (ret != 1)
		sens[2] = 0;
	fclose(fp);

	snprintf(final_path, sizeof(final_path), "%s/in_magn_scale", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%d\n", &scale);
	if (ret != 1)
		scale = 1;
	fclose(fp);

	snprintf(final_path, sizeof(final_path), "%s/info_secondary_name", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%s\n", mag_name);
	if (ret != 1)
		mag_name[0] = '\0';
	fclose(fp);
	if (!strcmp("AK09911", mag_name))
		shift = 23;
	else
		shift = 22;
	for (i = 0; i < 3; i++) {
		sens[i] +=  128;
		sens[i] = inv_q30_mult(sens[i] << shift, scale);
	}
	for (i = 0; i < 3; i++)
		soft_iron[i] = 0;
	soft_iron[0] = sens[0];
	soft_iron[4] = sens[1];
	soft_iron[8] = sens[2];
	MPL_LOGV("Soft Iron Matrix obtained =%d, %d, %d, name=%s, scale=%d, shift=%d\n", sens[0], sens[1], sens[2], mag_name, scale, shift);

	return 0;
}
#endif

#if 0
inv_error_t inv_get_compass_sens( int *compassSens)
{
	static char final_path[256];
	char soft_path[100], mag_name[100];
	int sens[3], scale, shift, i, ret;
	FILE *fp;

	if (process_sysfs_request(CMD_GET_SYSFS_PATH, (char *)soft_path) < 0)
		return INV_ERROR_NOT_OPENED;
	snprintf(final_path, sizeof(final_path), "%s/in_magn_sensitivity_x", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%d\n", &sens[0]);
	if (ret != 1)
		sens[0] = 0;
	fclose(fp);

	snprintf(final_path, sizeof(final_path), "%s/in_magn_sensitivity_y", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%d\n", &sens[1]);
	if (ret != 1)
		sens[1] = 0;
	fclose(fp);
	snprintf(final_path, sizeof(final_path), "%s/in_magn_sensitivity_z", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%d\n", &sens[2]);
	if (ret != 1)
		sens[2] = 0;
	fclose(fp);

	snprintf(final_path, sizeof(final_path), "%s/in_magn_scale", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%d\n", &scale);
	if (ret != 1)
		scale = 1;
	fclose(fp);

	snprintf(final_path, sizeof(final_path), "%s/info_secondary_name", soft_path);
	if ((fp = fopen(final_path, "r")) == NULL)
		return INV_ERROR_NOT_OPENED;
	ret = fscanf(fp, "%s\n", mag_name);
	if (ret != 1)
		mag_name[0] = '\0';
	fclose(fp);
	if (!strcmp("AK09911", mag_name))
		shift = 23;
	else
		shift = 22;
	for (i = 0; i < 3; i++) {
		sens[i] +=  128;
		sens[i] = inv_q30_mult(sens[i] << shift, scale);
	}

	for (i = 0; i < 3; i++)
		compassSens[i] = sens[i];

	return 0;
}
#endif
