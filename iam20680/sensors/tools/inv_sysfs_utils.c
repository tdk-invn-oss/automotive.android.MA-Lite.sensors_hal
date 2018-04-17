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

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <dirent.h>
#include <errno.h>
#include <unistd.h>
#include "inv_sysfs_utils.h"

/* General TODO list:
 * Select more reasonable string lengths or use fseek and malloc.
 */

/**
 *  inv_sysfs_write() - Write an integer to a file.
 *  @filename:	Path to file.
 *  @data:	Value to write to file.
 *  Returns number of bytes written or a negative error code.
 */
int inv_sysfs_write(char *filename, int data)
{
	FILE *fp;
	int count;

	if (!filename)
		return -1;
	fp = fopen(filename, "w");
	if (!fp)
		return -errno;
	count = fprintf(fp, "%d", data);
	fclose(fp);
	return count;
}

/**
 *  inv_sysfs_read() - Read a string from a file.
 *  @filename:	Path to file.
 *  @num_bytes:	Number of bytes to read.
 *  @data:	Data from file.
 *  Returns number of bytes written or a negative error code.
 */
int inv_sysfs_read(char *filename, int num_bytes, char *data)
{
	FILE *fp;
	int count;

	if (!filename)
		return -1;
	fp = fopen(filename, "r");
	if (!fp)
		return -errno;
	count = fread(data, 1, num_bytes, fp);
	fclose(fp);
	return count;
}

/**
 *  inv_read_buffer() - Read data from ring buffer.
 *  @fd:	File descriptor for buffer file.
 *  @data:	Data in hardware units.
 *  @timestamp:	Time when data was read from device. Use NULL if unsupported.
 *  Returns number of bytes written or a negative error code.
 */
int inv_read_buffer(int fd, int *data, long long *timestamp)
{
	char str[35];
	int count;

	count = read(fd, str, sizeof(str));
	if (!count)
		return count;
	if (!timestamp)
		count = sscanf(str, "%d%d%d", &data[0], &data[1], &data[2]);
	else
		count = sscanf(str, "%d%d%d%lld", &data[0], &data[1],
			&data[2], timestamp);

	if (count < (timestamp?4:3))
		return -EAGAIN;
	return count;
}

/**
 *  inv_read_raw() - Read raw data.
 *  @names:	Names of sysfs files.
 *  @data:	Data in hardware units.
 *  @timestamp:	Time when data was read from device. Use NULL if unsupported.
 *  Returns number of bytes written or a negative error code.
 */
int inv_read_raw(const struct inv_sysfs_names_s *names, int *data, 
	long long *timestamp)
{
	char str[40];
	int count;

	count = inv_sysfs_read((char*)names->raw_data, sizeof(str), str);
	if (count < 0)
		return count;
	if (!timestamp)
		count = sscanf(str, "%d%d%d", &data[0], &data[1], &data[2]);
	else
		count = sscanf(str, "%d%d%d%lld", &data[0], &data[1],
			&data[2], timestamp);
	if (count < (timestamp?4:3))
		return -EAGAIN;
	return count;
}

/**
 *  inv_read_temperature_raw() - Read temperature.
 *  @names:	Names of sysfs files.
 *  @data:	Data in hardware units.
 *  @timestamp:	Time when data was read from device.
 *  Returns number of bytes written or a negative error code.
 */
int inv_read_temperature_raw(const struct inv_sysfs_names_s *names, short *data,
	long long *timestamp)
{
	char str[25];
	int count;

	count = inv_sysfs_read((char*)names->temperature, sizeof(str), str);
	if (count < 0)
		return count;
	count = sscanf(str, "%hd%lld", &data[0], timestamp);
	if (count < 2)
		return -EAGAIN;
	return count;
}

/**
 *  inv_read_fifo_rate() - Read fifo rate.
 *  @names:	Names of sysfs files.
 *  @data:	Fifo rate.
 *  Returns number of bytes written or a negative error code.
 */
int inv_read_fifo_rate(const struct inv_sysfs_names_s *names, short *data)
{
	char str[8];
	int count;

	count = inv_sysfs_read((char*)names->fifo_rate, sizeof(str), str);
	if (count < 0)
		return count;
	count = sscanf(str, "%hd", data);
	if (count < 1)
		return -EAGAIN;
	return count;
}

/**
 *  inv_read_power_state() - Read power state.
 *  @names:	Names of sysfs files.
 *  @data:	1 if device is on.
 *  Returns number of bytes written or a negative error code.
 */
int inv_read_power_state(const struct inv_sysfs_names_s *names, char *data)
{
	char str[2];
	int count;

	count = inv_sysfs_read((char*)names->power_state, sizeof(str), str);
	if (count < 0)
		return count;
	count = sscanf(str, "%hd", (short*)data);
	if (count < 1)
		return -EAGAIN;
	return count;
}

/**
 *  inv_read_scale() - Read scale.
 *  @names:	Names of sysfs files.
 *  @data:	1 if device is on.
 *  Returns number of bytes written or a negative error code.
 */
int inv_read_scale(const struct inv_sysfs_names_s *names, float *data)
{
	char str[5];
	int count;

	count = inv_sysfs_read((char*)names->scale, sizeof(str), str);
	if (count < 0)
		return count;
	count = sscanf(str, "%f", data);
	if (count < 1)
		return -EAGAIN;
	return count;
}

/**
 *  inv_read_temp_scale() - Read temperature scale.
 *  @names:	Names of sysfs files.
 *  @data:	1 if device is on.
 *  Returns number of bytes written or a negative error code.
 */
int inv_read_temp_scale(const struct inv_sysfs_names_s *names, short *data)
{
	char str[4];
	int count;

	count = inv_sysfs_read((char*)names->temp_scale, sizeof(str), str);
	if (count < 0)
		return count;
	count = sscanf(str, "%hd", data);
	if (count < 1)
		return -EAGAIN;
	return count;
}

/**
 *  inv_read_temp_offset() - Read temperature offset.
 *  @names:	Names of sysfs files.
 *  @data:	1 if device is on.
 *  Returns number of bytes written or a negative error code.
 */
int inv_read_temp_offset(const struct inv_sysfs_names_s *names, short *data)
{
	char str[4];
	int count;

	count = inv_sysfs_read((char*)names->temp_offset, sizeof(str), str);
	if (count < 0)
		return count;
	count = sscanf(str, "%hd", data);
	if (count < 1)
		return -EAGAIN;
	return count;
}

/**
 *  inv_read_q16() - Get data as q16 fixed point.
 *  @names:	Names of sysfs files.
 *  @data:	1 if device is on.
 *  @timestamp:	Time when data was read from device.
 *  Returns number of bytes written or a negative error code.
 */
int inv_read_q16(const struct inv_sysfs_names_s *names, int *data,
	long long *timestamp)
{
	int count;
	short raw[3];
	float scale;
	count = inv_read_raw(names, (int*)raw, timestamp);
	count += inv_read_scale(names, &scale);
	data[0] = (int)(raw[0] * (65536.f / scale));
	data[1] = (int)(raw[1] * (65536.f / scale));
	data[2] = (int)(raw[2] * (65536.f / scale));
	return count;
}

/**
 *  inv_read_q16() - Get temperature data as q16 fixed point.
 *  @names:	Names of sysfs files.
 *  @data:	1 if device is on.
 *  @timestamp:	Time when data was read from device.
 *  Returns number of bytes read or a negative error code.
 */
int inv_read_temp_q16(const struct inv_sysfs_names_s *names, int *data,
	long long *timestamp)
{
	int count = 0;
	short raw;
	static short scale, offset;
	static unsigned char first_read = 1;

	if (first_read) {
		count += inv_read_temp_scale(names, &scale);
		count += inv_read_temp_offset(names, &offset);
		first_read = 0;
	}
	count += inv_read_temperature_raw(names, &raw, timestamp);
	data[0] = (int)((35 + ((float)(raw - offset) / scale)) * 65536.f);

	return count;
}

/**
 *  inv_write_fifo_rate() - Write fifo rate.
 *  @names:	Names of sysfs files.
 *  @data:	Fifo rate.
 *  Returns number of bytes written or a negative error code.
 */
int inv_write_fifo_rate(const struct inv_sysfs_names_s *names, short data)
{
	return inv_sysfs_write((char*)names->fifo_rate, (int)data);
}

/**
 *  inv_write_buffer_enable() - Enable/disable buffer in /dev.
 *  @names:	Names of sysfs files.
 *  @data:	Fifo rate.
 *  Returns number of bytes written or a negative error code.
 */
int inv_write_buffer_enable(const struct inv_sysfs_names_s *names, char data)
{
	return inv_sysfs_write((char*)names->enable, (int)data);
}

/**
 *  inv_write_power_state() - Turn device on/off.
 *  @names:	Names of sysfs files.
 *  @data:	1 to turn on.
 *  Returns number of bytes written or a negative error code.
 */
int inv_write_power_state(const struct inv_sysfs_names_s *names, char data)
{
	return inv_sysfs_write((char*)names->power_state, (int)data);
}



