/*
 * Copyright (C) 2017-2019 InvenSense, Inc.
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

#ifndef _INV_IIO_BUFFER_H_
#define _INV_IIO_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdbool.h>

/**
 *  struct inv_iio_buffer_channel - iio buffer channel data information
 *  @is_enabled:	Is the channel enabled or not
 *  @index:		Index ordering of the channel inside the buffer
 *  @size:		Data size in bytes
 *  @bits:		Number of used bits in the data
 *  @shift:		Data number of bits to shift right
 *  @is_signed:		Is data a signed number
 *  @is_be:		Is data big endian or little endian
 *  @offset:		Offset to apply to data
 *  @scale:		Scale to apply to data after appyling offset
 */
struct inv_iio_buffer_channel {
    bool is_enabled;
    unsigned index;
    size_t size;
    size_t bits;
    size_t shift;
    bool is_signed;
    bool is_be;
    double offset;
    double scale;
};

/**
 * inv_iio_buffer_scan_channel - scan a channel information
 * @sysfs_enable:	Path to sysfs scan_elements enable file
 * @sysfs_index:	Path to sysfs scan_elements index file
 * @sysfs_type:		Path to sysfs scan_elements type file
 * @sysfs_offset:	Path to sysfs offset file
 * @sysfs_scale:	Path to sysfs scale file
 * @channel:		Channel data information structure to fill
 */
int inv_iio_buffer_scan_channel(const char *sysfs_enable, const char *sysfs_index, const char *sysfs_type,
                                const char *sysfs_offset, const char *sysfs_scale,
                                struct inv_iio_buffer_channel *channel);

/**
 * inv_iio_buffer_channel_get_data - extract data from a channel
 * @channel:		Channel data information
 * @addr:		Data address in the sample corresponding to this channel
 * @return:		Data extracted from the sample
 */
int64_t inv_iio_buffer_channel_get_data(const struct inv_iio_buffer_channel *channel,
                                        const void *addr);

static inline double inv_iio_buffer_convert_data(const struct inv_iio_buffer_channel *channel,
                                                 int64_t data)
{
    return ((double)data + channel->offset) * channel->scale;
}

#ifdef __cplusplus
}
#endif

#endif  /* _INV_IIO_BUFFER_H_ */
