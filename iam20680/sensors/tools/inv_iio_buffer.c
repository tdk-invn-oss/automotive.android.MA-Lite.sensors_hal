/*
 * Copyright (C) 2017-2017 InvenSense, Inc.
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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <endian.h>

#include "inv_iio_buffer.h"

#ifndef __ANDROID__
static inline uint16_t betoh16(uint16_t val)
{
	return be16toh(val);
}

static inline uint16_t letoh16(uint16_t val)
{
	return le16toh(val);
}

static inline uint32_t betoh32(uint32_t val)
{
	return be32toh(val);
}

static inline uint32_t letoh32(uint32_t val)
{
	return le32toh(val);
}

static inline uint64_t betoh64(uint64_t val)
{
	return be64toh(val);
}

static inline uint64_t letoh64(uint64_t val)
{
	return le64toh(val);
}
#endif

int inv_iio_buffer_scan_channel(const char *sysfs_enable, const char *sysfs_index, const char *sysfs_type,
                                const char *sysfs_offset, const char *sysfs_scale,
                                struct inv_iio_buffer_channel *channel)
{
    FILE *file;
    int val;
    double valf;
    char endian, sign;
    unsigned realbits, storagebits, shift;
    int ret;

    /* parse channel enable state */
    file = fopen(sysfs_enable, "r");
    if (file == NULL) {
        goto error;
    }
    ret = fscanf(file, "%d", &val);
    fclose(file);
    if (ret != 1 || val < 0) {
        goto error;
    }
    channel->is_enabled = val ? true : false;

    /* parse channel index */
    file = fopen(sysfs_index, "r");
    if (file == NULL) {
        goto error;
    }
    ret = fscanf(file, "%d", &val);
    fclose(file);
    if (ret != 1 || val < 0) {
        goto error;
    }
    channel->index = val;

    /* parse channel type, ex: le:s16/32>>8 */
    file = fopen(sysfs_type, "r");
    if (file == NULL) {
        goto error;
    }
    ret = fscanf(file, "%ce:%c%u/%u>>%u", &endian, &sign, &realbits, &storagebits, &shift);
    fclose(file);
    if (ret != 5) {
        goto error;
    }
    if (endian == 'b') {
        channel->is_be = true;
    } else if (endian == 'l') {
        channel->is_be = false;
    } else {
        goto error;
    }
    if (sign == 's') {
        channel->is_signed = true;
    } else if (sign == 'u') {
        channel->is_signed = false;
    } else {
        goto error;
    }
    if (realbits <= storagebits) {
        channel->bits = realbits;
    } else {
        goto error;
    }
    switch (storagebits) {
    case 8:
    case 16:
    case 24:
    case 32:
    case 64:
        channel->size = storagebits / 8;
        break;
    default:
        goto error;
    }
    if (shift < storagebits) {
        channel->shift = shift;
    } else {
        goto error;
    }

    /* parse channel offset (optional) */
    channel->offset = 0;
    file = fopen(sysfs_offset, "r");
    if (file != NULL) {
        ret = fscanf(file, "%lf", &valf);
        fclose(file);
        if (ret == 1) {
            channel->offset = valf;
        }
    }

    /* parse channel scale (optional) */
    channel->scale = 1.0;
    file = fopen(sysfs_scale, "r");
    if (file != NULL) {
        ret = fscanf(file, "%lf", &valf);
        fclose(file);
        if (ret == 1) {
            channel->scale = valf;
        }
    }

    return 0;

error:
    // disable channel
    channel->is_enabled = false;
    return -1;
}

static inline uint8_t sample_get_u8(const struct inv_iio_buffer_channel *channel,
                                    const void *addr)
{
    const uint8_t *raw = addr;
    uint8_t value;

    value = *raw;
    value >>= channel->shift;
    value &= (1 << channel->bits);

    return value;
}

static inline int8_t sample_get_s8(const struct inv_iio_buffer_channel *channel,
                                   const void *addr)
{
    const int8_t *raw = addr;
    int8_t value;

    value = *raw;
    value <<= sizeof(value) * 8 - channel->bits - channel->shift;
    value >>= sizeof(value) * 8 - channel->bits + channel->shift;

    return value;
}

static inline uint16_t sample_get_u16(const struct inv_iio_buffer_channel *channel,
                                      const void *addr)
{
    uint16_t raw;
    uint16_t value;

    memcpy(&raw, addr, sizeof(raw));
    if (channel->is_be) {
        value = betoh16(raw);
    } else {
        value = letoh16(raw);
    }
    value >>= channel->shift;
    value &= (1 << channel->bits) - 1;

    return value;
}

static inline int16_t sample_get_s16(const struct inv_iio_buffer_channel *channel,
                                     const void *addr)
{
    uint16_t raw;
    int16_t value;

    memcpy(&raw, addr, sizeof(raw));
    if (channel->is_be) {
        value = betoh16(raw);
    } else {
        value = letoh16(raw);
    }
    value <<= sizeof(value) * 8 - channel->bits - channel->shift;
    value >>= sizeof(value) * 8 - channel->bits + channel->shift;

    return value;
}


static inline uint32_t sample_get_u24(const struct inv_iio_buffer_channel *channel,
                                      const void *addr)
{
    uint8_t *data;
    uint32_t raw = 0;
    uint32_t value;

    if (channel->is_be) {
        data = (uint8_t *)&raw + 1;
        memcpy(data, addr, 3);
        value = betoh32(raw);
    } else {
        memcpy(&raw, addr, 3);
        value = letoh32(raw);
    }
    value >>= channel->shift;
    value &= (1 << channel->bits) - 1;

    return value;
}

static inline int32_t sample_get_s24(const struct inv_iio_buffer_channel *channel,
                                     const void *addr)
{
    uint8_t *data;
    uint32_t raw = 0;
    int32_t value;

    if (channel->is_be) {
        data = (uint8_t *)&raw + 1;
        memcpy(data, addr, 3);
        value = betoh32(raw);
    } else {
        memcpy(&raw, addr, 3);
        value = letoh32(raw);
    }
    value <<= sizeof(value) * 8 - channel->bits - channel->shift;
    value >>= sizeof(value) * 8 - channel->bits + channel->shift;

    return value;
}

static inline uint32_t sample_get_u32(const struct inv_iio_buffer_channel *channel,
                                      const void *addr)
{
    uint32_t raw;
    uint32_t value;

    memcpy(&raw, addr, sizeof(raw));
    if (channel->is_be) {
        value = betoh32(raw);
    } else {
        value = letoh32(raw);
    }
    value >>= channel->shift;
    value &= (1 << channel->bits) - 1;

    return value;
}

static inline int32_t sample_get_s32(const struct inv_iio_buffer_channel *channel,
                                     const void *addr)
{
    uint32_t raw;
    int32_t value;

    memcpy(&raw, addr, sizeof(raw));
    if (channel->is_be) {
        value = betoh32(raw);
    } else {
        value = letoh32(raw);
    }
    value <<= sizeof(value) * 8 - channel->bits - channel->shift;
    value >>= sizeof(value) * 8 - channel->bits + channel->shift;

    return value;
}

static inline uint64_t sample_get_u64(const struct inv_iio_buffer_channel *channel,
                                      const void *addr)
{
    uint64_t raw;
    uint64_t value;

    memcpy(&raw, addr, sizeof(raw));
    if (channel->is_be) {
        value = betoh64(raw);
    } else {
        value = letoh64(raw);
    }
    value >>= channel->shift;
    value &= (1ULL << channel->bits) - 1;

    return value;
}

static inline int64_t sample_get_s64(const struct inv_iio_buffer_channel *channel,
                                     const void *addr)
{
    uint64_t raw;
    int64_t value;

    memcpy(&raw, addr, sizeof(raw));
    if (channel->is_be) {
        value = betoh64(raw);
    } else {
        value = letoh64(raw);
    }
    value <<= sizeof(value) * 8 - channel->bits - channel->shift;
    value >>= sizeof(value) * 8 - channel->bits + channel->shift;

    return value;
}

double inv_iio_buffer_channel_get_data(const struct inv_iio_buffer_channel *channel,
                                       const void *addr)
{
    double val, result;

    switch (channel->size) {
    case 1:
        if (channel->is_signed) {
            val = sample_get_s8(channel, addr);
        } else {
            val = sample_get_u8(channel, addr);
        }
        break;
    case 2:
        if (channel->is_signed) {
            val = sample_get_s16(channel, addr);
        } else {
            val = sample_get_u16(channel, addr);
        }
        break;
    case 3:
        if (channel->is_signed) {
            val = sample_get_s24(channel, addr);
        } else {
            val = sample_get_u24(channel, addr);
        }
        break;
    case 4:
        if (channel->is_signed) {
            val = sample_get_s32(channel, addr);
        } else {
            val = sample_get_u32(channel, addr);
        }
        break;
    case 8:
        if (channel->is_signed) {
            val = sample_get_s64(channel, addr);
        } else {
            val = sample_get_u64(channel, addr);
        }
        break;
    default:
        val = 0;
        break;
    }

    result = (val + channel->offset) * channel->scale;

    return result;
}
