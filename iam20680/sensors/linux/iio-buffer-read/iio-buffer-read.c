/*
 * Copyright (C) 2023 InvenSense, Inc.
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

#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <inttypes.h>
#include <poll.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include <getopt.h>
#include <math.h>
#include <string.h>
#include <sys/ioctl.h>

#define IIO_BUFFER_GET_FD_IOCTL                 _IOWR('i', 0x91, int)

#define VERSION_STR             "1.0.0"
#define USAGE_NOTE              ""

#define IIO_BUFFER_LENGTH       32768
#define NS_IN_SEC               1000000000LL

/* char device for sensor data */
#define IIO_DEVICE              "/dev/iio:device%lu"

/* sysfs to control chip */
#define SYSFS_PATH              "/sys/bus/iio/devices/iio:device%lu"
#define SYSFS_BUFFER_PATH	"buffer%lu"
#define SYSFS_CHIP_NAME         "name"
#define SYSFS_BUFFER_ENABLE     "enable"
#define SYSFS_BUFFER_LENGTH     "length"
#define SYSFS_SCAN_EL_EN        "in_accel_en"
#define SYSFS_SCAN_EL_INDEX     "in_accel_index"
#define SYSFS_SCAN_EL_TYPE      "in_accel_type"
#define SYSFS_GYRO_ORIENT       "info_anglvel_matrix"
#define SYSFS_GYRO_FIFO_ENABLE  "in_anglvel_enable"
#define SYSFS_GYRO_FSR          "in_anglvel_scale"
#define SYSFS_GYRO_RATE         "in_anglvel_rate"
#define SYSFS_ACCEL_ORIENT      "info_accel_matrix"
#define SYSFS_ACCEL_FIFO_ENABLE "in_accel_enable"
#define SYSFS_ACCEL_FSR         "in_accel_scale"
#define SYSFS_ACCEL_RATE        "in_accel_rate"
#define SYSFS_HIGH_RES_MODE     "in_high_res_mode"
#define SYSFS_BATCH_TIMEOUT     "misc_batchmode_timeout"

enum {
    SENSOR_ACCEL = 0,
    SENSOR_GYRO,
    SENSOR_NUM
};

/* iio sysfs path */
static char iio_sysfs_path[1024] = "";
static char iio_buffer_path[32] = "";

/* iio device path */
static char iio_dev_path[1024] = "";

/* file descriptor for IIO_DEVICE */
static int iio_buffer_fd = -1;

/* saved timestamp for each sensor */
static int64_t accel_prev_ts;
static int64_t gyro_prev_ts;

/* last poll time used for batch mode */
static int64_t last_poll_time_ns;

/* batched sample number */
static int batched_sample_accel_nb;
static int batched_sample_gyro_nb;

/* for data from driver */
static char iio_read_buf[2048];
static int iio_read_size;

/* data header from driver */
#define ACCEL_HDR                1
#define GYRO_HDR                 2
#define EMPTY_MARKER             17
#define END_MARKER               18

#define ACCEL_DATA_SZ            24
#define GYRO_DATA_SZ             24
#define EMPTY_MARKER_SZ          8
#define END_MARKER_SZ            8

/* commandline options */
static const struct option options[] = {
    {"help", no_argument, NULL, 'h'},
    {"device", required_argument, NULL, 'd'},
    {"buffer", required_argument, NULL, 'b'},
    {"convert", no_argument, NULL, 'c'},
    {0, 0, 0, 0},
};

static const char *options_descriptions[] = {
    "Show this help and quit.",
    "Choose device by numero.",
    "Choose buffer by numero.",
    "Show data after unit conversion (m/s^2, rad/s)",
};

/* get the current time */
static int64_t get_current_timestamp(void)
{
    struct timespec tp;

    clock_gettime(CLOCK_BOOTTIME, &tp);
    return  (int64_t)tp.tv_sec * 1000000000LL + (int64_t)tp.tv_nsec;
}

static int write_buffer_sysfs_int(char *attr, int data)
{
    FILE *fp;
    int ret;
    static char path[1024];

    ret = snprintf(path, sizeof(path), "%s/%s/%s", iio_sysfs_path, iio_buffer_path, attr);
    if (ret < 0 || ret >= (int)sizeof(path)) {
        return -1;
    }

    ret = 0;
    printf("sysfs: %d -> %s\n", data, path);
    fp = fopen(path, "w");
    if (fp == NULL) {
        ret = -errno;
        printf("Failed to open %s\n", path);
    } else {
        if (fprintf(fp, "%d\n", data) < 0) {
            printf("Failed to write to %s\n", path);
            ret = -errno;
        }
        fclose(fp);
    }
    fflush(stdout);
    return ret;
}

/* read a value from sysfs */
static int read_sysfs_int(char *attr, int *data)
{
    FILE *fp;
    int ret;
    static char path[1024];

    ret = snprintf(path, sizeof(path), "%s/%s", iio_sysfs_path, attr);
    if (ret < 0 || ret >= (int)sizeof(path)) {
        return -1;
    }

    ret = 0;
    fp = fopen(path, "r");
    if (fp == NULL) {
        ret = -errno;
        printf("Failed to open %s\n", path);
    } else {
        if (fscanf(fp, "%d", data) != 1) {
            printf("Failed to read %s\n", path);
            ret = -1;
        }
        fclose(fp);
    }
    printf("sysfs: %d <- %s\n", *data, path);
    fflush(stdout);
    return ret;
}

/* get sensor orientation from sysfs */
static int get_sensor_orient(int sensor, int *orient)
{
    static char path[1024];
    FILE *fp;
    int ret;
    char *attr;

    if (sensor == SENSOR_ACCEL) {
        attr = SYSFS_ACCEL_ORIENT;
    } else if (sensor == SENSOR_GYRO) {
        attr = SYSFS_GYRO_ORIENT;
    } else {
        printf("invalid sensor type\n");
        fflush(stdout);
        return -1;
    }

    ret = snprintf(path, sizeof(path), "%s/%s", iio_sysfs_path, attr);
    if (ret < 0 || ret >= (int)sizeof(path)) {
        return -1;
    }

    ret = 0;
    fp = fopen(path, "r");
    if (fp == NULL) {
        printf("Failed to open %s\n", path);
        ret = -errno;
    } else {
        if (fscanf(fp, "%d,%d,%d,%d,%d,%d,%d,%d,%d",
                &orient[0], &orient[1], &orient[2],
                &orient[3], &orient[4], &orient[5],
                &orient[6], &orient[7], &orient[8]) != 9) {
            printf("Failed to read %s\n", path);
            ret = -1;
        }
        fclose(fp);
    }
    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            orient[0], orient[1], orient[2],
            orient[3], orient[4], orient[5],
            orient[6], orient[7], orient[8]);

    fflush(stdout);
    return ret;
}

/* get chip name from sysfs */
static int show_chip_name(void)
{
    FILE *fp;
    int ret;
    static char name[256];
    static char path[1024];

    ret = snprintf(path, sizeof(path), "%s/%s", iio_sysfs_path, SYSFS_CHIP_NAME);
    if (ret < 0 || ret >= (int)sizeof(path)) {
        return -1;
    }

    ret = 0;
    fp = fopen(path, "r");
    if (fp == NULL) {
        ret = -errno;
        printf("Failed to open %s\n", path);
    } else {
        if (fscanf(fp, "%s", name) != 1) {
            printf("Failed to read chip name\n");
            ret = -1;
        } else
            printf("chip : %s\n", name);
        fclose(fp);
    }
    fflush(stdout);
    return ret;
}

/* setup iio */
static int setup_iio(int buffer_no)
{
    int iio_fd;
    int ret;
    static char path[1024];

    /* disable */
    ret = write_buffer_sysfs_int(SYSFS_BUFFER_ENABLE, 0);
    if (ret)
        return ret;

    /* scan_elements en */
    ret = write_buffer_sysfs_int(SYSFS_SCAN_EL_EN, 1);
    if (ret)
        return ret;

    /* buffer length */
    ret = write_buffer_sysfs_int(SYSFS_BUFFER_LENGTH, IIO_BUFFER_LENGTH);
    if (ret)
        return ret;

    /* enable */
    ret = write_buffer_sysfs_int(SYSFS_BUFFER_ENABLE, 1);
    if (ret)
        return ret;

    /* device */
    ret = snprintf(path, sizeof(path), "%s", iio_dev_path);
    if (ret < 0 || ret >= (int)sizeof(path)) {
        return -1;
    }

    iio_fd = open(iio_dev_path, O_RDONLY);
    if (iio_fd < 0) {
        printf("failed to open %s\n", iio_dev_path);
        fflush(stdout);
        return -errno;
    }

    ret = ioctl(iio_fd, IIO_BUFFER_GET_FD_IOCTL, &buffer_no);
    close(iio_fd);
    if (ret == -1) {
        printf("failed to do ioctl error %d\n", errno);
        fflush(stdout);
        return -errno;
    }
    iio_buffer_fd = buffer_no;

    return 0;
}

/* get fsr through sysfs */
static int get_sensor_fsr(int sensor, int *fsr)
{
    int ret = 0;

    if (sensor == SENSOR_ACCEL) {
        ret = read_sysfs_int(SYSFS_ACCEL_FSR, fsr);
    } else if (sensor == SENSOR_GYRO) {
        ret = read_sysfs_int(SYSFS_GYRO_FSR, fsr);
    } else {
        printf("invalid sensor type\n");
    }
    fflush(stdout);
    return ret;
}

/* show usage */
static void usage(void)
{
    unsigned int i;

    printf("Usage:\n\t test-sensors-sysfs [-d <device_no>] [-a <rate>] [-g <rate>] [-c]"
            "\n\nOptions:\n");
    for (i = 0; options[i].name; i++)
        printf("\t-%c, --%s\n\t\t\t%s\n",
                options[i].val, options[i].name,
                options_descriptions[i]);
    printf("Version:\n\t%s\n", VERSION_STR);
    printf("Note:\n\t%s\n\n", USAGE_NOTE);
    fflush(stdout);
}

/* read sensor data from char device */
static int read_and_show_data(int *accel_orient, int *gyro_orient, double accel_scale, double gyro_scale, bool convert)
{
    unsigned short header;
    char *rdata;
    int sensor, len;
    int nbytes;
    int ptr = 0;
    int gyro[3], accel[3];
    int body_lsb[3];
    double body_conv[3];
    int64_t  gyro_ts, accel_ts;
    bool accel_valid = false;
    bool gyro_valid = false;
    int left_over = 0;
    int64_t curr_ts;
    float ts_gap, ts_gap_prev;

    curr_ts = get_current_timestamp();

    /* read data from char device */
    nbytes = sizeof(iio_read_buf) - iio_read_size;
    len = read(iio_buffer_fd, &iio_read_buf[iio_read_size], nbytes);
    //printf("read len = %d\n", len);
    if (len < 0) {
        printf("failed to read iio buffer\n");
        return len;
    }
    if (len == 0) {
        printf("no data in buffer\n");
        return 0;
    }

    iio_read_size += len;

    /* parse data */
    while (ptr < iio_read_size) {
        accel_valid = false;
        gyro_valid = false;
        rdata = &iio_read_buf[ptr];
        header = *(unsigned short*)rdata;

        switch (header) {
            case END_MARKER:
                if ((iio_read_size - ptr) < END_MARKER_SZ) {
                    left_over = iio_read_size - ptr;
                    break;
                }
                sensor = *((int *) (rdata + 4));
                printf("HAL:MARKER DETECTED what:%d\n", sensor);
                ptr += END_MARKER_SZ;
                break;
            case EMPTY_MARKER:
                if ((iio_read_size - ptr) < EMPTY_MARKER_SZ) {
                    left_over = iio_read_size - ptr;
                    break;
                }
                sensor = *((int *) (rdata + 4));
                printf("HAL:EMPTY MARKER DETECTED what:%d\n", sensor);
                ptr += EMPTY_MARKER_SZ;
                break;
            case GYRO_HDR:
                if ((iio_read_size - ptr) < GYRO_DATA_SZ) {
                    left_over = iio_read_size - ptr;
                    break;
                }
                gyro[0] = *((int *) (rdata + 4));
                gyro[1] = *((int *) (rdata + 8));
                gyro[2] = *((int *) (rdata + 12));
                gyro_ts = *((int64_t*) (rdata + 16));
                gyro_valid = true;
                ptr += GYRO_DATA_SZ;
                break;
            case ACCEL_HDR:
                if ((iio_read_size - ptr) < ACCEL_DATA_SZ) {
                    left_over = iio_read_size - ptr;
                    break;
                }
                accel[0] = *((int *) (rdata + 4));
                accel[1] = *((int *) (rdata + 8));
                accel[2] = *((int *) (rdata + 12));
                accel_ts = *((int64_t*) (rdata + 16));
                accel_valid = true;
                ptr += ACCEL_DATA_SZ;
                break;
            default:
                ptr++;
                break;
        }

        /* show data */
        if (accel_valid) {
            ts_gap = (float)(curr_ts - accel_ts)/1000000.f;
            ts_gap_prev = (float)(accel_ts - accel_prev_ts)/1000000.f;
            accel_prev_ts = accel_ts;
            body_lsb[0] = accel[0] * accel_orient[0] + accel[1] * accel_orient[1] + accel[2] * accel_orient[2];
            body_lsb[1] = accel[0] * accel_orient[3] + accel[1] * accel_orient[4] + accel[2] * accel_orient[5];
            body_lsb[2] = accel[0] * accel_orient[6] + accel[1] * accel_orient[7] + accel[2] * accel_orient[8];
            if (convert) {
                body_conv[0] = (double)body_lsb[0] * accel_scale;
                body_conv[1] = (double)body_lsb[1] * accel_scale;
                body_conv[2] = (double)body_lsb[2] * accel_scale;
                printf("Accel body (m/s^2), %+13f, %+13f, %+13f, %20" PRId64 ", %8.3f, %8.3f\n",
                        body_conv[0], body_conv[1], body_conv[2],
                        accel_ts, ts_gap_prev, ts_gap);
            } else {
                printf("Accel body (LSB)  , %+6d, %+6d, %+6d, %20" PRId64 ", %8.3f, %8.3f\n",
                        body_lsb[0], body_lsb[1], body_lsb[2],
                        accel_ts, ts_gap_prev, ts_gap);
            }
            batched_sample_accel_nb++;
        }
        if (gyro_valid) {
            ts_gap = (float)(curr_ts - gyro_ts)/1000000.f;
            ts_gap_prev = (float)(gyro_ts - gyro_prev_ts)/1000000.f;
            gyro_prev_ts = gyro_ts;
            body_lsb[0] = gyro[0] * gyro_orient[0] + gyro[1] * gyro_orient[1] + gyro[2] * gyro_orient[2];
            body_lsb[1] = gyro[0] * gyro_orient[3] + gyro[1] * gyro_orient[4] + gyro[2] * gyro_orient[5];
            body_lsb[2] = gyro[0] * gyro_orient[6] + gyro[1] * gyro_orient[7] + gyro[2] * gyro_orient[8];
            if (convert) {
                body_conv[0] = (double)body_lsb[0] * gyro_scale;
                body_conv[1] = (double)body_lsb[1] * gyro_scale;
                body_conv[2] = (double)body_lsb[2] * gyro_scale;
                printf("Gyro  body (rad/s), %+13f, %+13f, %+13f, %20" PRId64 ", %8.3f, %8.3f\n",
                        body_conv[0], body_conv[1], body_conv[2],
                        gyro_ts, ts_gap_prev, ts_gap);
            } else {
                printf("Gyro  body (LSB)  , %+6d, %+6d, %+6d, %20" PRId64 ", %8.3f, %8.3f\n",
                        body_lsb[0], body_lsb[1], body_lsb[2],
                        gyro_ts, ts_gap_prev, ts_gap);
            }
            batched_sample_gyro_nb++;
        }
        if (left_over) {
            break;
        }
    }

    if (left_over > 0) {
        memmove(iio_read_buf, &iio_read_buf[ptr], left_over);
        iio_read_size = left_over;
    } else {
        iio_read_size = 0;
    }
    fflush(stdout);

    return 0;
}

/* signal handler to disable sensors when ctr-C */
static void sig_handler(int s)
{
    int ret = 0;

    (void)s;

    printf("Disable buffer\n");
    ret = write_buffer_sysfs_int(SYSFS_BUFFER_ENABLE, 0);
    if (ret) {
        printf("failed to disable buffer\n");
        fflush(stdout);
        return;
    }

    /* close */
    close(iio_buffer_fd);

    fflush(stdout);
    exit(1);
}


/* --- main --- */
int main(int argc, char *argv[])
{
    int ret;
    int gyro_orient[9], accel_orient[9];
    struct sigaction sig_action;
    int opt, option_index;
    unsigned long device_no = 0;
    unsigned long buffer_no = 0;
    bool convert = false;
    int accel_fsr_gee = 0;
    int gyro_fsr_dps = 0;
    double accel_scale = 0;
    double gyro_scale = 0;

    while ((opt = getopt_long(argc, argv, "hd:b:c", options, &option_index)) != -1) {
        switch (opt) {
            case 'd':
                device_no = strtoul(optarg, NULL, 10);
                break;
            case 'b':
                buffer_no = strtoul(optarg, NULL, 10);
                break;
            case 'c':
                convert = true;
                break;
            case 'h':
                usage();
                return 0;
        }
    }

    /* signal handling */
    sig_action.sa_handler = sig_handler;
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;
    sigaction(SIGINT, &sig_action, NULL);

    /* set iio sysfs and device paths */
    ret = snprintf(iio_sysfs_path, sizeof(iio_sysfs_path), SYSFS_PATH, device_no);
    if (ret < 0 || ret >= (int)sizeof(iio_sysfs_path)) {
        printf("error %d cannot set iio sysfs path\n", ret);
        fflush(stdout);
        return -errno;
    }
    ret = snprintf(iio_buffer_path, sizeof(iio_buffer_path), SYSFS_BUFFER_PATH, buffer_no);
    if (ret < 0 || ret >= (int)sizeof(iio_buffer_path)) {
        printf("error %d cannot set iio buffer path\n", ret);
        fflush(stdout);
        return -errno;
    }
    ret = snprintf(iio_dev_path, sizeof(iio_dev_path), IIO_DEVICE, device_no);
    if (ret < 0 || ret >= (int)sizeof(iio_dev_path)) {
        printf("error %d cannot set iio dev path\n", ret);
        fflush(stdout);
        return -errno;
    }

    printf(">Start\n");
    fflush(stdout);

    /* show chip name */
    ret = show_chip_name();
    if (ret) {
        return ret;
    }

    /* get sensor orientation */
    printf(">Get accel orientation\n");
    fflush(stdout);
    ret = get_sensor_orient(SENSOR_ACCEL, accel_orient);
    if (ret) {
        printf("failed to get accel orientation\n");
        fflush(stdout);
        return ret;
    }
    printf(">Get gyro orientation\n");
    fflush(stdout);
    ret = get_sensor_orient(SENSOR_GYRO, gyro_orient);
    if (ret) {
        printf("failed to get gyro orientation\n");
        fflush(stdout);
        return ret;
    }

    /* setup iio */
    printf(">Set up IIO with buffer #%lu\n", buffer_no);
    fflush(stdout);
    ret = setup_iio(buffer_no);
    if (ret) {
        printf("failed to set up iio\n");
        fflush(stdout);
        return ret;
    }

    /* accel setup */
    printf(">Get accel FSR\n");
    fflush(stdout);
    ret = get_sensor_fsr(SENSOR_ACCEL, &accel_fsr_gee);
    if (ret)
        return ret;
    accel_prev_ts = get_current_timestamp();
    batched_sample_accel_nb = 0;
#ifdef FIFO_HIGH_RES_ENABLE
    accel_scale = (double)accel_fsr_gee / 524288.f * 9.80665f; // LSB(20bit) to m/s^2
#else
    accel_scale = (double)accel_fsr_gee / 32768.f * 9.80665f; // LSB(16bit) to m/s^2
#endif

    /* gyro setup */
    printf(">Get gyro FSR\n");
    fflush(stdout);
    ret = get_sensor_fsr(SENSOR_GYRO, &gyro_fsr_dps);
    if (ret)
        return ret;
    gyro_prev_ts = get_current_timestamp();
    batched_sample_gyro_nb = 0;
#ifdef FIFO_HIGH_RES_ENABLE
    gyro_scale = (double)gyro_fsr_dps / 524288.f * M_PI / 180; // LSB(20bit) to rad/s
#else
    gyro_scale = (double)gyro_fsr_dps / 32768.f * M_PI / 180; // LSB(16bit) to rad/s
#endif

    last_poll_time_ns = get_current_timestamp();

    /* collect sensor data */
    while (1) {
        struct pollfd fds[1];
        int nb;
        fds[0].fd = iio_buffer_fd;
        fds[0].events = POLLIN;
        fds[0].revents = 0;
        nb = poll(fds, 1, -1);
        if (nb > 0) {
            if (fds[0].revents & (POLLIN | POLLPRI)) {
                fds[0].revents = 0;
                /* read sensor from FIFO and show */
                read_and_show_data(accel_orient, gyro_orient, accel_scale, gyro_scale, convert);
            }
        }
    }
    return 0;
}

