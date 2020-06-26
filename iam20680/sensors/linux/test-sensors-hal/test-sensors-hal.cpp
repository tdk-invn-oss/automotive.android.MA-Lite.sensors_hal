/*
 * Copyright (C) 2018-2019 InvenSense, Inc.
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
#include <unistd.h>
#include <errno.h>
#include <dlfcn.h>
#include <signal.h>
#include <pthread.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <vector>

#include "hardware/hardware.h"
#include "hardware/sensors.h"

#define ARRAY_SIZE(array)		(sizeof(array) / sizeof(array[0]))
#define NS_IN_SEC			1000000000LL

struct enabled_sensor {
	unsigned index;
	int handle;
	int rate_hz;
	int64_t timestamp;
	int batched_sample_nb;
};

static std::vector<struct enabled_sensor> enabled_sensor_list;
static const struct sensor_t *sensors_list;
static unsigned sensors_nb;
static int batch_ms;
static int64_t last_poll_time_ns;

static int64_t get_current_timestamp(void)
{
	struct timespec tp;

	clock_gettime(CLOCK_MONOTONIC, &tp);
	return  (int64_t)tp.tv_sec * 1000000000LL + (int64_t)tp.tv_nsec;
}

static void print_batch_info(void)
{
	int64_t timestamp = get_current_timestamp();
	int64_t odr_ns;
	int odr_hz = 4;

	for (unsigned i = 0; i < enabled_sensor_list.size(); i++) {
		if (enabled_sensor_list[i].rate_hz > odr_hz)
			odr_hz = enabled_sensor_list[i].rate_hz;
	}
	odr_ns = NS_IN_SEC / odr_hz;

	if (batch_ms != 0) {
		if (timestamp > (last_poll_time_ns + odr_ns)) {
			for (unsigned i = 0; i < enabled_sensor_list.size(); i++) {
				if (enabled_sensor_list[i].batched_sample_nb) {
					printf("INFO Previous batch count for sensor '%s' is %d\n",
							sensors_list[enabled_sensor_list[i].index].name,
							enabled_sensor_list[i].batched_sample_nb);
				}
				enabled_sensor_list[i].batched_sample_nb = 0;
			}
			printf("INFO New batch duration %" PRId64 " ms\n",
					(timestamp - last_poll_time_ns) / 1000000);
		}
	}
	last_poll_time_ns = timestamp;
}

static void print_data(const sensors_event_t* events, int count)
{
	int64_t timestamp;
	const sensors_event_t *data;

	timestamp = get_current_timestamp();

	//printf("read count = %d\n", count);
	for (int i = 0; i < count; i++) {
		data = &events[i];
		unsigned t;
		for (t = 0; t < enabled_sensor_list.size(); t++) {
			if (enabled_sensor_list[t].handle == data->sensor && data->version == sizeof(sensors_event_t))
				break;
		}
		if (t >= enabled_sensor_list.size())
			continue;

		printf("%-30s, ", sensors_list[enabled_sensor_list[t].index].name);
		printf("%+13f, %+13f, %+13f, ", data->data[0], data->data[1], data->data[2]);
		printf("%20" PRId64 ", ", data->timestamp);
		printf("%8.3f, ", float(data->timestamp - enabled_sensor_list[t].timestamp) / 1000000.f);
		printf("%8.3f, ", float(timestamp - data->timestamp) / 1000000.f);
		if (data->type == SENSOR_TYPE_GYROSCOPE_UNCALIBRATED) {
			printf("%+13f, %+13f, %+13f", data->data[3], data->data[4], data->data[5]);
		}
		printf("\n");
		enabled_sensor_list[t].timestamp = data->timestamp;
		enabled_sensor_list[t].batched_sample_nb++;
	}
}

static void *data_thread(void *arg)
{
	static sensors_event_t data[1024];
	sensors_poll_device_1_t *device = (sensors_poll_device_1_t *)arg;
	int ret;
	long status;

	do {
		ret = device->poll(&device->v0, data, ARRAY_SIZE(data));
		if (ret < 0) {
			printf("data poll error %d!\n", ret);
		} else {
			print_batch_info();
			print_data(data, ret);
		}
	} while (ret >= 0);

	status = ret;
	pthread_exit((void *)status);
}

static void show_usage(char *argv0)
{
	printf(
		"\n"
		"Usage : %s [OPTIONs] <sensor1,rate1> [<sensor2,rate2> ...]\n"
		"\n"
		"Options\n"
		"\t-l :Show sensors list\n"
		"\t-m :Show sensors metadata\n"
		"\t-p :Show sensors data\n"
		"\t-b timeout=<batch_ms> :Batch timeout in ms\n"
		"\n"
		"\t<sensorN,rateN>\n"
		"\t  sensorN : Index of sensor\n"
		"\t  rateN   : ODR (Hz) for sensorN\n"
		, argv0);
}

static void show_sensor_list(const struct sensor_t *sensors_list, unsigned sensors_nb)
{
	printf("Sensors' list:\n");
	for (unsigned i = 0; i < sensors_nb; i++) {
		printf("\t%-2d -> type %2d, name '%s'\n", i, sensors_list[i].type, sensors_list[i].name);
	}
}

static void show_sensor_metadata(const struct sensor_t *sensors_list, unsigned sensors_nb)
{
	for (unsigned i = 0; i < sensors_nb; i++) {
		printf("Sensor '%s' metadata:\n", sensors_list[i].name);
		printf("    vendor       : %s\n", sensors_list[i].vendor);
		printf("    version      : %d\n", sensors_list[i].version);
		printf("    handle       : %d\n", sensors_list[i].handle);
		printf("    type         : %d\n", sensors_list[i].type);
		printf("    maxRange     : %f\n", sensors_list[i].maxRange);
		printf("    resolution   : %f\n", sensors_list[i].resolution);
		printf("    power        : %f\n", sensors_list[i].power);
		printf("    minDelay     : %d\n", sensors_list[i].minDelay);
#ifdef __LP64__
		printf("    maxDelay     : %" PRId64 "\n", sensors_list[i].maxDelay);
#else
		printf("    maxDelay     : %d\n", sensors_list[i].maxDelay);
#endif
		printf("    fifoMaxEventCount : %d\n", sensors_list[i].fifoMaxEventCount);
	}
}


int main(int argc, char *argv[])
{
	sigset_t set, old;
	int sig;
	void *handle;
	struct sensors_module_t *module;
	sensors_poll_device_1_t *device;
	pthread_t tid;
	int ret, status;
	int opt;
	bool show_list = false;
	bool show_metadata = false;
	bool show_sensor_data = false;

	handle = dlopen("libinvnsensors.so", RTLD_LAZY);
	if (handle == NULL) {
		printf("cannot load Sensors HAL [%s]\n", dlerror());
		exit(EXIT_FAILURE);
	}

	module = (struct sensors_module_t *)dlsym(handle, HAL_MODULE_INFO_SYM_AS_STR);
	if (module == NULL) {
		printf("cannot load sensors module [%s]\n", dlerror());
		status = EXIT_FAILURE;
		goto exit_dlclose;
	}

	printf("Opening device...");
	fflush(stdout);
	ret = sensors_open_1(&module->common, &device);
	if (ret != 0) {
		printf(" error! [%s]\n", strerror(-ret));
		status = EXIT_FAILURE;
		goto exit_dlclose;
	}
	printf(" OK!\n");
	fflush(stdout);

	ret = module->get_sensors_list(module, &sensors_list);
	if (ret < 0) {
		printf("get sensors list failed [%s]\n", strerror(-ret));
		status = EXIT_FAILURE;
		goto exit_close;
	}
	sensors_nb = ret;
	if (sensors_nb == 0) {
		printf("no sensors available\n");
		status = EXIT_FAILURE;
		goto exit_close;
	}

	batch_ms = 0;
	printf("Checking commandline options...");
	while ((opt = getopt(argc, argv, "mlpb:")) != -1) {
		switch (opt) {
			case 'm':
				show_metadata = true;
				break;
			case 'l':
				show_list = true;
				break;
			case 'p':
				show_sensor_data = true;
				break;
			case 'b':
				if (sscanf(optarg, "timeout=%d", &batch_ms) != 1)
					show_usage(argv[0]);
				break;
			default:
				show_usage(argv[0]);
				break;
		}
	}

	if (optind < argc) {
		while (optind < argc) {
			int sen, hz;
			struct enabled_sensor s;
			sscanf(argv[optind++], "%d,%d", &sen, &hz);
			s.index = sen;
			s.rate_hz = hz;
			s.handle = sensors_list[s.index].handle;
			if (s.index < sensors_nb)
				enabled_sensor_list.push_back(s);
		}
	}

	if (!show_list && !show_metadata && !show_sensor_data) {
		show_usage(argv[0]);
		status = EXIT_SUCCESS;
		goto exit_close;
	}
	printf(" OK!\n");
	fflush(stdout);

	if (show_list) {
		show_sensor_list(sensors_list, sensors_nb);
		status = EXIT_SUCCESS;
		goto exit_close;
	}

	if (show_metadata) {
		show_sensor_metadata(sensors_list, sensors_nb);
		status = EXIT_SUCCESS;
		goto exit_close;
	}

	if (show_sensor_data && enabled_sensor_list.empty()) {
		show_usage(argv[0]);
		status = EXIT_FAILURE;
		goto exit_close;
	}

	printf("Start data thread...");
	fflush(stdout);
	sigfillset(&set);
	pthread_sigmask(SIG_BLOCK, &set, &old);
	ret = pthread_create(&tid, NULL, data_thread, device);
	if (ret != 0) {
		printf("thread create error [%s]\n", strerror(-ret));
		status = EXIT_FAILURE;
		goto exit_close;
	}
	pthread_sigmask(SIG_SETMASK, &old, NULL);
	printf(" OK!\n");
	fflush(stdout);

	printf("Disabling all sensors...");
	fflush(stdout);
	for (unsigned i = 0; i < sensors_nb; ++i) {
		ret = device->activate(&device->v0, sensors_list[i].handle, 0);
		if (ret != 0) {
			printf(" activate error (%d)!\n", ret);
			status = EXIT_FAILURE;
			goto exit_thread_close;
		}
	}
	printf(" OK!\n");
	fflush(stdout);

	printf("Enabling sensors...");
	for (std::vector<enabled_sensor>::iterator it = enabled_sensor_list.begin(); it != enabled_sensor_list.end(); ++it) {
		int64_t ns = 0;
		if (it->rate_hz)
			ns = NS_IN_SEC / it->rate_hz;
		ret = device->batch(device, it->handle, 0, ns, (int64_t)batch_ms * 1000000);
		if (ret != 0) {
			printf(" batch error (%d)!\n", ret);
			status = EXIT_FAILURE;
			goto exit_thread_close;
		}
		ret = device->activate(&device->v0, it->handle, 1);
		if (ret != 0) {
			printf(" activate error (%d)!\n", ret);
			status = EXIT_FAILURE;
			goto exit_thread_close;
		}
		it->timestamp = get_current_timestamp();
		it->batched_sample_nb = 0;
	}
	last_poll_time_ns = get_current_timestamp();
	printf(" OK!\n");
	fflush(stdout);

	sigemptyset(&set);
	sigaddset(&set, SIGTERM);
	sigaddset(&set, SIGINT);
	pthread_sigmask(SIG_BLOCK, &set, NULL);
	ret = sigwait(&set, &sig);
	if (ret != 0) {
		printf("error [%s](%d) waiting for signals\n", strerror(ret), ret);
	}
	pthread_sigmask(SIG_UNBLOCK, &set, NULL);
	status = EXIT_SUCCESS;

	printf("Disabling sensors...");
	fflush(stdout);
	for (std::vector<enabled_sensor>::iterator it = enabled_sensor_list.begin(); it != enabled_sensor_list.end(); ++it) {
		ret = device->activate(&device->v0, it->handle, 0);
		if (ret != 0) {
			printf(" activate error (%d)!\n", ret);
			status = EXIT_FAILURE;
			goto exit_thread_close;
		}
	}
	printf(" OK!\n");
	fflush(stdout);

exit_thread_close:
	printf("Terminate data thread...");
	fflush(stdout);
	pthread_cancel(tid);
	pthread_join(tid, NULL);
	printf(" OK!\n");
	fflush(stdout);
exit_close:
	printf("Closing device...");
	fflush(stdout);
	ret = sensors_close(&device->v0);
	if (ret != 0) {
		printf(" error! [%s]\n", strerror(-ret));
		status = EXIT_FAILURE;
	} else {
		printf(" OK!\n");
	}
	fflush(stdout);
exit_dlclose:
	dlclose(handle);
	exit(status);
}
