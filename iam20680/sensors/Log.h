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

#ifndef _LOG_H
#define _LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Android logs */
#ifdef __ANDROID__

#include <log/log.h>

#define ASSERT		ALOG_ASSERT

#define LOGV		ALOGV
#define LOGD		ALOGD
#define LOGI		ALOGI
#define LOGW		ALOGW
#define LOGE		ALOGE

#define LOGV_IF		ALOGV_IF
#define LOGD_IF		ALOGD_IF
#define LOGI_IF		ALOGI_IF
#define LOGW_IF		ALOGW_IF
#define LOGE_IF		ALOGE_IF

/* Linux logs */
#else

#ifndef LOG_NDEBUG
#define LOG_NDEBUG 0
#endif
#ifndef LOG_TAG
#define LOG_TAG "xxx"
#endif

#if LOG_NDEBUG
#define NDEBUG
#endif
#include <assert.h>
#include <syslog.h>

#define LOG_HEAD	LOG_TAG ": "

#if LOG_NDEBUG
#define ASSERT(cond, ...)				\
	if (0) {					\
		syslog(LOG_CRIT, LOG_HEAD __VA_ARGS__);	\
		assert(cond);				\
	}
#define LOGV(...)						\
	if (0) {						\
		syslog(LOG_DEBUG, LOG_HEAD __VA_ARGS__);	\
	}
#define LOGD(...)						\
	if (0) {						\
		syslog(LOG_DEBUG, LOG_HEAD __VA_ARGS__);	\
	}
#else
#define ASSERT(cond, ...)				\
	do {					\
		syslog(LOG_CRIT, LOG_HEAD __VA_ARGS__);	\
		assert(cond);				\
	} while (0)
#define LOGV(...)	syslog(LOG_DEBUG, LOG_HEAD __VA_ARGS__)
#define LOGD(...)	syslog(LOG_DEBUG, LOG_HEAD __VA_ARGS__)
#endif
#define LOGI(...)	syslog(LOG_INFO, LOG_HEAD __VA_ARGS__)
#define LOGW(...)	syslog(LOG_WARNING, LOG_HEAD __VA_ARGS__)
#define LOGE(...)	syslog(LOG_ERR, LOG_HEAD __VA_ARGS__)

#ifndef __predict_false
#define __predict_false(exp) __builtin_expect((exp) != 0, 0)
#endif

#define LOGV_IF(cond, ...)					\
	((__predict_false(cond)) ? ((void)LOGV(__VA_ARGS__))	\
				 : (void)0)
#define LOGD_IF(cond, ...)					\
	((__predict_false(cond)) ? ((void)LOGD(__VA_ARGS__))	\
				 : (void)0)
#define LOGI_IF(cond, ...)					\
	((__predict_false(cond)) ? ((void)LOGI(__VA_ARGS__))	\
				 : (void)0)
#define LOGW_IF(cond, ...)					\
	((__predict_false(cond)) ? ((void)LOGW(__VA_ARGS__))	\
				 : (void)0)
#define LOGE_IF(cond, ...)					\
	((__predict_false(cond)) ? ((void)LOGE(__VA_ARGS__))	\
				 : (void)0)

#endif		/* __ANDROID__ */

#ifdef __cplusplus
}
#endif

#endif		/* _LOG_H */
