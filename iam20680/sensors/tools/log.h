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

#ifndef _MPL_LOG_H
#define _MPL_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __ANDROID__

#ifdef MPL_LOG_NDEBUG
#undef LOG_NDEBUG
#define LOG_NDEBUG	MPL_LOG_NDEBUG
#endif
#ifdef MPL_LOG_TAG
#undef LOG_TAG
#define LOG_TAG		MPL_LOG_TAG
#endif

/* Android logs */
#include <log/log.h>

#define MPL_ASSERT	ALOG_ASSERT

#define MPL_LOGV	ALOGV
#define MPL_LOGD	ALOGD
#define MPL_LOGI	ALOGI
#define MPL_LOGW	ALOGW
#define MPL_LOGE	ALOGE

#define MPL_LOGV_IF	ALOGV_IF
#define MPL_LOGD_IF	ALOGD_IF
#define MPL_LOGI_IF	ALOGI_IF
#define MPL_LOGW_IF	ALOGW_IF
#define MPL_LOGE_IF	ALOGE_IF

#else

/* Linux logs */
#if MPL_LOG_NDEBUG
#define NDEBUG
#endif
#include <assert.h>
#include <syslog.h>

#ifndef MPL_LOG_TAG
#define MPL_LOG_HEAD	"xxx: "
#else
#define MPL_LOG_HEAD	MPL_LOG_TAG ": "
#endif

#if MPL_LOG_NDEBUG
#define MPL_ASSERT(cond, ...)	((void)0)
#else
#define MPL_ASSERT(cond, ...)					\
	do {							\
		syslog(LOG_CRIT, MPL_LOG_HEAD __VA_ARGS__);	\
		assert(cond);					\
	} while (0)
#endif

#if MPL_LOG_NDEBUG
#define MPL_LOGV(...)						\
	if (0) {						\
		syslog(LOG_DEBUG, MPL_LOG_HEAD __VA_ARGS__);	\
	}
#define MPL_LOGD(...)						\
	if (0) {						\
		syslog(LOG_DEBUG, MPL_LOG_HEAD __VA_ARGS__);	\
	}
#else
#define MPL_LOGV(...)	syslog(LOG_DEBUG, MPL_LOG_HEAD __VA_ARGS__)
#define MPL_LOGD(...)	syslog(LOG_DEBUG, MPL_LOG_HEAD __VA_ARGS__)
#endif
#define MPL_LOGI(...)	syslog(LOG_INFO, MPL_LOG_HEAD __VA_ARGS__)
#define MPL_LOGW(...)	syslog(LOG_WARNING, MPL_LOG_HEAD __VA_ARGS__)
#define MPL_LOGE(...)	syslog(LOG_ERR, MPL_LOG_HEAD __VA_ARGS__)

#ifndef __predict_false
#define __predict_false(exp) __builtin_expect((exp) != 0, 0)
#endif

#define MPL_LOGV_IF(cond, ...)						\
	((__predict_false(cond)) ? ((void)MPL_LOGV(__VA_ARGS__))	\
				 : (void)0)
#define MPL_LOGD_IF(cond, ...)						\
	((__predict_false(cond)) ? ((void)MPL_LOGD(__VA_ARGS__))	\
				 : (void)0)
#define MPL_LOGI_IF(cond, ...)						\
	((__predict_false(cond)) ? ((void)MPL_LOGI(__VA_ARGS__))	\
				 : (void)0)
#define MPL_LOGW_IF(cond, ...)						\
	((__predict_false(cond)) ? ((void)MPL_LOGW(__VA_ARGS__))	\
				 : (void)0)
#define MPL_LOGE_IF(cond, ...)						\
	((__predict_false(cond)) ? ((void)MPL_LOGE(__VA_ARGS__))	\
				 : (void)0)

#endif		/* __ANDROID__ */

static inline void __print_result_location(int result,
					   const char *file,
					   const char *func, int line)
{
	MPL_LOGE("%s|%s|%d returning %d\n", file, func, line, result);
}

#define LOG_RESULT_LOCATION(condition)					\
	do {								\
		__print_result_location((int)(condition), __FILE__,	\
					__func__, __LINE__);		\
	} while (0)

#define INV_ERROR_CHECK(r_1329)			\
	if (r_1329) {				\
		LOG_RESULT_LOCATION(r_1329);	\
		return r_1329;			\
	}

#ifdef __cplusplus
}
#endif

#endif		/* _MPL_LOG_H */
