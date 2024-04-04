/*
 * Copyright (C) 2022 InvenSense, Inc.
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

#include <cstdint>
#include <cinttypes>

#include <android-base/logging.h>
#include <android/hardware/sensors/2.1/types.h>
#include <convertV2_1.h>
#include <log/log.h>

#include "SensorsSubHalV2_1.h"

::android::hardware::sensors::V2_1::implementation::ISensorsSubHal* sensorsHalGetSubHal_2_1(
        uint32_t* version) {
    static ::android::hardware::sensors::V2_1::implementation::SensorsSubHalV2_1 *subHal = nullptr;

    ALOGV("sensorsHalGetSubHal_2_1()");

    if (subHal == nullptr) {
        subHal = new ::android::hardware::sensors::V2_1::implementation::SensorsSubHalV2_1();
    }

    *version = SUB_HAL_2_1_VERSION;
    return subHal;
}

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace implementation {

using ::android::hardware::Void;
using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::RateLevel;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SensorStatus;
using ::android::hardware::sensors::V1_0::SharedMemInfo;
using ::android::hardware::sensors::V1_0::SharedMemType;
using ::android::hardware::sensors::V1_0::SharedMemFormat;
using ::android::hardware::sensors::V2_0::SensorTimeout;
using ::android::hardware::sensors::V2_0::WakeLockQueueFlagBits;
using ::android::hardware::sensors::V2_0::implementation::ScopedWakelock;
using ::android::hardware::sensors::V2_1::SensorType;
using ::android::hardware::sensors::V2_1::SensorInfo;
using ::android::hardware::sensors::V2_1::Event;

void SensorsSubHalV2_1::pollThread(SensorsSubHalV2_1 *hal)
{
    ALOGV("%s()", __func__);
#define SENSORS_EVENTS_NB       1024
    static sensors_event_t data[SENSORS_EVENTS_NB];
    static std::vector<Event> events(SENSORS_EVENTS_NB);
    int ret;

    while (true) {
        ret = hal->mSensorDevice->poll(reinterpret_cast<struct sensors_poll_device_t *>(hal->mSensorDevice), data, SENSORS_EVENTS_NB);
        if (ret <= 0) {
            ALOGE_IF(ret < 0, "poll error %d", ret);
            return;
        }

        size_t nbEvents = ret;
        bool isWakeUp = false;
        for (size_t i = 0; i < nbEvents; ++i) {
            Event event;
            convertFromSensorEvent(data[i], &event);
            events.push_back(event);
            if (!isWakeUp) {
                auto wakeUp = hal->mWakeUpSensorsList.find(event.sensorHandle);
                if (wakeUp != hal->mWakeUpSensorsList.end()) {
                    isWakeUp = true;
                }
            }
        }
        ALOGV_IF(isWakeUp, "WakeUp sensor event");
        hal->mCallback->postEvents(events, hal->mCallback->createScopedWakelock(isWakeUp));
        events.clear();
    }
}

SensorsSubHalV2_1::SensorsSubHalV2_1() :
                mSensorModule(nullptr),
                mSensorDevice(nullptr),
                mInit(false),
                mCurrentOperationMode(OperationMode::NORMAL)
{
    ALOGV("%s()", __func__);
    int ret;

    ALOGI("InvenSense Sensors Multi-HAL 2.1 wrapper version %d.%d.%d%s",
          INV_VERSION_MAJOR, INV_VERSION_MINOR, INV_VERSION_PATCH, INV_VERSION_SUFFIX);

    ret = hw_get_module(SENSORS_HARDWARE_MODULE_ID, (hw_module_t const **)&mSensorModule);
    if (ret < 0) {
        ALOGE("error %d loading Sensors HAL 1.0", ret);
        return;
    }
    if (mSensorModule == nullptr) {
        ALOGE("cannot get module from Sensors HAL 1.0");
        return;
    }

    ret = this->open();
    if (ret < 0) {
        mSensorModule = nullptr;
        return;
    }
}

SensorsSubHalV2_1::~SensorsSubHalV2_1()
{
    ALOGV("%s()", __func__);

    this->close();
    mSensorModule = nullptr;
}

int SensorsSubHalV2_1::open()
{
    ALOGV("%s()", __func__);
    int ret;

    if (mSensorModule == nullptr || mSensorDevice != nullptr) {
        return -1;
    }

    ret = sensors_open_1(&mSensorModule->common, &mSensorDevice);
    if (ret < 0) {
        ALOGE("error %d opening Sensors HAL 1.0", ret);
        mSensorDevice = nullptr;
    }

    return ret;
}

int SensorsSubHalV2_1::close()
{
    ALOGV("%s()", __func__);
    int ret;

    if (mSensorDevice == nullptr) {
        return -1;
    }

    ret = sensors_close_1(mSensorDevice);
    ALOGE_IF(ret < 0, "error %d closing Sensors HAL 1.0", ret);
    mSensorDevice = nullptr;

    return ret;
}

Return<Result> SensorsSubHalV2_1::initialize(const sp<V2_1::implementation::IHalProxyCallback>& halProxyCallback)
{
    ALOGV("%s()", __func__);
    int ret;

    if (mSensorDevice == nullptr) {
        return Result::INVALID_OPERATION;
    }

    // Already init, restarting
    if (mInit) {
        ALOGI("restart Sensors HAL 1.0");
        this->close();
        mPollThread.join();
        ret = this->open();
        if (ret < 0) {
            return static_cast<Result>(ret);
        }
    }

    mCallback = halProxyCallback;
    mCurrentOperationMode = OperationMode::NORMAL;
    mPollThread = std::thread(pollThread, this);
    mInit = true;

    return Result::OK;
}

Return<void> SensorsSubHalV2_1::debug(const hidl_handle& fd, const hidl_vec<hidl_string>& args)
{
    ALOGV("%s()", __func__);

    if (fd.getNativeHandle() == nullptr || fd->numFds < 1) {
        ALOGE("%s: missing fd for writing", __FUNCTION__);
        return Void();
    }

    FILE* out = fdopen(dup(fd->data[0]), "w");
    if (args.size() != 0) {
        fprintf(out, "Note: sub-HAL %s currently does not support args. Input arguments are "
                     "ignored.\n", getName().c_str());
    }

    std::ostringstream stream;
    stream << "Available sensors:" << std::endl;
    for (auto info : mSensorsList) {
        stream << info.name << " (" << info.vendor << "): " << info.typeAsString << std::endl;
    }
    stream << std::endl;

    fprintf(out, "%s", stream.str().c_str());

    fclose(out);
    return Void();
}

Return<void> SensorsSubHalV2_1::getSensorsList_2_1(V2_1::ISensors::getSensorsList_2_1_cb _hidl_cb)
{
    ALOGV("%s()", __func__);
    const struct sensor_t *sensorsList;
    int sensorsNb = 0;

    if (mSensorModule == nullptr) {
        _hidl_cb(mSensorsList);
        return Void();
    }

    // populate the sensor list
    sensorsNb = mSensorModule->get_sensors_list(mSensorModule, &sensorsList);
    ALOGI("Sensors nb %d", sensorsNb);
    mSensorsList.clear();
    mSensorsList.reserve(sensorsNb);
    mWakeUpSensorsList.clear();
    for (int i = 0; i < sensorsNb; ++i) {
        V1_0::SensorInfo sensor;
        V1_0::implementation::convertFromSensor(sensorsList[i], &sensor);
        mSensorsList.push_back(convertToNewSensorInfo(sensor));
        if (sensor.flags & V1_0::SensorFlagBits::WAKE_UP) {
            mWakeUpSensorsList.insert(sensor.sensorHandle);
            ALOGV("WakeUp sensor %d", sensor.sensorHandle);
        }
    }

    _hidl_cb(mSensorsList);
    return Void();
}

Return<Result> SensorsSubHalV2_1::injectSensorData_2_1(const V2_1::Event& event)
{
    ALOGV("%s()", __func__);
    struct sensors_event_t data;
    int ret;

    if (mSensorDevice == nullptr) {
        return Result::INVALID_OPERATION;
    }

    if (event.sensorType == SensorType::ADDITIONAL_INFO) {
        // Used to push environment data into the device
        return Result::OK;
    }
    if (mCurrentOperationMode != OperationMode::DATA_INJECTION) {
        return Result::BAD_VALUE;
    }

    convertToSensorEvent(event, &data);
    ret = mSensorDevice->inject_sensor_data(mSensorDevice, &data);
    ALOGE_IF(ret < 0, "inject data error %d", ret);

    return static_cast<Result>(ret);
}

Return<Result> SensorsSubHalV2_1::setOperationMode(OperationMode mode)
{
    ALOGV("%s()", __func__);
    int ret;

    if (mSensorModule == nullptr) {
        return Result::INVALID_OPERATION;
    }

    ret = mSensorModule->set_operation_mode(static_cast<unsigned int>(mode));
    if (ret == 0) {
        mCurrentOperationMode = mode;
        return Result::OK;
    }

    return Result::BAD_VALUE;
}

Return<Result> SensorsSubHalV2_1::activate(int32_t sensorHandle, bool enabled)
{
    ALOGV("%s(%d, %d)", __func__, sensorHandle, enabled);
    int ret;

    if (mSensorDevice == nullptr) {
        return Result::INVALID_OPERATION;
    }

    ret = mSensorDevice->activate(reinterpret_cast<struct sensors_poll_device_t *>(mSensorDevice), sensorHandle, enabled);
    ALOGE_IF(ret < 0, "activate error %d", ret);

    return static_cast<Result>(ret);
}

Return<Result> SensorsSubHalV2_1::batch(int32_t sensorHandle, int64_t samplingPeriodNs, int64_t maxReportLatencyNs)
{
    ALOGV("%s(%d, %" PRId64 ", %" PRId64 ")", __func__, sensorHandle, samplingPeriodNs, maxReportLatencyNs);
    int ret;

    if (mSensorDevice == nullptr) {
        return Result::INVALID_OPERATION;
    }

    ret = mSensorDevice->batch(mSensorDevice, sensorHandle, 0, samplingPeriodNs, maxReportLatencyNs);
    ALOGE_IF(ret < 0, "batch error %d", ret);

    return static_cast<Result>(ret);
}

Return<Result> SensorsSubHalV2_1::flush(int32_t sensorHandle)
{
    ALOGV("%s(%d)", __func__, sensorHandle);
    int ret;

    if (mSensorDevice == nullptr) {
        return Result::INVALID_OPERATION;
    }

    ret = mSensorDevice->flush(mSensorDevice, sensorHandle);
    ALOGE_IF(ret < 0, "flush error %d", ret);

    return static_cast<Result>(ret);
}

Return<void> SensorsSubHalV2_1::registerDirectChannel(const SharedMemInfo& mem,
                                   V2_0::ISensors::registerDirectChannel_cb _hidl_cb)
{
    ALOGV("%s()", __func__);
    struct sensors_direct_mem_t mem2;
    bool convert;
    int ret;

    if (mSensorDevice == nullptr || mSensorDevice->register_direct_channel == nullptr) {
        _hidl_cb(Result::INVALID_OPERATION, -1);
        return Void();
    }

    convert = V1_0::implementation::convertFromSharedMemInfo(mem, &mem2);
    if (!convert) {
        ALOGE("SharedMem conversion error");
        _hidl_cb(Result::BAD_VALUE, -1);
        return Void();
    }

    ret = mSensorDevice->register_direct_channel(mSensorDevice, &mem2, 0);
    if (ret < 0) {
        ALOGE("Direct channel register error %d", ret);
        _hidl_cb(static_cast<Result>(ret), -1);
    } else {
         ALOGV("Register direct channel channel handle %d", ret);
        _hidl_cb(Result::OK, ret);
    }

    return Void();
}

Return<Result> SensorsSubHalV2_1::unregisterDirectChannel(int32_t channelHandle)
{
    ALOGV("%s(%d)", __func__, channelHandle);
    int ret;

    if (mSensorDevice == nullptr || mSensorDevice->register_direct_channel == nullptr) {
        return Result::INVALID_OPERATION;
    }

    ret = mSensorDevice->register_direct_channel(mSensorDevice, NULL, channelHandle);
    ALOGE_IF(ret < 0, "unregister direct channel error %d", ret);

    return static_cast<Result>(ret);
}

Return<void> SensorsSubHalV2_1::configDirectReport(int32_t sensorHandle, int32_t channelHandle, RateLevel rate,
                                V2_0::ISensors::configDirectReport_cb _hidl_cb)
{
    ALOGV("%s(%d, %d, %d)", __func__, sensorHandle, channelHandle, rate);
    struct sensors_direct_cfg_t config = {
        .rate_level = V1_0::implementation::convertFromRateLevel(rate),
    };
    int ret;

    if (mSensorDevice == nullptr || mSensorDevice->config_direct_report == nullptr) {
        _hidl_cb(Result::INVALID_OPERATION, -1);
        return Void();
    }

    ret = mSensorDevice->config_direct_report(mSensorDevice, sensorHandle, channelHandle, &config);
    if (ret < 0) {
        ALOGE("config direct report error %d", ret);
        _hidl_cb(static_cast<Result>(ret), -1);
    } else {
        ALOGV("config direct report OK token %d", ret);
        _hidl_cb(Result::OK, ret);
    }

    return Void();
}

}  // namespace implementation
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
