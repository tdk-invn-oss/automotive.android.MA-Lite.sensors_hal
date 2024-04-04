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

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <set>
#include <thread>
#include <poll.h>

#include <hardware/sensors.h>
#include <V2_1/SubHal.h>

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace implementation {

using ::android::hardware::sensors::V1_0::OperationMode;
using ::android::hardware::sensors::V1_0::Result;
using ::android::hardware::sensors::V1_0::SharedMemInfo;
using ::android::hardware::sensors::V1_0::RateLevel;
using ::android::hardware::sensors::V2_1::Event;

class SensorsSubHalV2_1 : public ISensorsSubHal {
  public:
    SensorsSubHalV2_1();
    virtual ~SensorsSubHalV2_1();
    int open();
    int close();
    virtual Return<Result> initialize(const sp<IHalProxyCallback>& halProxyCallback) override;
    virtual Return<void> debug(const hidl_handle& fd, const hidl_vec<hidl_string>& args) override;
    virtual const std::string getName() override {
        return "TDK-InvenSense Sensors HAL";
    }
    virtual Return<void> getSensorsList_2_1(V2_1::ISensors::getSensorsList_2_1_cb _hidl_cb) override;
    virtual Return<Result> injectSensorData_2_1(const Event& event) override;
    virtual Return<Result> setOperationMode(OperationMode mode) override;
    virtual Return<Result> activate(int32_t sensorHandle, bool enabled) override;
    virtual Return<Result> batch(int32_t sensorHandle, int64_t samplingPeriodNs, int64_t maxReportLatencyNs) override;
    virtual Return<Result> flush(int32_t sensorHandle) override;
    virtual Return<void> registerDirectChannel(const SharedMemInfo& mem,
                                       V2_0::ISensors::registerDirectChannel_cb _hidl_cb) override;
    virtual Return<Result> unregisterDirectChannel(int32_t channelHandle) override;
    virtual Return<void> configDirectReport(int32_t sensorHandle, int32_t channelHandle, RateLevel rate,
                                    V2_0::ISensors::configDirectReport_cb _hidl_cb) override;

  private:
    static void pollThread(SensorsSubHalV2_1 *hal);
    sensors_module_t *mSensorModule;
    sensors_poll_device_1 *mSensorDevice;
    bool mInit;
    sp<IHalProxyCallback> mCallback;
    OperationMode mCurrentOperationMode;
    std::vector<SensorInfo> mSensorsList;
    std::set<int32_t> mWakeUpSensorsList;
    std::thread mPollThread;
};

}  // namespace implementation
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android
