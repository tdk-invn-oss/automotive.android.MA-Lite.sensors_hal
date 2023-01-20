/*
 * Copyright (C) 2014-2021 InvenSense, Inc.
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

#ifndef INV_SENSOR_PARAMS_H
#define INV_SENSOR_PARAMS_H

#include <math.h>

/******************************************/
/******************************************/
//COMPASS_ID_AK8975
#define COMPASS_AKM8975_RANGE           (1229.f)
#define COMPASS_AKM8975_RESOLUTION      (0.3f)
#define COMPASS_AKM8975_POWER           (10.0f)     // IDD2 (max)
#define COMPASS_AKM8975_MINDELAY        (10000)
#define COMPASS_AKM8975_MAXDELAY        (1000000)
//COMPASS_ID_AK8963C
#define COMPASS_AKM8963_RANGE           (4912.f)
#define COMPASS_AKM8963_RESOLUTION      (0.15f)
#define COMPASS_AKM8963_POWER           (10.0f)     // IDD2 (max)
#define COMPASS_AKM8963_MINDELAY        (10000)
#define COMPASS_AKM8963_MAXDELAY        (1000000)
//COMPASS_ID_AK09911
#define COMPASS_AKM9911_RANGE           (4912.f)
#define COMPASS_AKM9911_RESOLUTION      (0.6f)
#define COMPASS_AKM9911_POWER           (6.0f)      // IDD2 (max)
#define COMPASS_AKM9911_MINDELAY        (10000)
#define COMPASS_AKM9911_MAXDELAY        (1000000)
//COMPASS_ID_AK09912C
#define COMPASS_AKM9912_RANGE           (4912.f)
#define COMPASS_AKM9912_RESOLUTION      (0.15f)
#define COMPASS_AKM9912_POWER           (2.3f)      // IDD2 (max)
#define COMPASS_AKM9912_MINDELAY        (10000)
#define COMPASS_AKM9912_MAXDELAY        (1000000)
//COMPASS_ID_AK09916
#define COMPASS_AKM9916_RANGE           (4912.f)
#define COMPASS_AKM9916_RESOLUTION      (0.15f)
#define COMPASS_AKM9916_POWER           (3.0f)       // IDD2 (max)
#define COMPASS_AKM9916_MINDELAY        (10000)
#define COMPASS_AKM9916_MAXDELAY        (1000000)
//COMPASS_ID_AK09915
#define COMPASS_AKM9915_RANGE           (4912.f)
#define COMPASS_AKM9915_RESOLUTION      (0.15f)
#define COMPASS_AKM9915_POWER           (3.5f)       // IDD2 (max)
#define COMPASS_AKM9915_MINDELAY        (10000)
#define COMPASS_AKM9915_MAXDELAY        (1000000)
//COMPASS_ID_AMI306
#define COMPASS_AMI306_RANGE            (5461.f)
#define COMPASS_AMI306_RESOLUTION       (0.9f)
#define COMPASS_AMI306_POWER            (0.15f)
#define COMPASS_AMI306_MINDELAY         (10000)
#define COMPASS_AMI306_MAXDELAY         (1000000)
//COMPASS_ID_YAS53x
#define COMPASS_YAS53x_RANGE            (8001.f)
#define COMPASS_YAS53x_RESOLUTION       (0.012f)
#define COMPASS_YAS53x_POWER            (4.f)
#define COMPASS_YAS53x_MINDELAY         (10000)
#define COMPASS_YAS53x_MAXDELAY         (1000000)// driver decimate to 1hz for CTS

/******************************************/
// ACCEL ICM20602
#define ACCEL_ICM20602_RANGE             (2.f * GRAVITY_EARTH)
#define ACCEL_ICM20602_RESOLUTION        (2.f * GRAVITY_EARTH / 32768.f)
#define ACCEL_ICM20602_POWER             (0.4f)
#define ACCEL_ICM20602_MINDELAY          (5000)
#define ACCEL_ICM20602_MINDELAY_HIFI     (2000)
#define ACCEL_ICM20602_MAXDELAY          (250000)
// ACCEL ICM20690
#define ACCEL_ICM20690_RANGE             (2.f * GRAVITY_EARTH)
#define ACCEL_ICM20690_RESOLUTION        (2.f * GRAVITY_EARTH / 32768.f)
#define ACCEL_ICM20690_POWER             (0.5f)
#define ACCEL_ICM20690_MINDELAY          (5000)
#define ACCEL_ICM20690_MINDELAY_HIFI     (2000)
#define ACCEL_ICM20690_MAXDELAY          (250000)
// ACCEL IAM20680
#define ACCEL_IAM20680_RANGE             (2.f * GRAVITY_EARTH)
#define ACCEL_IAM20680_RESOLUTION        (2.f * GRAVITY_EARTH / 32768.f)
#define ACCEL_IAM20680_POWER             (0.5f)
#define ACCEL_IAM20680_MINDELAY          (5000)
#define ACCEL_IAM20680_MINDELAY_HIFI     (2000)
#define ACCEL_IAM20680_MAXDELAY          (250000)
//ACCEL ICM20648
#define ACCEL_ICM20648_RANGE             (2.f * GRAVITY_EARTH)
#define ACCEL_ICM20648_RESOLUTION        (2.f * GRAVITY_EARTH / 32768.f)
#define ACCEL_ICM20648_POWER             (0.5f)
#define ACCEL_ICM20648_MINDELAY          (4444)
#define ACCEL_ICM20648_MAXDELAY          (1000000)
//ACCEL ICM42600
#define ACCEL_ICM42600_RANGE             (2.f * GRAVITY_EARTH)
#define ACCEL_ICM42600_RESOLUTION        (2.f * GRAVITY_EARTH / 32768.f)
#define ACCEL_ICM42600_POWER             (0.35f)
#define ACCEL_ICM42600_MINDELAY          (5000)
#define ACCEL_ICM42600_MINDELAY_HIFI     (2000)
#define ACCEL_ICM42600_MAXDELAY          (320000)
//ACCEL ICM43600
#define ACCEL_ICM43600_RANGE             (2.f * GRAVITY_EARTH)
#define ACCEL_ICM43600_RESOLUTION        (2.f * GRAVITY_EARTH / 32768.f)
#define ACCEL_ICM43600_POWER             (0.35f)
#define ACCEL_ICM43600_MINDELAY          (5000)
#define ACCEL_ICM43600_MINDELAY_HIFI     (1250)
#define ACCEL_ICM43600_MAXDELAY          (320000)

/******************************************/
//GYRO ICM20602
#define GYRO_ICM20602_RANGE              (2000.f * M_PI / 180.f)
#define GYRO_ICM20602_RESOLUTION         (2000.f * M_PI / (180.f * 32768.f))
#define GYRO_ICM20602_POWER              (3.f)
#define GYRO_ICM20602_MINDELAY           (5000)
#define GYRO_ICM20602_MINDELAY_HIFI      (2000)
#define GYRO_ICM20602_MAXDELAY           (250000)
//GYRO ICM20690
#define GYRO_ICM20690_RANGE              (2000.f * M_PI / 180.f)
#define GYRO_ICM20690_RESOLUTION         (2000.f * M_PI / (180.f * 32768.f))
#define GYRO_ICM20690_POWER              (3.5f)
#define GYRO_ICM20690_MINDELAY           (5000)
#define GYRO_ICM20690_MINDELAY_HIFI      (2000)
#define GYRO_ICM20690_MAXDELAY           (250000)
//GYRO IAM20680
#define GYRO_IAM20680_RANGE              (2000.f * M_PI / 180.f)
#define GYRO_IAM20680_RESOLUTION         (2000.f * M_PI / (180.f * 32768.f))
#define GYRO_IAM20680_POWER              (3.f)
#define GYRO_IAM20680_MINDELAY           (5000)
#define GYRO_IAM20680_MINDELAY_HIFI      (2000)
#define GYRO_IAM20680_MAXDELAY           (250000)
//GYRO ICM20648
#define GYRO_ICM20648_RANGE              (2000.f * M_PI / 180.f)
#define GYRO_ICM20648_RESOLUTION         (2000.f * M_PI / (180.f * 32768.f))
#define GYRO_ICM20648_POWER              (5.5f)
#define GYRO_ICM20648_MINDELAY           (4444)
#define GYRO_ICM20648_MAXDELAY           (1000000)
//GYRO ICM42600
#define GYRO_ICM42600_RANGE              (2000.f * M_PI / 180.f)
#define GYRO_ICM42600_RESOLUTION         (2000.f * M_PI / (180.f * 32768.f))
#define GYRO_ICM42600_POWER              (0.75f)
#define GYRO_ICM42600_MINDELAY           (5000)
#define GYRO_ICM42600_MINDELAY_HIFI      (2000)
#define GYRO_ICM42600_MAXDELAY           (80000)
//GYRO ICM43600
#define GYRO_ICM43600_RANGE              (2000.f * M_PI / 180.f)
#define GYRO_ICM43600_RESOLUTION         (2000.f * M_PI / (180.f * 32768.f))
#define GYRO_ICM43600_POWER              (0.75f)
#define GYRO_ICM43600_MINDELAY           (5000)
#define GYRO_ICM43600_MINDELAY_HIFI      (1250)
#define GYRO_ICM43600_MAXDELAY           (80000)

/******************************************/
//PRESSURE BMP280
#define PRESSURE_BMP280_RANGE           (1100.f)   // hpa
#define PRESSURE_BMP280_RESOLUTION      (0.009995f)// in psi
#define PRESSURE_BMP280_POWER           (0.004f)   // 0.004mA
#define PRESSURE_BMP280_MINDELAY        (33333)    // 30Hz unit in ns
#define PRESSURE_BMP280_MAXDELAY        (1000000)  // driver decimate to 1hz for CTS
/******************************************/
//LIGHT APDS9930
#define LIGHT_APS9930_RANGE                     (10000)
#define LIGHT_APS9930_RESOLUTION                (0.009994f)
#define LIGHT_APS9930_POWER                     (0.175f)
#define LIGHT_APS9930_MINDELAY                  (112000)
#define LIGHT_APS9930_MAXDELAY                  (1000000)
/******************************************/
//PROXIMITY APDS9930
#define PROXIMITY_APS9930_RANGE                 (5)
#define PROXIMITY_APS9930_RESOLUTION            (0.10070801f)
#define PROXIMITY_APS9930_POWER                 (12.675f)
#define PROXIMITY_APS9930_MINDELAY              (112000)
#define PROXIMITY_APS9930_MAXDELAY              (1000000)

/******************************************/
// FIFO_SIZE
#define FIFO_SIZE_COMPUTE(_sz)    ((_sz) * 7 / 10 / 6)
#define FIFO_SIZE_IAM20680        512
#define FIFO_SIZE_ICM20648        512
#define FIFO_SIZE_ICM20602        1024
#define FIFO_SIZE_ICM20690        1024
#define FIFO_SIZE_ICM42600        2048
#define FIFO_SIZE_ICM43600        1024

#endif  /* INV_SENSOR_PARAMS_H */
