/*
 * Copyright (C) 2014-2020 InvenSense, Inc.
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

#ifndef ANDROID_INVN_SENSORS_H
#define ANDROID_INVN_SENSORS_H

#include <math.h>
#include <hardware/sensors.h>

__BEGIN_DECLS

/*****************************************************************************/

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#endif

enum {
    ID_G = 0,
    ID_A,
    ID_M,
    ID_PS,
    ID_NUMBER
};

enum {
    Gyro = ID_G,
    Accelerometer = ID_A,
    MagneticField = ID_M,
    Pressure = ID_PS,
    TotalNumSensors = ID_NUMBER,
};

/* Physical parameters of the sensors supported by Invensense MPL */
#define SENSORS_GYROSCOPE_HANDLE                   (ID_G)
#define SENSORS_ACCELERATION_HANDLE                (ID_A)
#define SENSORS_MAGNETIC_FIELD_HANDLE              (ID_M)
#define SENSORS_PRESSURE_HANDLE                    (ID_PS)

__END_DECLS

#endif  // ANDROID_INVN_SENSORS_H
