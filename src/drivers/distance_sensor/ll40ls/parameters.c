/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Lidar-Lite (LL40LS)
 *
 * @reboot_required true
 * @min 0
 * @max 2
 * @group Sensors
 * @value 0 Disabled
 * @value 1 PWM
 * @value 2 I2C
 */
PARAM_DEFINE_INT32(SENS_EN_LL40LS, 0);

/**
 * LidarLite Sensor 0 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_0_ROT, 0);

/**
 * LidarLite Sensor 1 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_1_ROT, 0);

/**
 * LidarLite Sensor 2 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_2_ROT, 0);

/**
 * LidarLite Sensor 3 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_3_ROT, 0);

/**
 * LidarLite Sensor 4 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_4_ROT, 0);

/**
 * LidarLite Sensor 5 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_5_ROT, 0);

/**
 * LidarLite Sensor 6 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_6_ROT, 0);

/**
 * LidarLite Sensor 7 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_7_ROT, 0);

/**
 * LidarLite Sensor 8 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_8_ROT, 0);

/**
 * LidarLite Sensor 9 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_9_ROT, 0);

/**
 * LidarLite Sensor 10 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_10_ROT, 0);

/**
 * LidarLite Sensor 12 Rotation
 *
 * This parameter defines the rotation of the LidarLite sensor relative to the platform.
 *
 * @reboot_required true
 * @min 0
 * @max 7
 * @group Sensors
 *
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 */
PARAM_DEFINE_INT32(LL40LS_11_ROT, 0);