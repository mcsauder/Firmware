/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file vl53l1x.h
 * Driver register map for the vl53l1x ToF Sensor from ST Microelectronics.
 */

#pragma once

#define VL53L1_SOFT_RESET                                                                0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS                                                 0x0001

#define VL53L1_ANA_CONFIG__VHV_REF_SEL_VDDPIX                                            0x0002
#define VL53L1_ANA_CONFIG__VHV_REF_SEL_VQUENCH                                           0x0003
#define VL53L1_ANA_CONFIG__REG_AVDD1V2_SEL                                               0x0004
#define VL53L1_ANA_CONFIG__FAST_OSC__TRIM                                                0x0005

#define VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY                                         0x0006
#define VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY_HI                                      0x0006
#define VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY_LO                                      0x0007

#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND                                     0x0008
#define VL53L1_VHV_CONFIG__COUNT_THRESH                                                  0x0009
#define VL53L1_VHV_CONFIG__OFFSET                                                        0x000A
#define VL53L1_VHV_CONFIG__INIT                                                          0x000B

#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_0                                         0x000D
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_1                                         0x000E
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_2                                         0x000F
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_3                                         0x0010
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_4                                         0x0011
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_5                                         0x0012
#define VL53L1_GLOBAL_CONFIG__REF_EN_START_SELECT                                        0x0013

#define VL53L1_REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS                                     0x0014
#define VL53L1_REF_SPAD_MAN__REF_LOCATION                                                0x0015

#define VL53L1_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS                            0x0016
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_HI                         0x0016
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS_LO                         0x0017
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS                        0x0018
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_HI                     0x0018
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS_LO                     0x0019
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS                        0x001A
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_HI                     0x001A
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS_LO                     0x001B

#define VL53L1_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS                                     0x001C
#define VL53L1_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS_HI                                  0x001C
#define VL53L1_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS_LO                                  0x001D

#define VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM                                        0x001E
#define VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM_HI                                     0x001E
#define VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM_LO                                     0x001F

#define VL53L1_MM_CONFIG__INNER_OFFSET_MM                                                0x0020
#define VL53L1_MM_CONFIG__INNER_OFFSET_MM_HI                                             0x0020
#define VL53L1_MM_CONFIG__INNER_OFFSET_MM_LO                                             0x0021

#define VL53L1_MM_CONFIG__OUTER_OFFSET_MM                                                0x0022
#define VL53L1_MM_CONFIG__OUTER_OFFSET_MM_HI                                             0x0022
#define VL53L1_MM_CONFIG__OUTER_OFFSET_MM_LO                                             0x0023

#define VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS                                        0x0024
#define VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_HI                                     0x0024
#define VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS_LO                                     0x0025

#define VL53L1_DEBUG__CTRL                                                               0x0026
#define VL53L1_TEST_MODE__CTRL                                                           0x0027
#define VL53L1_CLK_GATING__CTRL                                                          0x0028
#define VL53L1_NVM_BIST__CTRL                                                            0x0029
#define VL53L1_NVM_BIST__NUM_NVM_WORDS                                                   0x002A
#define VL53L1_NVM_BIST__START_ADDRESS                                                   0x002B
#define VL53L1_HOST_IF__STATUS                                                           0x002C

#define VL53L1_PAD_I2C_HV__CONFIG                                                        0x002D
#define VL53L1_PAD_I2C_HV__EXTSUP_CONFIG                                                 0x002E

#define VL53L1_GPIO_HV_PAD__CTRL                                                         0x002F
#define VL53L1_GPIO_HV_MUX__CTRL                                                         0x0030
#define VL53L1_GPIO__TIO_HV_STATUS                                                       0x0031
#define VL53L1_GPIO__FIO_HV_STATUS                                                       0x0032

#define VL53L1_ANA_CONFIG__SPAD_SEL_PSWIDTH                                              0x0033
#define VL53L1_ANA_CONFIG__VCSEL_PULSE_WIDTH_OFFSET                                      0x0034
#define VL53L1_ANA_CONFIG__FAST_OSC__CONFIG_CTRL                                         0x0035

#define VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS                                 0x0036
#define VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS                               0x0037
#define VL53L1_SIGMA_ESTIMATOR__SIGMA_REF_MM                                             0x0038

#define VL53L1_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM                              0x0039

#define VL53L1_SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_0                                  0x003A
#define VL53L1_SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_1                                  0x003B

#define VL53L1_ALGO__RANGE_IGNORE_THRESHOLD_MCPS                                         0x003C
#define VL53L1_ALGO__RANGE_IGNORE_THRESHOLD_MCPS_HI                                      0x003C
#define VL53L1_ALGO__RANGE_IGNORE_THRESHOLD_MCPS_LO                                      0x003D
#define VL53L1_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM                                        0x003E
#define VL53L1_ALGO__RANGE_MIN_CLIP                                                      0x003F

#define VL53L1_ALGO__CONSISTENCY_CHECK__TOLERANCE                                        0x0040

#define VL53L1_SPARE_HOST_CONFIG__STATIC_CONFIG_SPARE_2                                  0x0041

#define VL53L1_SD_CONFIG__RESET_STAGES_MSB                                               0x0042
#define VL53L1_SD_CONFIG__RESET_STAGES_LSB                                               0x0043

#define VL53L1_GPH_CONFIG__STREAM_COUNT_UPDATE_VALUE                                     0x0044
#define VL53L1_GLOBAL_CONFIG__STREAM_DIVIDER                                             0x0045
#define VL53L1_SYSTEM__INTERRUPT_CONFIG_GPIO                                             0x0046

#define VL53L1_CAL_CONFIG__VCSEL_START                                                   0x0047
#define VL53L1_CAL_CONFIG__REPEAT_RATE                                                   0x0048
#define VL53L1_CAL_CONFIG__REPEAT_RATE_HI                                                0x0048
#define VL53L1_CAL_CONFIG__REPEAT_RATE_LO                                                0x0049

#define VL53L1_GLOBAL_CONFIG__VCSEL_WIDTH                                                0x004A

#define VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP                                           0x004B
#define VL53L1_PHASECAL_CONFIG__TARGET                                                   0x004C
#define VL53L1_PHASECAL_CONFIG__OVERRIDE                                                 0x004D

#define VL53L1_DSS_CONFIG__ROI_MODE_CONTROL                                              0x004F

#define VL53L1_SYSTEM__THRESH_RATE_HIGH                                                  0x0050
#define VL53L1_SYSTEM__THRESH_RATE_HIGH_HI                                               0x0050
#define VL53L1_SYSTEM__THRESH_RATE_HIGH_LO                                               0x0051
#define VL53L1_SYSTEM__THRESH_RATE_LOW                                                   0x0052
#define VL53L1_SYSTEM__THRESH_RATE_LOW_HI                                                0x0052
#define VL53L1_SYSTEM__THRESH_RATE_LOW_LO                                                0x0053

#define VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT                                 0x0054
#define VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_HI                              0x0054
#define VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_LO                              0x0055
#define VL53L1_DSS_CONFIG__MANUAL_BLOCK_SELECT                                           0x0056
#define VL53L1_DSS_CONFIG__APERTURE_ATTENUATION                                          0x0057
#define VL53L1_DSS_CONFIG__MAX_SPADS_LIMIT                                               0x0058
#define VL53L1_DSS_CONFIG__MIN_SPADS_LIMIT                                               0x0059

#define VL53L1_MM_CONFIG__TIMEOUT_MACROP_A_HI                                            0x005A
#define VL53L1_MM_CONFIG__TIMEOUT_MACROP_A_LO                                            0x005B
#define VL53L1_MM_CONFIG__TIMEOUT_MACROP_B_HI                                            0x005C
#define VL53L1_MM_CONFIG__TIMEOUT_MACROP_B_LO                                            0x005D

#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI                                         0x005E
#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_LO                                         0x005F
#define VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A                                              0x0060

#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI                                         0x0061
#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_LO                                         0x0062
#define VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B                                              0x0063

#define VL53L1_RANGE_CONFIG__SIGMA_THRESH                                                0x0064
#define VL53L1_RANGE_CONFIG__SIGMA_THRESH_HI                                             0x0064
#define VL53L1_RANGE_CONFIG__SIGMA_THRESH_LO                                             0x0065

#define VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS                               0x0066
#define VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI                            0x0066
#define VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO                            0x0067

#define VL53L1_RANGE_CONFIG__VALID_PHASE_LOW                                             0x0068
#define VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH                                            0x0069

#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD                                           0x006C
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD_3                                         0x006C
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD_2                                         0x006D
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD_1                                         0x006E
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD_0                                         0x006F

#define VL53L1_SYSTEM__FRACTIONAL_ENABLE                                                 0x0070
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_0                                          0x0071

#define VL53L1_SYSTEM__THRESH_HIGH                                                       0x0072
#define VL53L1_SYSTEM__THRESH_HIGH_HI                                                    0x0072
#define VL53L1_SYSTEM__THRESH_HIGH_LO                                                    0x0073
#define VL53L1_SYSTEM__THRESH_LOW                                                        0x0074
#define VL53L1_SYSTEM__THRESH_LOW_HI                                                     0x0074
#define VL53L1_SYSTEM__THRESH_LOW_LO                                                     0x0075

#define VL53L1_SYSTEM__ENABLE_XTALK_PER_QUADRANT                                         0x0076
#define VL53L1_SYSTEM__SEED_CONFIG                                                       0x0077

#define VL53L1_SD_CONFIG__WOI_SD0                                                        0x0078
#define VL53L1_SD_CONFIG__WOI_SD1                                                        0x0079
#define VL53L1_SD_CONFIG__INITIAL_PHASE_SD0                                              0x007A
#define VL53L1_SD_CONFIG__INITIAL_PHASE_SD1                                              0x007B

#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_1                                          0x007C

#define VL53L1_SD_CONFIG__FIRST_ORDER_SELECT                                             0x007D
#define VL53L1_SD_CONFIG__QUANTIFIER                                                     0x007E

#define VL53L1_ROI_CONFIG__USER_ROI_CENTRE_SPAD                                          0x007F
#define VL53L1_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE                             0x0080

#define VL53L1_SYSTEM__SEQUENCE_CONFIG                                                   0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD                                            0x0082

#define VL53L1_POWER_MANAGEMENT__GO1_POWER_FORCE                                         0x0083
#define VL53L1_SYSTEM__STREAM_COUNT_CTRL                                                 0x0084
#define VL53L1_FIRMWARE__ENABLE                                                          0x0085
#define VL53L1_SYSTEM__INTERRUPT_CLEAR                                                   0x0086
#define VL53L1_SYSTEM__MODE_START                                                        0x0087

#define VL53L1_RESULT__INTERRUPT_STATUS                                                  0x0088
#define VL53L1_RESULT__RANGE_STATUS                                                      0x0089
#define VL53L1_RESULT__REPORT_STATUS                                                     0x008A
#define VL53L1_RESULT__STREAM_COUNT                                                      0x008B

#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0                                    0x008C
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI                                 0x008C
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO                                 0x008D

#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0                                   0x008E
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI                                0x008E
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO                                0x008F

#define VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0                                       0x0090
#define VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI                                    0x0090
#define VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO                                    0x0091

#define VL53L1_RESULT__SIGMA_SD0                                                         0x0092
#define VL53L1_RESULT__SIGMA_SD0_HI                                                      0x0092
#define VL53L1_RESULT__SIGMA_SD0_LO                                                      0x0093

#define VL53L1_RESULT__PHASE_SD0                                                         0x0094
#define VL53L1_RESULT__PHASE_SD0_HI                                                      0x0094
#define VL53L1_RESULT__PHASE_SD0_LO                                                      0x0095

#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                            0x0096
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI                         0x0096
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO                         0x0097

#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0               0x0098
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI            0x0098
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO            0x0099

#define VL53L1_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0                               0x009A
#define VL53L1_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                            0x009A
#define VL53L1_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                            0x009B
#define VL53L1_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0                               0x009C
#define VL53L1_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                            0x009C
#define VL53L1_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                            0x009D

#define VL53L1_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0                                    0x009E
#define VL53L1_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI                                 0x009E
#define VL53L1_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO                                 0x009F

#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1                                    0x00A0
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI                                 0x00A0
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO                                 0x00A1

#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1                                   0x00A2
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI                                0x00A2
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO                                0x00A3

#define VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1                                       0x00A4
#define VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI                                    0x00A4
#define VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO                                    0x00A5

#define VL53L1_RESULT__SIGMA_SD1                                                         0x00A6
#define VL53L1_RESULT__SIGMA_SD1_HI                                                      0x00A6
#define VL53L1_RESULT__SIGMA_SD1_LO                                                      0x00A7
#define VL53L1_RESULT__PHASE_SD1                                                         0x00A8
#define VL53L1_RESULT__PHASE_SD1_HI                                                      0x00A8
#define VL53L1_RESULT__PHASE_SD1_LO                                                      0x00A9

#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1                            0x00AA
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI                         0x00AA
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO                         0x00AB

#define VL53L1_RESULT__SPARE_0_SD1                                                       0x00AC
#define VL53L1_RESULT__SPARE_0_SD1_HI                                                    0x00AC
#define VL53L1_RESULT__SPARE_0_SD1_LO                                                    0x00AD
#define VL53L1_RESULT__SPARE_1_SD1                                                       0x00AE
#define VL53L1_RESULT__SPARE_1_SD1_HI                                                    0x00AE
#define VL53L1_RESULT__SPARE_1_SD1_LO                                                    0x00AF
#define VL53L1_RESULT__SPARE_2_SD1                                                       0x00B0
#define VL53L1_RESULT__SPARE_2_SD1_HI                                                    0x00B0
#define VL53L1_RESULT__SPARE_2_SD1_LO                                                    0x00B1
#define VL53L1_RESULT__SPARE_3_SD1                                                       0x00B2

#define VL53L1_RESULT__THRESH_INFO                                                       0x00B3

#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0                                    0x00B4
#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3                                  0x00B4
#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2                                  0x00B5
#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1                                  0x00B6
#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0                                  0x00B7

#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0                                     0x00B8
#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3                                   0x00B8
#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2                                   0x00B9
#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1                                   0x00BA
#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0                                   0x00BB

#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0                                      0x00BC
#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3                                    0x00BC
#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2                                    0x00BD
#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1                                    0x00BE
#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0                                    0x00BF

#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0                                    0x00C0
#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3                                  0x00C0
#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2                                  0x00C1
#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1                                  0x00C2
#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0                                  0x00C3

#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1                                    0x00C4
#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3                                  0x00C4
#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2                                  0x00C5
#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1                                  0x00C6
#define VL53L1_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0                                  0x00C7

#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1                                     0x00C8
#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3                                   0x00C8
#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2                                   0x00C9
#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1                                   0x00CA
#define VL53L1_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0                                   0x00CB

#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1                                      0x00CC
#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3                                    0x00CC
#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2                                    0x00CD
#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1                                    0x00CE
#define VL53L1_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0                                    0x00CF

#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1                                    0x00D0
#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3                                  0x00D0
#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2                                  0x00D1
#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1                                  0x00D2
#define VL53L1_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0                                  0x00D3

#define VL53L1_RESULT_CORE__SPARE_0                                                      0x00D4

#define VL53L1_PHASECAL_RESULT__REFERENCE_PHASE                                          0x00D6
#define VL53L1_PHASECAL_RESULT__REFERENCE_PHASE_HI                                       0x00D6
#define VL53L1_PHASECAL_RESULT__REFERENCE_PHASE_LO                                       0x00D7

#define VL53L1_PHASECAL_RESULT__VCSEL_START                                              0x00D8

#define VL53L1_REF_SPAD_CHAR_RESULT__NUM_ACTUAL_REF_SPADS                                0x00D9
#define VL53L1_REF_SPAD_CHAR_RESULT__REF_LOCATION                                        0x00DA

#define VL53L1_VHV_RESULT__COLDBOOT_STATUS                                               0x00DB
#define VL53L1_VHV_RESULT__SEARCH_RESULT                                                 0x00DC
#define VL53L1_VHV_RESULT__LATEST_SETTING                                                0x00DD

#define VL53L1_RESULT__OSC_CALIBRATE_VAL                                                 0x00DE
#define VL53L1_RESULT__OSC_CALIBRATE_VAL_HI                                              0x00DE
#define VL53L1_RESULT__OSC_CALIBRATE_VAL_LO                                              0x00DF

#define VL53L1_ANA_CONFIG__POWERDOWN_GO1                                                 0x00E0
#define VL53L1_ANA_CONFIG__REF_BG_CTRL                                                   0x00E1
#define VL53L1_ANA_CONFIG__REGDVDD1V2_CTRL                                               0x00E2
#define VL53L1_ANA_CONFIG__OSC_SLOW_CTRL                                                 0x00E3

#define VL53L1_TEST_MODE__STATUS                                                         0x00E4

#define VL53L1_FIRMWARE__SYSTEM_STATUS                                                   0x00E5
#define VL53L1_FIRMWARE__MODE_STATUS                                                     0x00E6
#define VL53L1_FIRMWARE__SECONDARY_MODE_STATUS                                           0x00E7

#define VL53L1_FIRMWARE__CAL_REPEAT_RATE_COUNTER                                         0x00E8
#define VL53L1_FIRMWARE__CAL_REPEAT_RATE_COUNTER_HI                                      0x00E8
#define VL53L1_FIRMWARE__CAL_REPEAT_RATE_COUNTER_LO                                      0x00E9

#define VL53L1_FIRMWARE__HISTOGRAM_BIN                                                   0x00EA

#define VL53L1_GPH__SYSTEM__THRESH_HIGH                                                  0x00EC
#define VL53L1_GPH__SYSTEM__THRESH_HIGH_HI                                               0x00EC
#define VL53L1_GPH__SYSTEM__THRESH_HIGH_LO                                               0x00ED
#define VL53L1_GPH__SYSTEM__THRESH_LOW                                                   0x00EE
#define VL53L1_GPH__SYSTEM__THRESH_LOW_HI                                                0x00EE
#define VL53L1_GPH__SYSTEM__THRESH_LOW_LO                                                0x00EF

#define VL53L1_GPH__SYSTEM__ENABLE_XTALK_PER_QUADRANT                                    0x00F0

#define VL53L1_GPH__SPARE_0                                                              0x00F1

#define VL53L1_GPH__SD_CONFIG__WOI_SD0                                                   0x00F2
#define VL53L1_GPH__SD_CONFIG__WOI_SD1                                                   0x00F3

#define VL53L1_GPH__SD_CONFIG__INITIAL_PHASE_SD0                                         0x00F4
#define VL53L1_GPH__SD_CONFIG__INITIAL_PHASE_SD1                                         0x00F5

#define VL53L1_GPH__SD_CONFIG__FIRST_ORDER_SELECT                                        0x00F6

#define VL53L1_GPH__SD_CONFIG__QUANTIFIER                                                0x00F7

#define VL53L1_GPH__ROI_CONFIG__USER_ROI_CENTRE_SPAD                                     0x00F8
#define VL53L1_GPH__ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE                        0x00F9

#define VL53L1_GPH__SYSTEM__SEQUENCE_CONFIG                                              0x00FA

#define VL53L1_GPH__GPH_ID                                                               0x00FB

#define VL53L1_SYSTEM__INTERRUPT_SET                                                     0x00FC

#define VL53L1_INTERRUPT_MANAGER__ENABLES                                                0x00FD
#define VL53L1_INTERRUPT_MANAGER__CLEAR                                                  0x00FE
#define VL53L1_INTERRUPT_MANAGER__STATUS                                                 0x00FF

#define VL53L1_MCU_TO_HOST_BANK__WR_ACCESS_EN                                            0x0100

#define VL53L1_POWER_MANAGEMENT__GO1_RESET_STATUS                                        0x0101

#define VL53L1_PAD_STARTUP_MODE__VALUE_RO                                                0x0102
#define VL53L1_PAD_STARTUP_MODE__VALUE_CTRL                                              0x0103

#define VL53L1_PLL_PERIOD_US                                                             0x0104
#define VL53L1_PLL_PERIOD_US_3                                                           0x0104
#define VL53L1_PLL_PERIOD_US_2                                                           0x0105
#define VL53L1_PLL_PERIOD_US_1                                                           0x0106
#define VL53L1_PLL_PERIOD_US_0                                                           0x0107

#define VL53L1_INTERRUPT_SCHEDULER__DATA_OUT                                             0x0108
#define VL53L1_INTERRUPT_SCHEDULER__DATA_OUT_3                                           0x0108
#define VL53L1_INTERRUPT_SCHEDULER__DATA_OUT_2                                           0x0109
#define VL53L1_INTERRUPT_SCHEDULER__DATA_OUT_1                                           0x010A
#define VL53L1_INTERRUPT_SCHEDULER__DATA_OUT_0                                           0x010B

#define VL53L1_NVM_BIST__COMPLETE                                                        0x010C
#define VL53L1_NVM_BIST__STATUS                                                          0x010D

#define VL53L1_IDENTIFICATION__MODEL_ID                                                  0x010F
#define VL53L1_IDENTIFICATION__MODULE_TYPE                                               0x0110
#define VL53L1_IDENTIFICATION__REVISION_ID                                               0x0111
#define VL53L1_IDENTIFICATION__MODULE_ID                                                 0x0112
#define VL53L1_IDENTIFICATION__MODULE_ID_HI                                              0x0112
#define VL53L1_IDENTIFICATION__MODULE_ID_LO                                              0x0113

#define VL53L1_ANA_CONFIG__FAST_OSC__TRIM_MAX                                            0x0114
#define VL53L1_ANA_CONFIG__FAST_OSC__FREQ_SET                                            0x0115
#define VL53L1_ANA_CONFIG__VCSEL_TRIM                                                    0x0116
#define VL53L1_ANA_CONFIG__VCSEL_SELION                                                  0x0117
#define VL53L1_ANA_CONFIG__VCSEL_SELION_MAX                                              0x0118

#define VL53L1_PROTECTED_LASER_SAFETY__LOCK_BIT                                          0x0119

#define VL53L1_LASER_SAFETY__KEY                                                         0x011A
#define VL53L1_LASER_SAFETY__KEY_RO                                                      0x011B
#define VL53L1_LASER_SAFETY__CLIP                                                        0x011C
#define VL53L1_LASER_SAFETY__MULT                                                        0x011D

#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_0                                         0x011E
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_1                                         0x011F
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_2                                         0x0120
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_3                                         0x0121
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_4                                         0x0122
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_5                                         0x0123
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_6                                         0x0124
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_7                                         0x0125
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_8                                         0x0126
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_9                                         0x0127
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_10                                        0x0128
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_11                                        0x0129
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_12                                        0x012A
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_13                                        0x012B
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_14                                        0x012C
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_15                                        0x012D
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_16                                        0x012E
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_17                                        0x012F
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_18                                        0x0130
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_19                                        0x0131
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_20                                        0x0132
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_21                                        0x0133
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_22                                        0x0134
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_23                                        0x0135
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_24                                        0x0136
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_25                                        0x0137
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_26                                        0x0138
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_27                                        0x0139
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_28                                        0x013A
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_29                                        0x013B
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_30                                        0x013C
#define VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_RTN_31                                        0x013D

#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD                                          0x013E
#define VL53L1_ROI_CONFIG__MODE_ROI_XY_SIZE                                              0x013F
#define VL53L1_GO2_HOST_BANK_ACCESS__OVERRIDE                                            0x0300

#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND                                         0x0400
#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND_3                                       0x0400
#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND_2                                       0x0401
#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND_1                                       0x0402
#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLICAND_0                                       0x0403
#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER                                           0x0404
#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER_3                                         0x0404
#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER_2                                         0x0405
#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER_1                                         0x0406
#define VL53L1_MCU_UTIL_MULTIPLIER__MULTIPLIER_0                                         0x0407

#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI                                           0x0408
#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI_3                                         0x0408
#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI_2                                         0x0409
#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI_1                                         0x040A
#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_HI_0                                         0x040B
#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO                                           0x040C
#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO_3                                         0x040C
#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO_2                                         0x040D
#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO_1                                         0x040E
#define VL53L1_MCU_UTIL_MULTIPLIER__PRODUCT_LO_0                                         0x040F

#define VL53L1_MCU_UTIL_MULTIPLIER__START                                                0x0410
#define VL53L1_MCU_UTIL_MULTIPLIER__STATUS                                               0x0411

#define VL53L1_MCU_UTIL_DIVIDER__START                                                   0x0412
#define VL53L1_MCU_UTIL_DIVIDER__STATUS                                                  0x0413
#define VL53L1_MCU_UTIL_DIVIDER__DIVIDEND                                                0x0414
#define VL53L1_MCU_UTIL_DIVIDER__DIVIDEND_3                                              0x0414
#define VL53L1_MCU_UTIL_DIVIDER__DIVIDEND_2                                              0x0415
#define VL53L1_MCU_UTIL_DIVIDER__DIVIDEND_1                                              0x0416
#define VL53L1_MCU_UTIL_DIVIDER__DIVIDEND_0                                              0x0417
#define VL53L1_MCU_UTIL_DIVIDER__DIVISOR                                                 0x0418
#define VL53L1_MCU_UTIL_DIVIDER__DIVISOR_3                                               0x0418
#define VL53L1_MCU_UTIL_DIVIDER__DIVISOR_2                                               0x0419
#define VL53L1_MCU_UTIL_DIVIDER__DIVISOR_1                                               0x041A
#define VL53L1_MCU_UTIL_DIVIDER__DIVISOR_0                                               0x041B
#define VL53L1_MCU_UTIL_DIVIDER__QUOTIENT                                                0x041C
#define VL53L1_MCU_UTIL_DIVIDER__QUOTIENT_3                                              0x041C
#define VL53L1_MCU_UTIL_DIVIDER__QUOTIENT_2                                              0x041D
#define VL53L1_MCU_UTIL_DIVIDER__QUOTIENT_1                                              0x041E
#define VL53L1_MCU_UTIL_DIVIDER__QUOTIENT_0                                              0x041F

#define VL53L1_TIMER0__VALUE_IN                                                          0x0420
#define VL53L1_TIMER0__VALUE_IN_3                                                        0x0420
#define VL53L1_TIMER0__VALUE_IN_2                                                        0x0421
#define VL53L1_TIMER0__VALUE_IN_1                                                        0x0422
#define VL53L1_TIMER0__VALUE_IN_0                                                        0x0423
#define VL53L1_TIMER1__VALUE_IN                                                          0x0424
#define VL53L1_TIMER1__VALUE_IN_3                                                        0x0424
#define VL53L1_TIMER1__VALUE_IN_2                                                        0x0425
#define VL53L1_TIMER1__VALUE_IN_1                                                        0x0426
#define VL53L1_TIMER1__VALUE_IN_0                                                        0x0427

#define VL53L1_TIMER0__CTRL                                                              0x0428
#define VL53L1_TIMER1__CTRL                                                              0x0429

#define VL53L1_MCU_GENERAL_PURPOSE__GP_0                                                 0x042C
#define VL53L1_MCU_GENERAL_PURPOSE__GP_1                                                 0x042D
#define VL53L1_MCU_GENERAL_PURPOSE__GP_2                                                 0x042E
#define VL53L1_MCU_GENERAL_PURPOSE__GP_3                                                 0x042F

#define VL53L1_MCU_RANGE_CALC__CONFIG                                                    0x0430

#define VL53L1_MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE                                    0x0432
#define VL53L1_MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE_HI                                 0x0432
#define VL53L1_MCU_RANGE_CALC__OFFSET_CORRECTED_RANGE_LO                                 0x0433

#define VL53L1_MCU_RANGE_CALC__SPARE_4                                                   0x0434
#define VL53L1_MCU_RANGE_CALC__SPARE_4_3                                                 0x0434
#define VL53L1_MCU_RANGE_CALC__SPARE_4_2                                                 0x0435
#define VL53L1_MCU_RANGE_CALC__SPARE_4_1                                                 0x0436
#define VL53L1_MCU_RANGE_CALC__SPARE_4_0                                                 0x0437

#define VL53L1_MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC                                 0x0438
#define VL53L1_MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC_HI                              0x0438
#define VL53L1_MCU_RANGE_CALC__AMBIENT_DURATION_PRE_CALC_LO                              0x0439

#define VL53L1_MCU_RANGE_CALC__ALGO_VCSEL_PERIOD                                         0x043C
#define VL53L1_MCU_RANGE_CALC__SPARE_5                                                   0x043D

#define VL53L1_MCU_RANGE_CALC__ALGO_TOTAL_PERIODS                                        0x043E
#define VL53L1_MCU_RANGE_CALC__ALGO_TOTAL_PERIODS_HI                                     0x043E
#define VL53L1_MCU_RANGE_CALC__ALGO_TOTAL_PERIODS_LO                                     0x043F
#define VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE                                          0x0440
#define VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE_3                                        0x0440
#define VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE_2                                        0x0441
#define VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE_1                                        0x0442
#define VL53L1_MCU_RANGE_CALC__ALGO_ACCUM_PHASE_0                                        0x0443
#define VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS                                        0x0444
#define VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_3                                      0x0444
#define VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_2                                      0x0445
#define VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_1                                      0x0446
#define VL53L1_MCU_RANGE_CALC__ALGO_SIGNAL_EVENTS_0                                      0x0447
#define VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS                                       0x0448
#define VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_3                                     0x0448
#define VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_2                                     0x0449
#define VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_1                                     0x044A
#define VL53L1_MCU_RANGE_CALC__ALGO_AMBIENT_EVENTS_0                                     0x044B

#define VL53L1_MCU_RANGE_CALC__SPARE_6                                                   0x044C
#define VL53L1_MCU_RANGE_CALC__SPARE_6_HI                                                0x044C
#define VL53L1_MCU_RANGE_CALC__SPARE_6_LO                                                0x044D

#define VL53L1_MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD                                  0x044E
#define VL53L1_MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD_HI                               0x044E
#define VL53L1_MCU_RANGE_CALC__ALGO_ADJUST_VCSEL_PERIOD_LO                               0x044F

#define VL53L1_MCU_RANGE_CALC__NUM_SPADS                                                 0x0450
#define VL53L1_MCU_RANGE_CALC__NUM_SPADS_HI                                              0x0450
#define VL53L1_MCU_RANGE_CALC__NUM_SPADS_LO                                              0x0451

#define VL53L1_MCU_RANGE_CALC__PHASE_OUTPUT                                              0x0452
#define VL53L1_MCU_RANGE_CALC__PHASE_OUTPUT_HI                                           0x0452
#define VL53L1_MCU_RANGE_CALC__PHASE_OUTPUT_LO                                           0x0453

#define VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS                                        0x0454
#define VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_3                                      0x0454
#define VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_2                                      0x0455
#define VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_1                                      0x0456
#define VL53L1_MCU_RANGE_CALC__RATE_PER_SPAD_MCPS_0                                      0x0457

#define VL53L1_MCU_RANGE_CALC__SPARE_7                                                   0x0458
#define VL53L1_MCU_RANGE_CALC__SPARE_8                                                   0x0459

#define VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS                                     0x045A
#define VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS_HI                                  0x045A
#define VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_MCPS_LO                                  0x045B
#define VL53L1_MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS                                      0x045C
#define VL53L1_MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS_HI                                   0x045C
#define VL53L1_MCU_RANGE_CALC__AVG_SIGNAL_RATE_MCPS_LO                                   0x045D

#define VL53L1_MCU_RANGE_CALC__AMBIENT_RATE_MCPS                                         0x045E
#define VL53L1_MCU_RANGE_CALC__AMBIENT_RATE_MCPS_HI                                      0x045E
#define VL53L1_MCU_RANGE_CALC__AMBIENT_RATE_MCPS_LO                                      0x045F

#define VL53L1_MCU_RANGE_CALC__XTALK                                                     0x0460
#define VL53L1_MCU_RANGE_CALC__XTALK_HI                                                  0x0460
#define VL53L1_MCU_RANGE_CALC__XTALK_LO                                                  0x0461
#define VL53L1_MCU_RANGE_CALC__CALC_STATUS                                               0x0462
#define VL53L1_MCU_RANGE_CALC__DEBUG                                                     0x0463

#define VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS                          0x0464
#define VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS_HI                       0x0464
#define VL53L1_MCU_RANGE_CALC__PEAK_SIGNAL_RATE_XTALK_CORR_MCPS_LO                       0x0465

#define VL53L1_MCU_RANGE_CALC__SPARE_0                                                   0x0468
#define VL53L1_MCU_RANGE_CALC__SPARE_1                                                   0x0469
#define VL53L1_MCU_RANGE_CALC__SPARE_2                                                   0x046A
#define VL53L1_MCU_RANGE_CALC__SPARE_3                                                   0x046B

#define VL53L1_PATCH__CTRL                                                               0x0470

#define VL53L1_PATCH__JMP_ENABLES                                                        0x0472
#define VL53L1_PATCH__JMP_ENABLES_HI                                                     0x0472
#define VL53L1_PATCH__JMP_ENABLES_LO                                                     0x0473

#define VL53L1_PATCH__DATA_ENABLES                                                       0x0474
#define VL53L1_PATCH__DATA_ENABLES_HI                                                    0x0474
#define VL53L1_PATCH__DATA_ENABLES_LO                                                    0x0475

#define VL53L1_PATCH__OFFSET_0                                                           0x0476
#define VL53L1_PATCH__OFFSET_0_HI                                                        0x0476
#define VL53L1_PATCH__OFFSET_0_LO                                                        0x0477
#define VL53L1_PATCH__OFFSET_1                                                           0x0478
#define VL53L1_PATCH__OFFSET_1_HI                                                        0x0478
#define VL53L1_PATCH__OFFSET_1_LO                                                        0x0479
#define VL53L1_PATCH__OFFSET_2                                                           0x047A
#define VL53L1_PATCH__OFFSET_2_HI                                                        0x047A
#define VL53L1_PATCH__OFFSET_2_LO                                                        0x047B
#define VL53L1_PATCH__OFFSET_3                                                           0x047C
#define VL53L1_PATCH__OFFSET_3_HI                                                        0x047C
#define VL53L1_PATCH__OFFSET_3_LO                                                        0x047D
#define VL53L1_PATCH__OFFSET_4                                                           0x047E
#define VL53L1_PATCH__OFFSET_4_HI                                                        0x047E
#define VL53L1_PATCH__OFFSET_4_LO                                                        0x047F
#define VL53L1_PATCH__OFFSET_5                                                           0x0480
#define VL53L1_PATCH__OFFSET_5_HI                                                        0x0480
#define VL53L1_PATCH__OFFSET_5_LO                                                        0x0481
#define VL53L1_PATCH__OFFSET_6                                                           0x0482
#define VL53L1_PATCH__OFFSET_6_HI                                                        0x0482
#define VL53L1_PATCH__OFFSET_6_LO                                                        0x0483
#define VL53L1_PATCH__OFFSET_7                                                           0x0484
#define VL53L1_PATCH__OFFSET_7_HI                                                        0x0484
#define VL53L1_PATCH__OFFSET_7_LO                                                        0x0485
#define VL53L1_PATCH__OFFSET_8                                                           0x0486
#define VL53L1_PATCH__OFFSET_8_HI                                                        0x0486
#define VL53L1_PATCH__OFFSET_8_LO                                                        0x0487
#define VL53L1_PATCH__OFFSET_9                                                           0x0488
#define VL53L1_PATCH__OFFSET_9_HI                                                        0x0488
#define VL53L1_PATCH__OFFSET_9_LO                                                        0x0489
#define VL53L1_PATCH__OFFSET_10                                                          0x048A
#define VL53L1_PATCH__OFFSET_10_HI                                                       0x048A
#define VL53L1_PATCH__OFFSET_10_LO                                                       0x048B
#define VL53L1_PATCH__OFFSET_11                                                          0x048C
#define VL53L1_PATCH__OFFSET_11_HI                                                       0x048C
#define VL53L1_PATCH__OFFSET_11_LO                                                       0x048D
#define VL53L1_PATCH__OFFSET_12                                                          0x048E
#define VL53L1_PATCH__OFFSET_12_HI                                                       0x048E
#define VL53L1_PATCH__OFFSET_12_LO                                                       0x048F
#define VL53L1_PATCH__OFFSET_13                                                          0x0490
#define VL53L1_PATCH__OFFSET_13_HI                                                       0x0490
#define VL53L1_PATCH__OFFSET_13_LO                                                       0x0491
#define VL53L1_PATCH__OFFSET_14                                                          0x0492
#define VL53L1_PATCH__OFFSET_14_HI                                                       0x0492
#define VL53L1_PATCH__OFFSET_14_LO                                                       0x0493
#define VL53L1_PATCH__OFFSET_15                                                          0x0494
#define VL53L1_PATCH__OFFSET_15_HI                                                       0x0494
#define VL53L1_PATCH__OFFSET_15_LO                                                       0x0495

#define VL53L1_PATCH__ADDRESS_0                                                          0x0496
#define VL53L1_PATCH__ADDRESS_0_HI                                                       0x0496
#define VL53L1_PATCH__ADDRESS_0_LO                                                       0x0497
#define VL53L1_PATCH__ADDRESS_1                                                          0x0498
#define VL53L1_PATCH__ADDRESS_1_HI                                                       0x0498
#define VL53L1_PATCH__ADDRESS_1_LO                                                       0x0499
#define VL53L1_PATCH__ADDRESS_2                                                          0x049A
#define VL53L1_PATCH__ADDRESS_2_HI                                                       0x049A
#define VL53L1_PATCH__ADDRESS_2_LO                                                       0x049B
#define VL53L1_PATCH__ADDRESS_3                                                          0x049C
#define VL53L1_PATCH__ADDRESS_3_HI                                                       0x049C
#define VL53L1_PATCH__ADDRESS_3_LO                                                       0x049D
#define VL53L1_PATCH__ADDRESS_4                                                          0x049E
#define VL53L1_PATCH__ADDRESS_4_HI                                                       0x049E
#define VL53L1_PATCH__ADDRESS_4_LO                                                       0x049F
#define VL53L1_PATCH__ADDRESS_5                                                          0x04A0
#define VL53L1_PATCH__ADDRESS_5_HI                                                       0x04A0
#define VL53L1_PATCH__ADDRESS_5_LO                                                       0x04A1
#define VL53L1_PATCH__ADDRESS_6                                                          0x04A2
#define VL53L1_PATCH__ADDRESS_6_HI                                                       0x04A2
#define VL53L1_PATCH__ADDRESS_6_LO                                                       0x04A3
#define VL53L1_PATCH__ADDRESS_7                                                          0x04A4
#define VL53L1_PATCH__ADDRESS_7_HI                                                       0x04A4
#define VL53L1_PATCH__ADDRESS_7_LO                                                       0x04A5
#define VL53L1_PATCH__ADDRESS_8                                                          0x04A6
#define VL53L1_PATCH__ADDRESS_8_HI                                                       0x04A6
#define VL53L1_PATCH__ADDRESS_8_LO                                                       0x04A7
#define VL53L1_PATCH__ADDRESS_9                                                          0x04A8
#define VL53L1_PATCH__ADDRESS_9_HI                                                       0x04A8
#define VL53L1_PATCH__ADDRESS_9_LO                                                       0x04A9
#define VL53L1_PATCH__ADDRESS_10                                                         0x04AA
#define VL53L1_PATCH__ADDRESS_10_HI                                                      0x04AA
#define VL53L1_PATCH__ADDRESS_10_LO                                                      0x04AB
#define VL53L1_PATCH__ADDRESS_11                                                         0x04AC
#define VL53L1_PATCH__ADDRESS_11_HI                                                      0x04AC
#define VL53L1_PATCH__ADDRESS_11_LO                                                      0x04AD
#define VL53L1_PATCH__ADDRESS_12                                                         0x04AE
#define VL53L1_PATCH__ADDRESS_12_HI                                                      0x04AE
#define VL53L1_PATCH__ADDRESS_12_LO                                                      0x04AF
#define VL53L1_PATCH__ADDRESS_13                                                         0x04B0
#define VL53L1_PATCH__ADDRESS_13_HI                                                      0x04B0
#define VL53L1_PATCH__ADDRESS_13_LO                                                      0x04B1
#define VL53L1_PATCH__ADDRESS_14                                                         0x04B2
#define VL53L1_PATCH__ADDRESS_14_HI                                                      0x04B2
#define VL53L1_PATCH__ADDRESS_14_LO                                                      0x04B3
#define VL53L1_PATCH__ADDRESS_15                                                         0x04B4
#define VL53L1_PATCH__ADDRESS_15_HI                                                      0x04B4
#define VL53L1_PATCH__ADDRESS_15_LO                                                      0x04B5

#define VL53L1_SPI_ASYNC_MUX__CTRL                                                       0x04C0
#define VL53L1_CLK__CONFIG                                                               0x04C4
#define VL53L1_GPIO_LV_MUX__CTRL                                                         0x04CC
#define VL53L1_GPIO_LV_PAD__CTRL                                                         0x04CD
#define VL53L1_PAD_I2C_LV__CONFIG                                                        0x04D0
#define VL53L1_PAD_STARTUP_MODE__VALUE_RO_GO1                                            0x04D4
#define VL53L1_HOST_IF__STATUS_GO1                                                       0x04D5
#define VL53L1_MCU_CLK_GATING__CTRL                                                      0x04D8
#define VL53L1_TEST__BIST_ROM_CTRL                                                       0x04E0
#define VL53L1_TEST__BIST_ROM_RESULT                                                     0x04E1
#define VL53L1_TEST__BIST_ROM_MCU_SIG                                                    0x04E2
#define VL53L1_TEST__BIST_ROM_MCU_SIG_HI                                                 0x04E2
#define VL53L1_TEST__BIST_ROM_MCU_SIG_LO                                                 0x04E3
#define VL53L1_TEST__BIST_RAM_CTRL                                                       0x04E4
#define VL53L1_TEST__BIST_RAM_RESULT                                                     0x04E5
#define VL53L1_TEST__TMC                                                                 0x04E8
#define VL53L1_TEST__PLL_BIST_MIN_THRESHOLD                                              0x04F0
#define VL53L1_TEST__PLL_BIST_MIN_THRESHOLD_HI                                           0x04F0
#define VL53L1_TEST__PLL_BIST_MIN_THRESHOLD_LO                                           0x04F1
#define VL53L1_TEST__PLL_BIST_MAX_THRESHOLD                                              0x04F2
#define VL53L1_TEST__PLL_BIST_MAX_THRESHOLD_HI                                           0x04F2
#define VL53L1_TEST__PLL_BIST_MAX_THRESHOLD_LO                                           0x04F3
#define VL53L1_TEST__PLL_BIST_COUNT_OUT                                                  0x04F4
#define VL53L1_TEST__PLL_BIST_COUNT_OUT_HI                                               0x04F4
#define VL53L1_TEST__PLL_BIST_COUNT_OUT_LO                                               0x04F5
#define VL53L1_TEST__PLL_BIST_GONOGO                                                     0x04F6
#define VL53L1_TEST__PLL_BIST_CTRL                                                       0x04F7

#define VL53L1_RANGING_CORE__DEVICE_ID                                                   0x0680
#define VL53L1_RANGING_CORE__REVISION_ID                                                 0x0681
#define VL53L1_RANGING_CORE__CLK_CTRL1                                                   0x0683
#define VL53L1_RANGING_CORE__CLK_CTRL2                                                   0x0684
#define VL53L1_RANGING_CORE__WOI_1                                                       0x0685
#define VL53L1_RANGING_CORE__WOI_REF_1                                                   0x0686
#define VL53L1_RANGING_CORE__START_RANGING                                               0x0687
#define VL53L1_RANGING_CORE__LOW_LIMIT_1                                                 0x0690
#define VL53L1_RANGING_CORE__HIGH_LIMIT_1                                                0x0691
#define VL53L1_RANGING_CORE__LOW_LIMIT_REF_1                                             0x0692
#define VL53L1_RANGING_CORE__HIGH_LIMIT_REF_1                                            0x0693
#define VL53L1_RANGING_CORE__QUANTIFIER_1_MSB                                            0x0694
#define VL53L1_RANGING_CORE__QUANTIFIER_1_LSB                                            0x0695
#define VL53L1_RANGING_CORE__QUANTIFIER_REF_1_MSB                                        0x0696
#define VL53L1_RANGING_CORE__QUANTIFIER_REF_1_LSB                                        0x0697
#define VL53L1_RANGING_CORE__AMBIENT_OFFSET_1_MSB                                        0x0698
#define VL53L1_RANGING_CORE__AMBIENT_OFFSET_1_LSB                                        0x0699
#define VL53L1_RANGING_CORE__AMBIENT_OFFSET_REF_1_MSB                                    0x069A
#define VL53L1_RANGING_CORE__AMBIENT_OFFSET_REF_1_LSB                                    0x069B
#define VL53L1_RANGING_CORE__FILTER_STRENGTH_1                                           0x069C
#define VL53L1_RANGING_CORE__FILTER_STRENGTH_REF_1                                       0x069D
#define VL53L1_RANGING_CORE__SIGNAL_EVENT_LIMIT_1_MSB                                    0x069E
#define VL53L1_RANGING_CORE__SIGNAL_EVENT_LIMIT_1_LSB                                    0x069F
#define VL53L1_RANGING_CORE__SIGNAL_EVENT_LIMIT_REF_1_MSB                                0x06A0
#define VL53L1_RANGING_CORE__SIGNAL_EVENT_LIMIT_REF_1_LSB                                0x06A1
#define VL53L1_RANGING_CORE__TIMEOUT_OVERALL_PERIODS_MSB                                 0x06A4
#define VL53L1_RANGING_CORE__TIMEOUT_OVERALL_PERIODS_LSB                                 0x06A5
#define VL53L1_RANGING_CORE__INVERT_HW                                                   0x06A6
#define VL53L1_RANGING_CORE__FORCE_HW                                                    0x06A7
#define VL53L1_RANGING_CORE__STATIC_HW_VALUE                                             0x06A8
#define VL53L1_RANGING_CORE__FORCE_CONTINUOUS_AMBIENT                                    0x06A9
#define VL53L1_RANGING_CORE__TEST_PHASE_SELECT_TO_FILTER                                 0x06AA
#define VL53L1_RANGING_CORE__TEST_PHASE_SELECT_TO_TIMING_GEN                             0x06AB
#define VL53L1_RANGING_CORE__INITIAL_PHASE_VALUE_1                                       0x06AC
#define VL53L1_RANGING_CORE__INITIAL_PHASE_VALUE_REF_1                                   0x06AD
#define VL53L1_RANGING_CORE__FORCE_UP_IN                                                 0x06AE
#define VL53L1_RANGING_CORE__FORCE_DN_IN                                                 0x06AF
#define VL53L1_RANGING_CORE__STATIC_UP_VALUE_1                                           0x06B0
#define VL53L1_RANGING_CORE__STATIC_UP_VALUE_REF_1                                       0x06B1
#define VL53L1_RANGING_CORE__STATIC_DN_VALUE_1                                           0x06B2
#define VL53L1_RANGING_CORE__STATIC_DN_VALUE_REF_1                                       0x06B3
#define VL53L1_RANGING_CORE__MONITOR_UP_DN                                               0x06B4
#define VL53L1_RANGING_CORE__INVERT_UP_DN                                                0x06B5
#define VL53L1_RANGING_CORE__CPUMP_1                                                     0x06B6
#define VL53L1_RANGING_CORE__CPUMP_2                                                     0x06B7
#define VL53L1_RANGING_CORE__CPUMP_3                                                     0x06B8
#define VL53L1_RANGING_CORE__OSC_1                                                       0x06B9
#define VL53L1_RANGING_CORE__PLL_1                                                       0x06BB
#define VL53L1_RANGING_CORE__PLL_2                                                       0x06BC
#define VL53L1_RANGING_CORE__REFERENCE_1                                                 0x06BD
#define VL53L1_RANGING_CORE__REFERENCE_3                                                 0x06BF
#define VL53L1_RANGING_CORE__REFERENCE_4                                                 0x06C0
#define VL53L1_RANGING_CORE__REFERENCE_5                                                 0x06C1
#define VL53L1_RANGING_CORE__REGAVDD1V2                                                  0x06C3
#define VL53L1_RANGING_CORE__CALIB_1                                                     0x06C4
#define VL53L1_RANGING_CORE__CALIB_2                                                     0x06C5
#define VL53L1_RANGING_CORE__CALIB_3                                                     0x06C6
#define VL53L1_RANGING_CORE__TST_MUX_SEL1                                                0x06C9
#define VL53L1_RANGING_CORE__TST_MUX_SEL2                                                0x06CA
#define VL53L1_RANGING_CORE__TST_MUX                                                     0x06CB
#define VL53L1_RANGING_CORE__GPIO_OUT_TESTMUX                                            0x06CC
#define VL53L1_RANGING_CORE__CUSTOM_FE                                                   0x06CD
#define VL53L1_RANGING_CORE__CUSTOM_FE_2                                                 0x06CE
#define VL53L1_RANGING_CORE__SPAD_READOUT                                                0x06CF
#define VL53L1_RANGING_CORE__SPAD_READOUT_1                                              0x06D0
#define VL53L1_RANGING_CORE__SPAD_READOUT_2                                              0x06D1
#define VL53L1_RANGING_CORE__SPAD_PS                                                     0x06D2
#define VL53L1_RANGING_CORE__LASER_SAFETY_2                                              0x06D4
#define VL53L1_RANGING_CORE__NVM_CTRL__MODE                                              0x0780
#define VL53L1_RANGING_CORE__NVM_CTRL__PDN                                               0x0781
#define VL53L1_RANGING_CORE__NVM_CTRL__PROGN                                             0x0782
#define VL53L1_RANGING_CORE__NVM_CTRL__READN                                             0x0783
#define VL53L1_RANGING_CORE__NVM_CTRL__PULSE_WIDTH_MSB                                   0x0784
#define VL53L1_RANGING_CORE__NVM_CTRL__PULSE_WIDTH_LSB                                   0x0785
#define VL53L1_RANGING_CORE__NVM_CTRL__HV_RISE_MSB                                       0x0786
#define VL53L1_RANGING_CORE__NVM_CTRL__HV_RISE_LSB                                       0x0787
#define VL53L1_RANGING_CORE__NVM_CTRL__HV_FALL_MSB                                       0x0788
#define VL53L1_RANGING_CORE__NVM_CTRL__HV_FALL_LSB                                       0x0789
#define VL53L1_RANGING_CORE__NVM_CTRL__TST                                               0x078A
#define VL53L1_RANGING_CORE__NVM_CTRL__TESTREAD                                          0x078B
#define VL53L1_RANGING_CORE__NVM_CTRL__DATAIN_MMM                                        0x078C
#define VL53L1_RANGING_CORE__NVM_CTRL__DATAIN_LMM                                        0x078D
#define VL53L1_RANGING_CORE__NVM_CTRL__DATAIN_LLM                                        0x078E
#define VL53L1_RANGING_CORE__NVM_CTRL__DATAIN_LLL                                        0x078F
#define VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_MMM                                       0x0790
#define VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_LMM                                       0x0791
#define VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_LLM                                       0x0792
#define VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_LLL                                       0x0793
#define VL53L1_RANGING_CORE__NVM_CTRL__ADDR                                              0x0794
#define VL53L1_RANGING_CORE__NVM_CTRL__DATAOUT_ECC                                       0x0795
#define VL53L1_RANGING_CORE__RET_SPAD_EN_0                                               0x0796
#define VL53L1_RANGING_CORE__RET_SPAD_EN_1                                               0x0797
#define VL53L1_RANGING_CORE__RET_SPAD_EN_2                                               0x0798
#define VL53L1_RANGING_CORE__RET_SPAD_EN_3                                               0x0799
#define VL53L1_RANGING_CORE__RET_SPAD_EN_4                                               0x079A
#define VL53L1_RANGING_CORE__RET_SPAD_EN_5                                               0x079B
#define VL53L1_RANGING_CORE__RET_SPAD_EN_6                                               0x079C
#define VL53L1_RANGING_CORE__RET_SPAD_EN_7                                               0x079D
#define VL53L1_RANGING_CORE__RET_SPAD_EN_8                                               0x079E
#define VL53L1_RANGING_CORE__RET_SPAD_EN_9                                               0x079F
#define VL53L1_RANGING_CORE__RET_SPAD_EN_10                                              0x07A0
#define VL53L1_RANGING_CORE__RET_SPAD_EN_11                                              0x07A1
#define VL53L1_RANGING_CORE__RET_SPAD_EN_12                                              0x07A2
#define VL53L1_RANGING_CORE__RET_SPAD_EN_13                                              0x07A3
#define VL53L1_RANGING_CORE__RET_SPAD_EN_14                                              0x07A4
#define VL53L1_RANGING_CORE__RET_SPAD_EN_15                                              0x07A5
#define VL53L1_RANGING_CORE__RET_SPAD_EN_16                                              0x07A6
#define VL53L1_RANGING_CORE__RET_SPAD_EN_17                                              0x07A7
#define VL53L1_RANGING_CORE__SPAD_SHIFT_EN                                               0x07BA
#define VL53L1_RANGING_CORE__SPAD_DISABLE_CTRL                                           0x07BB
#define VL53L1_RANGING_CORE__SPAD_EN_SHIFT_OUT_DEBUG                                     0x07BC
#define VL53L1_RANGING_CORE__SPI_MODE                                                    0x07BD
#define VL53L1_RANGING_CORE__GPIO_DIR                                                    0x07BE
#define VL53L1_RANGING_CORE__VCSEL_PERIOD                                                0x0880
#define VL53L1_RANGING_CORE__VCSEL_START                                                 0x0881
#define VL53L1_RANGING_CORE__VCSEL_STOP                                                  0x0882
#define VL53L1_RANGING_CORE__VCSEL_1                                                     0x0885
#define VL53L1_RANGING_CORE__VCSEL_STATUS                                                0x088D
#define VL53L1_RANGING_CORE__STATUS                                                      0x0980
#define VL53L1_RANGING_CORE__LASER_CONTINUITY_STATE                                      0x0981
#define VL53L1_RANGING_CORE__RANGE_1_MMM                                                 0x0982
#define VL53L1_RANGING_CORE__RANGE_1_LMM                                                 0x0983
#define VL53L1_RANGING_CORE__RANGE_1_LLM                                                 0x0984
#define VL53L1_RANGING_CORE__RANGE_1_LLL                                                 0x0985
#define VL53L1_RANGING_CORE__RANGE_REF_1_MMM                                             0x0986
#define VL53L1_RANGING_CORE__RANGE_REF_1_LMM                                             0x0987
#define VL53L1_RANGING_CORE__RANGE_REF_1_LLM                                             0x0988
#define VL53L1_RANGING_CORE__RANGE_REF_1_LLL                                             0x0989
#define VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_MMM                                 0x098A
#define VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LMM                                 0x098B
#define VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LLM                                 0x098C
#define VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_1_LLL                                 0x098D
#define VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_1_MMM                                  0x098E
#define VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_1_LMM                                  0x098F
#define VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_1_LLM                                  0x0990
#define VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_1_LLL                                  0x0991
#define VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_MMM                                   0x0992
#define VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LMM                                   0x0993
#define VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LLM                                   0x0994
#define VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_1_LLL                                   0x0995
#define VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_MM                                  0x0996
#define VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_LM                                  0x0997
#define VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_1_LL                                  0x0998
#define VL53L1_RANGING_CORE__AMBIENT_MISMATCH_MM                                         0x0999
#define VL53L1_RANGING_CORE__AMBIENT_MISMATCH_LM                                         0x099A
#define VL53L1_RANGING_CORE__AMBIENT_MISMATCH_LL                                         0x099B
#define VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_MMM                             0x099C
#define VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LMM                             0x099D
#define VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LLM                             0x099E
#define VL53L1_RANGING_CORE__AMBIENT_WINDOW_EVENTS_REF_1_LLL                             0x099F
#define VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_MMM                              0x09A0
#define VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LMM                              0x09A1
#define VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LLM                              0x09A2
#define VL53L1_RANGING_CORE__RANGING_TOTAL_EVENTS_REF_1_LLL                              0x09A3
#define VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_MMM                               0x09A4
#define VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LMM                               0x09A5
#define VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LLM                               0x09A6
#define VL53L1_RANGING_CORE__SIGNAL_TOTAL_EVENTS_REF_1_LLL                               0x09A7
#define VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_MM                              0x09A8
#define VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_LM                              0x09A9
#define VL53L1_RANGING_CORE__TOTAL_PERIODS_ELAPSED_REF_1_LL                              0x09AA
#define VL53L1_RANGING_CORE__AMBIENT_MISMATCH_REF_MM                                     0x09AB
#define VL53L1_RANGING_CORE__AMBIENT_MISMATCH_REF_LM                                     0x09AC
#define VL53L1_RANGING_CORE__AMBIENT_MISMATCH_REF_LL                                     0x09AD
#define VL53L1_RANGING_CORE__GPIO_CONFIG__A0                                             0x0A00
#define VL53L1_RANGING_CORE__RESET_CONTROL__A0                                           0x0A01
#define VL53L1_RANGING_CORE__INTR_MANAGER__A0                                            0x0A02
#define VL53L1_RANGING_CORE__POWER_FSM_TIME_OSC__A0                                      0x0A06
#define VL53L1_RANGING_CORE__VCSEL_ATEST__A0                                             0x0A07
#define VL53L1_RANGING_CORE__VCSEL_PERIOD_CLIPPED__A0                                    0x0A08
#define VL53L1_RANGING_CORE__VCSEL_STOP_CLIPPED__A0                                      0x0A09
#define VL53L1_RANGING_CORE__CALIB_2__A0                                                 0x0A0A
#define VL53L1_RANGING_CORE__STOP_CONDITION__A0                                          0x0A0B
#define VL53L1_RANGING_CORE__STATUS_RESET__A0                                            0x0A0C
#define VL53L1_RANGING_CORE__READOUT_CFG__A0                                             0x0A0D
#define VL53L1_RANGING_CORE__WINDOW_SETTING__A0                                          0x0A0E
#define VL53L1_RANGING_CORE__VCSEL_DELAY__A0                                             0x0A1A
#define VL53L1_RANGING_CORE__REFERENCE_2__A0                                             0x0A1B
#define VL53L1_RANGING_CORE__REGAVDD1V2__A0                                              0x0A1D
#define VL53L1_RANGING_CORE__TST_MUX__A0                                                 0x0A1F
#define VL53L1_RANGING_CORE__CUSTOM_FE_2__A0                                             0x0A20
#define VL53L1_RANGING_CORE__SPAD_READOUT__A0                                            0x0A21
#define VL53L1_RANGING_CORE__CPUMP_1__A0                                                 0x0A22
#define VL53L1_RANGING_CORE__SPARE_REGISTER__A0                                          0x0A23
#define VL53L1_RANGING_CORE__VCSEL_CONT_STAGE5_BYPASS__A0                                0x0A24
#define VL53L1_RANGING_CORE__RET_SPAD_EN_18                                              0x0A25
#define VL53L1_RANGING_CORE__RET_SPAD_EN_19                                              0x0A26
#define VL53L1_RANGING_CORE__RET_SPAD_EN_20                                              0x0A27
#define VL53L1_RANGING_CORE__RET_SPAD_EN_21                                              0x0A28
#define VL53L1_RANGING_CORE__RET_SPAD_EN_22                                              0x0A29
#define VL53L1_RANGING_CORE__RET_SPAD_EN_23                                              0x0A2A
#define VL53L1_RANGING_CORE__RET_SPAD_EN_24                                              0x0A2B
#define VL53L1_RANGING_CORE__RET_SPAD_EN_25                                              0x0A2C
#define VL53L1_RANGING_CORE__RET_SPAD_EN_26                                              0x0A2D
#define VL53L1_RANGING_CORE__RET_SPAD_EN_27                                              0x0A2E
#define VL53L1_RANGING_CORE__RET_SPAD_EN_28                                              0x0A2F
#define VL53L1_RANGING_CORE__RET_SPAD_EN_29                                              0x0A30
#define VL53L1_RANGING_CORE__RET_SPAD_EN_30                                              0x0A31
#define VL53L1_RANGING_CORE__RET_SPAD_EN_31                                              0x0A32
#define VL53L1_RANGING_CORE__REF_SPAD_EN_0__EWOK                                         0x0A33
#define VL53L1_RANGING_CORE__REF_SPAD_EN_1__EWOK                                         0x0A34
#define VL53L1_RANGING_CORE__REF_SPAD_EN_2__EWOK                                         0x0A35
#define VL53L1_RANGING_CORE__REF_SPAD_EN_3__EWOK                                         0x0A36
#define VL53L1_RANGING_CORE__REF_SPAD_EN_4__EWOK                                         0x0A37
#define VL53L1_RANGING_CORE__REF_SPAD_EN_5__EWOK                                         0x0A38
#define VL53L1_RANGING_CORE__REF_EN_START_SELECT                                         0x0A39
#define VL53L1_RANGING_CORE__REGDVDD1V2_ATEST__EWOK                                      0x0A41

#define VL53L1_SOFT_RESET_GO1                                                            0x0B00

#define VL53L1_PRIVATE__PATCH_BASE_ADDR_RSLV                                             0x0E00

#define VL53L1_PREV_SHADOW_RESULT__INTERRUPT_STATUS                                      0x0ED0
#define VL53L1_PREV_SHADOW_RESULT__RANGE_STATUS                                          0x0ED1
#define VL53L1_PREV_SHADOW_RESULT__REPORT_STATUS                                         0x0ED2
#define VL53L1_PREV_SHADOW_RESULT__STREAM_COUNT                                          0x0ED3
#define VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0                        0x0ED4
#define VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI                     0x0ED4
#define VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO                     0x0ED5
#define VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0                       0x0ED6
#define VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI                    0x0ED6
#define VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO                    0x0ED7
#define VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0                           0x0ED8
#define VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI                        0x0ED8
#define VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO                        0x0ED9
#define VL53L1_PREV_SHADOW_RESULT__SIGMA_SD0                                             0x0EDA
#define VL53L1_PREV_SHADOW_RESULT__SIGMA_SD0_HI                                          0x0EDA
#define VL53L1_PREV_SHADOW_RESULT__SIGMA_SD0_LO                                          0x0EDB
#define VL53L1_PREV_SHADOW_RESULT__PHASE_SD0                                             0x0EDC
#define VL53L1_PREV_SHADOW_RESULT__PHASE_SD0_HI                                          0x0EDC
#define VL53L1_PREV_SHADOW_RESULT__PHASE_SD0_LO                                          0x0EDD
#define VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                0x0EDE
#define VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI             0x0EDE
#define VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO             0x0EDF
#define VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0   0x0EE0
#define VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI 0x0EE0
#define VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO 0x0EE1
#define VL53L1_PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0                   0x0EE2
#define VL53L1_PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                0x0EE2
#define VL53L1_PREV_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                0x0EE3
#define VL53L1_PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0                   0x0EE4
#define VL53L1_PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                0x0EE4
#define VL53L1_PREV_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                0x0EE5
#define VL53L1_PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0                        0x0EE6
#define VL53L1_PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI                     0x0EE6
#define VL53L1_PREV_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO                     0x0EE7
#define VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1                        0x0EE8
#define VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI                     0x0EE8
#define VL53L1_PREV_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO                     0x0EE9
#define VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1                       0x0EEA
#define VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI                    0x0EEA
#define VL53L1_PREV_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO                    0x0EEB
#define VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1                           0x0EEC
#define VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI                        0x0EEC
#define VL53L1_PREV_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO                        0x0EED
#define VL53L1_PREV_SHADOW_RESULT__SIGMA_SD1                                             0x0EEE
#define VL53L1_PREV_SHADOW_RESULT__SIGMA_SD1_HI                                          0x0EEE
#define VL53L1_PREV_SHADOW_RESULT__SIGMA_SD1_LO                                          0x0EEF
#define VL53L1_PREV_SHADOW_RESULT__PHASE_SD1                                             0x0EF0
#define VL53L1_PREV_SHADOW_RESULT__PHASE_SD1_HI                                          0x0EF0
#define VL53L1_PREV_SHADOW_RESULT__PHASE_SD1_LO                                          0x0EF1
#define VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1                0x0EF2
#define VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI             0x0EF2
#define VL53L1_PREV_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO             0x0EF3
#define VL53L1_PREV_SHADOW_RESULT__SPARE_0_SD1                                           0x0EF4
#define VL53L1_PREV_SHADOW_RESULT__SPARE_0_SD1_HI                                        0x0EF4
#define VL53L1_PREV_SHADOW_RESULT__SPARE_0_SD1_LO                                        0x0EF5
#define VL53L1_PREV_SHADOW_RESULT__SPARE_1_SD1                                           0x0EF6
#define VL53L1_PREV_SHADOW_RESULT__SPARE_1_SD1_HI                                        0x0EF6
#define VL53L1_PREV_SHADOW_RESULT__SPARE_1_SD1_LO                                        0x0EF7
#define VL53L1_PREV_SHADOW_RESULT__SPARE_2_SD1                                           0x0EF8
#define VL53L1_PREV_SHADOW_RESULT__SPARE_2_SD1_HI                                        0x0EF8
#define VL53L1_PREV_SHADOW_RESULT__SPARE_2_SD1_LO                                        0x0EF9
#define VL53L1_PREV_SHADOW_RESULT__SPARE_3_SD1                                           0x0EFA
#define VL53L1_PREV_SHADOW_RESULT__SPARE_3_SD1_HI                                        0x0EFA
#define VL53L1_PREV_SHADOW_RESULT__SPARE_3_SD1_LO                                        0x0EFB

#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0                        0x0EFC
#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3                      0x0EFC
#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2                      0x0EFD
#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1                      0x0EFE
#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0                      0x0EFF
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0                         0x0F00
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3                       0x0F00
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2                       0x0F01
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1                       0x0F02
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0                       0x0F03
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0                          0x0F04
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3                        0x0F04
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2                        0x0F05
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1                        0x0F06
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0                        0x0F07
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0                        0x0F08
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3                      0x0F08
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2                      0x0F09
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1                      0x0F0A
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0                      0x0F0B
#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1                        0x0F0C
#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3                      0x0F0C
#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2                      0x0F0D
#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1                      0x0F0E
#define VL53L1_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0                      0x0F0F
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1                         0x0F10
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3                       0x0F10
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2                       0x0F11
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1                       0x0F12
#define VL53L1_PREV_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0                       0x0F13
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1                          0x0F14
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3                        0x0F14
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2                        0x0F15
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1                        0x0F16
#define VL53L1_PREV_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0                        0x0F17
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1                        0x0F18
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3                      0x0F18
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2                      0x0F19
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1                      0x0F1A
#define VL53L1_PREV_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0                      0x0F1B
#define VL53L1_PREV_SHADOW_RESULT_CORE__SPARE_0                                          0x0F1C

#define VL53L1_RESULT__DEBUG_STATUS                                                      0x0F20
#define VL53L1_RESULT__DEBUG_STAGE                                                       0x0F21

#define VL53L1_GPH__SYSTEM__THRESH_RATE_HIGH                                             0x0F24
#define VL53L1_GPH__SYSTEM__THRESH_RATE_HIGH_HI                                          0x0F24
#define VL53L1_GPH__SYSTEM__THRESH_RATE_HIGH_LO                                          0x0F25
#define VL53L1_GPH__SYSTEM__THRESH_RATE_LOW                                              0x0F26
#define VL53L1_GPH__SYSTEM__THRESH_RATE_LOW_HI                                           0x0F26
#define VL53L1_GPH__SYSTEM__THRESH_RATE_LOW_LO                                           0x0F27
#define VL53L1_GPH__SYSTEM__INTERRUPT_CONFIG_GPIO                                        0x0F28

#define VL53L1_GPH__DSS_CONFIG__ROI_MODE_CONTROL                                         0x0F2F
#define VL53L1_GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT                            0x0F30
#define VL53L1_GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_HI                         0x0F30
#define VL53L1_GPH__DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT_LO                         0x0F31
#define VL53L1_GPH__DSS_CONFIG__MANUAL_BLOCK_SELECT                                      0x0F32
#define VL53L1_GPH__DSS_CONFIG__MAX_SPADS_LIMIT                                          0x0F33
#define VL53L1_GPH__DSS_CONFIG__MIN_SPADS_LIMIT                                          0x0F34

#define VL53L1_GPH__MM_CONFIG__TIMEOUT_MACROP_A_HI                                       0x0F36
#define VL53L1_GPH__MM_CONFIG__TIMEOUT_MACROP_A_LO                                       0x0F37
#define VL53L1_GPH__MM_CONFIG__TIMEOUT_MACROP_B_HI                                       0x0F38
#define VL53L1_GPH__MM_CONFIG__TIMEOUT_MACROP_B_LO                                       0x0F39

#define VL53L1_GPH__RANGE_CONFIG__TIMEOUT_MACROP_A_HI                                    0x0F3A
#define VL53L1_GPH__RANGE_CONFIG__TIMEOUT_MACROP_A_LO                                    0x0F3B
#define VL53L1_GPH__RANGE_CONFIG__VCSEL_PERIOD_A                                         0x0F3C
#define VL53L1_GPH__RANGE_CONFIG__VCSEL_PERIOD_B                                         0x0F3D
#define VL53L1_GPH__RANGE_CONFIG__TIMEOUT_MACROP_B_HI                                    0x0F3E
#define VL53L1_GPH__RANGE_CONFIG__TIMEOUT_MACROP_B_LO                                    0x0F3F
#define VL53L1_GPH__RANGE_CONFIG__SIGMA_THRESH                                           0x0F40
#define VL53L1_GPH__RANGE_CONFIG__SIGMA_THRESH_HI                                        0x0F40
#define VL53L1_GPH__RANGE_CONFIG__SIGMA_THRESH_LO                                        0x0F41
#define VL53L1_GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS                          0x0F42
#define VL53L1_GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_HI                       0x0F42
#define VL53L1_GPH__RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS_LO                       0x0F43
#define VL53L1_GPH__RANGE_CONFIG__VALID_PHASE_LOW                                        0x0F44
#define VL53L1_GPH__RANGE_CONFIG__VALID_PHASE_HIGH                                       0x0F45

#define VL53L1_FIRMWARE__INTERNAL_STREAM_COUNT_DIV                                       0x0F46
#define VL53L1_FIRMWARE__INTERNAL_STREAM_COUNTER_VAL                                     0x0F47

#define VL53L1_DSS_CALC__ROI_CTRL                                                        0x0F54
#define VL53L1_DSS_CALC__SPARE_1                                                         0x0F55
#define VL53L1_DSS_CALC__SPARE_2                                                         0x0F56
#define VL53L1_DSS_CALC__SPARE_3                                                         0x0F57
#define VL53L1_DSS_CALC__SPARE_4                                                         0x0F58
#define VL53L1_DSS_CALC__SPARE_5                                                         0x0F59
#define VL53L1_DSS_CALC__SPARE_6                                                         0x0F5A
#define VL53L1_DSS_CALC__SPARE_7                                                         0x0F5B
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_0                                              0x0F5C
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_1                                              0x0F5D
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_2                                              0x0F5E
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_3                                              0x0F5F
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_4                                              0x0F60
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_5                                              0x0F61
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_6                                              0x0F62
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_7                                              0x0F63
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_8                                              0x0F64
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_9                                              0x0F65
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_10                                             0x0F66
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_11                                             0x0F67
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_12                                             0x0F68
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_13                                             0x0F69
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_14                                             0x0F6A
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_15                                             0x0F6B
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_16                                             0x0F6C
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_17                                             0x0F6D
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_18                                             0x0F6E
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_19                                             0x0F6F
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_20                                             0x0F70
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_21                                             0x0F71
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_22                                             0x0F72
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_23                                             0x0F73
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_24                                             0x0F74
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_25                                             0x0F75
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_26                                             0x0F76
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_27                                             0x0F77
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_28                                             0x0F78
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_29                                             0x0F79
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_30                                             0x0F7A
#define VL53L1_DSS_CALC__USER_ROI_SPAD_EN_31                                             0x0F7B

#define VL53L1_DSS_CALC__USER_ROI_0                                                      0x0F7C
#define VL53L1_DSS_CALC__USER_ROI_1                                                      0x0F7D

#define VL53L1_DSS_CALC__MODE_ROI_0                                                      0x0F7E
#define VL53L1_DSS_CALC__MODE_ROI_1                                                      0x0F7F

#define VL53L1_SIGMA_ESTIMATOR_CALC__SPARE_0                                             0x0F80

#define VL53L1_VHV_RESULT__PEAK_SIGNAL_RATE_MCPS                                         0x0F82
#define VL53L1_VHV_RESULT__PEAK_SIGNAL_RATE_MCPS_HI                                      0x0F82
#define VL53L1_VHV_RESULT__PEAK_SIGNAL_RATE_MCPS_LO                                      0x0F83

#define VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF                                       0x0F84
#define VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_3                                     0x0F84
#define VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_2                                     0x0F85
#define VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_1                                     0x0F86
#define VL53L1_VHV_RESULT__SIGNAL_TOTAL_EVENTS_REF_0                                     0x0F87

#define VL53L1_PHASECAL_RESULT__PHASE_OUTPUT_REF                                         0x0F88
#define VL53L1_PHASECAL_RESULT__PHASE_OUTPUT_REF_HI                                      0x0F88
#define VL53L1_PHASECAL_RESULT__PHASE_OUTPUT_REF_LO                                      0x0F89

#define VL53L1_DSS_RESULT__TOTAL_RATE_PER_SPAD                                           0x0F8A
#define VL53L1_DSS_RESULT__TOTAL_RATE_PER_SPAD_HI                                        0x0F8A
#define VL53L1_DSS_RESULT__TOTAL_RATE_PER_SPAD_LO                                        0x0F8B

#define VL53L1_DSS_RESULT__ENABLED_BLOCKS                                                0x0F8C

#define VL53L1_DSS_RESULT__NUM_REQUESTED_SPADS                                           0x0F8E
#define VL53L1_DSS_RESULT__NUM_REQUESTED_SPADS_HI                                        0x0F8E
#define VL53L1_DSS_RESULT__NUM_REQUESTED_SPADS_LO                                        0x0F8F

#define VL53L1_MM_RESULT__INNER_INTERSECTION_RATE                                        0x0F92
#define VL53L1_MM_RESULT__INNER_INTERSECTION_RATE_HI                                     0x0F92
#define VL53L1_MM_RESULT__INNER_INTERSECTION_RATE_LO                                     0x0F93

#define VL53L1_MM_RESULT__OUTER_COMPLEMENT_RATE                                          0x0F94
#define VL53L1_MM_RESULT__OUTER_COMPLEMENT_RATE_HI                                       0x0F94
#define VL53L1_MM_RESULT__OUTER_COMPLEMENT_RATE_LO                                       0x0F95

#define VL53L1_MM_RESULT__TOTAL_OFFSET                                                   0x0F96
#define VL53L1_MM_RESULT__TOTAL_OFFSET_HI                                                0x0F96
#define VL53L1_MM_RESULT__TOTAL_OFFSET_LO                                                0x0F97

#define VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS                                       0x0F98
#define VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS_3                                     0x0F98
#define VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS_2                                     0x0F99
#define VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS_1                                     0x0F9A
#define VL53L1_XTALK_CALC__XTALK_FOR_ENABLED_SPADS_0                                     0x0F9B

#define VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS                                     0x0F9C
#define VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_3                                   0x0F9C
#define VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_2                                   0x0F9D
#define VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_1                                   0x0F9E
#define VL53L1_XTALK_RESULT__AVG_XTALK_USER_ROI_KCPS_0                                   0x0F9F

#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS                                 0x0FA0
#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_3                               0x0FA0
#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_2                               0x0FA1
#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_1                               0x0FA2
#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_INNER_ROI_KCPS_0                               0x0FA3
#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS                                 0x0FA4
#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_3                               0x0FA4
#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_2                               0x0FA5
#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_1                               0x0FA6
#define VL53L1_XTALK_RESULT__AVG_XTALK_MM_OUTER_ROI_KCPS_0                               0x0FA7

#define VL53L1_RANGE_RESULT__ACCUM_PHASE                                                 0x0FA8
#define VL53L1_RANGE_RESULT__ACCUM_PHASE_3                                               0x0FA8
#define VL53L1_RANGE_RESULT__ACCUM_PHASE_2                                               0x0FA9
#define VL53L1_RANGE_RESULT__ACCUM_PHASE_1                                               0x0FAA
#define VL53L1_RANGE_RESULT__ACCUM_PHASE_0                                               0x0FAB

#define VL53L1_RANGE_RESULT__OFFSET_CORRECTED_RANGE                                      0x0FAC
#define VL53L1_RANGE_RESULT__OFFSET_CORRECTED_RANGE_HI                                   0x0FAC
#define VL53L1_RANGE_RESULT__OFFSET_CORRECTED_RANGE_LO                                   0x0FAD

#define VL53L1_SHADOW_PHASECAL_RESULT__VCSEL_START                                       0x0FAE

#define VL53L1_SHADOW_RESULT__INTERRUPT_STATUS                                           0x0FB0
#define VL53L1_SHADOW_RESULT__RANGE_STATUS                                               0x0FB1
#define VL53L1_SHADOW_RESULT__REPORT_STATUS                                              0x0FB2
#define VL53L1_SHADOW_RESULT__STREAM_COUNT                                               0x0FB3

#define VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0                             0x0FB4
#define VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_HI                          0x0FB4
#define VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0_LO                          0x0FB5

#define VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0                            0x0FB6
#define VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_HI                         0x0FB6
#define VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD0_LO                         0x0FB7

#define VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0                                0x0FB8
#define VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_HI                             0x0FB8
#define VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0_LO                             0x0FB9

#define VL53L1_SHADOW_RESULT__SIGMA_SD0                                                  0x0FBA
#define VL53L1_SHADOW_RESULT__SIGMA_SD0_HI                                               0x0FBA
#define VL53L1_SHADOW_RESULT__SIGMA_SD0_LO                                               0x0FBB

#define VL53L1_SHADOW_RESULT__PHASE_SD0                                                  0x0FBC
#define VL53L1_SHADOW_RESULT__PHASE_SD0_HI                                               0x0FBC
#define VL53L1_SHADOW_RESULT__PHASE_SD0_LO                                               0x0FBD

#define VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0                     0x0FBE
#define VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_HI                  0x0FBE
#define VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0_LO                  0x0FBF

#define VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0        0x0FC0
#define VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_HI     0x0FC0
#define VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LO     0x0FC1

#define VL53L1_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0                        0x0FC2
#define VL53L1_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                     0x0FC2
#define VL53L1_SHADOW_RESULT__MM_INNER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                     0x0FC3
#define VL53L1_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0                        0x0FC4
#define VL53L1_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_HI                     0x0FC4
#define VL53L1_SHADOW_RESULT__MM_OUTER_ACTUAL_EFFECTIVE_SPADS_SD0_LO                     0x0FC5

#define VL53L1_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0                             0x0FC6
#define VL53L1_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_HI                          0x0FC6
#define VL53L1_SHADOW_RESULT__AVG_SIGNAL_COUNT_RATE_MCPS_SD0_LO                          0x0FC7

#define VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1                             0x0FC8
#define VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_HI                          0x0FC8
#define VL53L1_SHADOW_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD1_LO                          0x0FC9

#define VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1                            0x0FCA
#define VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_HI                         0x0FCA
#define VL53L1_SHADOW_RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS_SD1_LO                         0x0FCB

#define VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1                                0x0FCC
#define VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_HI                             0x0FCC
#define VL53L1_SHADOW_RESULT__AMBIENT_COUNT_RATE_MCPS_SD1_LO                             0x0FCD

#define VL53L1_SHADOW_RESULT__SIGMA_SD1                                                  0x0FCE
#define VL53L1_SHADOW_RESULT__SIGMA_SD1_HI                                               0x0FCE
#define VL53L1_SHADOW_RESULT__SIGMA_SD1_LO                                               0x0FCF
#define VL53L1_SHADOW_RESULT__PHASE_SD1                                                  0x0FD0
#define VL53L1_SHADOW_RESULT__PHASE_SD1_HI                                               0x0FD0
#define VL53L1_SHADOW_RESULT__PHASE_SD1_LO                                               0x0FD1

#define VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1                     0x0FD2
#define VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_HI                  0x0FD2
#define VL53L1_SHADOW_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD1_LO                  0x0FD3

#define VL53L1_SHADOW_RESULT__SPARE_0_SD1                                                0x0FD4
#define VL53L1_SHADOW_RESULT__SPARE_0_SD1_HI                                             0x0FD4
#define VL53L1_SHADOW_RESULT__SPARE_0_SD1_LO                                             0x0FD5
#define VL53L1_SHADOW_RESULT__SPARE_1_SD1                                                0x0FD6
#define VL53L1_SHADOW_RESULT__SPARE_1_SD1_HI                                             0x0FD6
#define VL53L1_SHADOW_RESULT__SPARE_1_SD1_LO                                             0x0FD7
#define VL53L1_SHADOW_RESULT__SPARE_2_SD1                                                0x0FD8
#define VL53L1_SHADOW_RESULT__SPARE_2_SD1_HI                                             0x0FD8
#define VL53L1_SHADOW_RESULT__SPARE_2_SD1_LO                                             0x0FD9
#define VL53L1_SHADOW_RESULT__SPARE_3_SD1                                                0x0FDA

#define VL53L1_SHADOW_RESULT__THRESH_INFO                                                0x0FDB

#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0                             0x0FDC
#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_3                           0x0FDC
#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_2                           0x0FDD
#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_1                           0x0FDE
#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0_0                           0x0FDF

#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0                              0x0FE0
#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_3                            0x0FE0
#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_2                            0x0FE1
#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_1                            0x0FE2
#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD0_0                            0x0FE3

#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0                               0x0FE4
#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_3                             0x0FE4
#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_2                             0x0FE5
#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_1                             0x0FE6
#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD0_0                             0x0FE7

#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0                             0x0FE8
#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_3                           0x0FE8
#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_2                           0x0FE9
#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_1                           0x0FEA
#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD0_0                           0x0FEB

#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1                             0x0FEC
#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_3                           0x0FEC
#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_2                           0x0FED
#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_1                           0x0FEE
#define VL53L1_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD1_0                           0x0FEF

#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1                              0x0FF0
#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_3                            0x0FF0
#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_2                            0x0FF1
#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_1                            0x0FF2
#define VL53L1_SHADOW_RESULT_CORE__RANGING_TOTAL_EVENTS_SD1_0                            0x0FF3

#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1                               0x0FF4
#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_3                             0x0FF4
#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_2                             0x0FF5
#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_1                             0x0FF6
#define VL53L1_SHADOW_RESULT_CORE__SIGNAL_TOTAL_EVENTS_SD1_0                             0x0FF7

#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1                             0x0FF8
#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_3                           0x0FF8
#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_2                           0x0FF9
#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_1                           0x0FFA
#define VL53L1_SHADOW_RESULT_CORE__TOTAL_PERIODS_ELAPSED_SD1_0                           0x0FFB

#define VL53L1_SHADOW_RESULT_CORE__SPARE_0                                               0x0FFC

#define VL53L1_SHADOW_PHASECAL_RESULT__REFERENCE_PHASE_HI                                0x0FFE
#define VL53L1_SHADOW_PHASECAL_RESULT__REFERENCE_PHASE_LO                                0x0FFF
