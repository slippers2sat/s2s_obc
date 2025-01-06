/****************************************************************************
 * apps/examples/spi_test/spi_test_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/
#ifndef __APPS_CUSTOM_APPS_CUBUS_APP_MAIN_H
#define __APPS_CUSTOM_APPS_CUBUS_APP_MAIN_H

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "adc.h"
#include "common_functions.h"
#include <nuttx/fs/fs.h>
#include "imu_mag.h"
#include "gpio_definitions.h"
#include "mission_operations.h"
// #include "com_app_main.h"
#include "file_operations.h"

#include <uORB/uORB.h>
#include <sensor/mag.h>
#include <poll.h>
#include <sensor/rgb.h>

#include "common_functions.h"



struct orb_mag_s
{
  uint64_t timestamp;
  float x;
  float y;
  float z;
  float temperature;
};

void write_to_mfm(char *path, CRITICAL_FLAGS *flag_data);
struct data
{
    void *data_struct;
    uint16_t data_size;
};

// ORB_DECLARE(mpu6500_imu_msg);
// ORB_DEFINE(mpu6500_imu_msg, struct mpu6500_imu_msg, print_combined_msg);



// Ensure this is defined before you use MISSION_STATUS in your functions
struct mission_status {
    bool ADCS_MISSION;
    bool CAM_MISSION;
    bool EPDM_MISSION;
    bool FLASH_OPERATION;
};

extern struct mission_status MISSION_STATUS;
#define ADC_MAX_LOOP 400
#define I_ADC1_DATA_LEN 15
uint16_t I_ADC1_RAW[I_ADC1_DATA_LEN][ADC_MAX_LOOP];
uint16_t I_ADC1_DATA[I_ADC1_DATA_LEN];
int32_t averageRaw[I_ADC1_DATA_LEN];
 

// struct mpu6500_imu_msg
// {
//   int16_t acc_x;
//   int16_t acc_y;
//   int16_t acc_z;
//   int16_t temp;
//   int16_t gyro_x;
//   int16_t gyro_y;
//   int16_t gyro_z;
//   int16_t mag_x;
//   int16_t mag_y;
//   int16_t mag_z;
// };


void make_satellite_health();
void RUN_HK();

void collect_imu_mag();

void read_mpu6050(int fd, struct sensor_accel *acc_data, struct sensor_gyro *gyro_data, struct mpu6500_imu_msg *raw_imu);
void read_lis3mdl(int fd_mag, struct mpu6500_imu_msg *raw_imu, int16_t mag_data[4]);


#endif  //__APPS_CUSTOM_APPS_CUBUS_APP_MAIN_H


