/****************************************************************************
 * apps/custom_apps/cubus_app/common_functions.h
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

#ifndef __APPS_CUSTOM_APPS_COMMON_FUNCTIONS_H
#define __APPS_CUSTOM_APPS_COMMON_FUNCTIONS_H

#include <nuttx/config.h>
#include "file_operations.h"

#define FLAG_DATA_INT_ADDR    0x081C0000
#define FLAG_DATA_SECTOR_NUM 

#define MFM_MAIN_STRPATH        "/mnt/fs/mfm/mtd_mainstorage"
#define MFM_MSN_STRPATH		    "/mnt/fs/mfm/mtd_mission"

#define SFM_MAIN_STRPATH        "/mnt/fs/sfm/mtd_mainstorage"
#define SFM_MSN_STRPATH		    "/mnt/fs/sfm/mtd_mission"

#define file_name_flag             "/flags.txt"
#define file_name_sat_health       "/sat_health.txt"

#define file_name_epdm_msn          "/epdm.txt"
#define file_name_cam_msn           "/cam.txt"
#define file_name_test_msn          "/test_msn.txt"

typedef struct  {
	// uint64_t timestamp;
	// float accl_x;
	// float accl_y;
	// float accl_z;
	// float gyro_x;
	// float gyro_y;
	// float gyro_z;
	// float mag_x;
	// float mag_y;
	// float mag_z;
	int16_t temp_x;
	int16_t temp_x1;
	int16_t temp_y;
	int16_t temp_y1;
	int16_t temp_z;
	int16_t temp_z1;
	int16_t temp_bpb;
	int16_t temp_obc;
	int16_t temp_com;
	int16_t temp_batt;
	int16_t batt_volt;
	int16_t sol_p1_v;
	int16_t sol_p2_v;
	int16_t sol_p3_v;
	int16_t sol_p4_v;
	int16_t sol_p5_v;
	int16_t sol_t_v;
	int16_t raw_v;
	int16_t sol_p1_c;
	int16_t sol_p2_c;
	int16_t sol_p3_c;
	int16_t sol_p4_c;
	int16_t sol_p5_c;
	int16_t sol_t_c;
	int16_t rst_3v3_c;
	int16_t raw_c;
	int16_t v3_main_c;
	int16_t v3_com_c;
	int16_t v3_2_c;
	int16_t v5_c;
	int16_t unreg_c;
	int16_t v4_c;
	int16_t batt_c;
	int8_t rsv_cmd;

	int8_t ant_dep_stat;
	int8_t ul_state;
	int8_t oper_mode;
	int8_t msn_flag;
	int8_t rsv_flag;
	int8_t kill_switch;

    int16_t ant_temp_out;
}satellite_health_s;

typedef struct {
	uint8_t ANT_DEP_STAT;		//antenna deployment status
	uint8_t RSV_FLAG;			//reservation command flag 
	uint8_t UL_STATE;			//uplink success
	uint8_t OPER_MODE;			//operation modes
	uint8_t KILL_SWITCH_STAT;	//kill switch status
	uint8_t FILLER;				//to make sure data is stored in internal flash 
}CRITICAL_FLAGS;

typedef enum _OPERA_MODES {
	NRML_MODE = 0x5A,	//90 on decimal
	LOW_PWR_MODE = 0x6A,//
	SAFE_MODE = 0x7A,
	SAT_KILL_MODE = 0x8A,
} OPERA_MODES;

/*
 * @brief	enumeration definition for ACK STATES
 */
typedef enum _ACK_STATE {
	ACK_SUCCESS = 0xAC, ACK_FAILURE = 0xEE,
} ACK_STATE;

/*
 * @brief enumeration definition for ANTENNA deployment states
 */
typedef enum _ANT_STATE {
	UNDEPLOYED = 0x00, DEPLOY_NOW = 0XAE, DEPLOYED = 0xDE,
} ANT_STATE;

/*
 * @brief enumeration definition for Uplink status
 */
typedef enum _UL_STATE {
	UL_NOT_RX = 0x00, UL_RX = 0xAE,
} UL_STATE;

typedef enum _KILL_SWITCH_STATE {
	KILL_SW_ON = 0x01, KILL_SW_OFF = 0x00,
} KILL_SWITCH_STATE;

/*
 * @brief	enumeration definition for status of Reservation commands
 */
typedef enum _RSV_CMD_STATE {
	RSV_NOT_RUNNING = 0x00, RSV_RUNNING = 0xAE,
} RSV_CMD_STATE;

extern CRITICAL_FLAGS critic_flags;

void store_sat_health_data(satellite_health_s *sat_health_data);
void retrieve_latest_sat_health_data(satellite_health_s *sat_health_buf);
void retrieve_sat_health_data(satellite_health_s sat_health_buf[], int times);

void print_satellite_health_data(satellite_health_s *sat_health);
void print_critical_flag_data(CRITICAL_FLAGS *flags);

#endif