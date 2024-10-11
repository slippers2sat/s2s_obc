/****************************************************************************
 * apps/examples/cubus_app/cubus_app_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed withEPDMEPDM_
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
// #include "ads7953_raw_msg.h"
#include <sys/ioctl.h>
#include "cubus_app_main.h"
#include "common_functions.h"
// #include<sys/ddi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/progmem.h>
#include <fcntl.h>
#include <stdio.h>
#include <nuttx/irq.h>
#include <stdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <time.h>
#include <malloc.h>
#include <nuttx/timers/watchdog.h>
// #include "adc.h"

// #include "com_app_main.h"
#include "gpio_definitions.h"

#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/lis3mdl.h>
/****************************************************************************
 * COM TASK task
 ****************************************************************************/

/*
*****************************************
*/

// TODO uncomment below for logging
//  #define LOGGING

/*
*****************************************
*/

// Macro Definition
#define COM_DATA_SIZE 44
#define BEACON_DELAY 90
#define BEACON_DATA_SIZE 85
#define ACK_DATA_SIZE 6 + 1
#define COM_RX_CMD_SIZE 43
#define COM_DG_MSG_SIZE 44
#define NB_LOWERHALFS 1
#define SIZE_OF_DATA_DOWNLOAD 80
// #define DELAY_ADC 4000
// #define ADC_DELAY 6000
#define HK_DELAY 90
#define ANT_DEP_DELAY 30 * 60
#define BEACON_DELAY 180

/*Private variable start*/
// Declare the instance of the struct
uint8_t COM_HANDSHAKE_STATUS = 0;
bool FLASH_OPERATION = false;
struct mission_status MISSION_STATUS = {false, false, false}; // Initialize all to false

uint8_t beacon_status = 0;
uint8_t COM_BUSY = 0;
static struct work_s work_beacon;
uint8_t beacon_type = 0;
static bool g_commander_task_started = false, g_wdog_task_started = false, g_beacon_task_started = false;
uint8_t data[7] = {0x53, 0x01, 0x02, 0x03, 0x04, 0x7e};
uint8_t Msn_Start_Cmd[7] = {0x53, 0xad, 0xba, 0xcd, 0x9e, 0x7e};
// uint8_t ACK[7] = {0x53, 0xaa, 0xcc, 0xaa, 0xcc, 0x7e};
uint8_t NACK[7] = {0x53, 0xee, 0xff, 0xee, 0xff, 0x7e};

enum MCU_IDS
{
  OBC_MCU = 0x01,
  COM_MCU = 0x02,
  ADCS_MCU = 0x03,
  CAM_MCU = 0x04,
  EPDM_MCU = 0x05
};
/*
Defing work structures for work_queue thread
*/
static struct work_s work_hk;
static struct work_s work_ant_dep;

/*OBC command ended*/
uint8_t command_list[] = {};

uint8_t RX_DATA_EPDM[48] = {'\0'};
uint8_t digipeating = 1;
// int Execute_EPDM();
extern ext_adc_s ext_adc_data[EXT_ADC_MAX_CHANNELS];
extern CRITICAL_FLAGS critic_flags;

CRITICAL_FLAGS test_flags;

struct sensor_accel imu_acc_data;
struct sensor_gyro imu_gyro_data;
struct mpu6500_imu_msg raw_imu;
// struct MISSION_STATUS MISSION_STATUS;

char buffer[255] = {'\0'};

satellite_health_s sat_health = {'\0'};
satellite_health_s sat_health_rx_buf = {'\0'};

S2S_BEACON_A s2s_beacon_type_a;
S2S_BEACON_TYPE_B s2s_beacon_type_b;

/*
@brief
clear/empty or truncate the file
Two arguments are to be supplied:
  1.select_flash : determines the flash memory to be selected or from where the data is to be deleted
  2.select_file :  determines the name of the text file which is to be emptied

  Requirement: An enumeration named select_flash is to be defined with two components
*/
enum SELECT_FLASH
{
  MAIN_FLASH_MEMORY,
  SHARED_FLASH_MEMORY,
  INTERNAL_FLASH
};
/*
@brief
This enumeration is created for easy implementation to select required files.
*/
enum SELECT_FILE
{
  FLAGS,
  SATELLITE_HEALTH,
  SATELLITE_LOG,
  RESERVATION_TABLE,
  CAMERA_TXT,
  EPDM_TXT,
  ADCS_TXT
};
/*
@brief:
cmd: main command,
select_file: enum to select the required text file,
select_flash: enum to select the required partition to read the text file,
rsv_table: this contains the information how lately this command is to be executed i.e. time latency info
filepath: the full path to the textfile: MOUNT_POINT/TEXT_FILENAME
address: 4 byte of adress data here it might be the data download counter
number of packets: may be DNC(0x00,0x00) or number of packets in 2 bytes.
*/

struct FILE_OPERATIONS
{
  uint8_t cmd;
  enum SELECT_FILE select_file;
  enum SELECT_FLASH select_flash;
  uint8_t rsv_table[2];
  char filepath[200];
  uint8_t address[4];
  uint8_t number_of_packets[2];
  uint8_t mcu_id;
};

typedef struct
{
  uint8_t SATELLITE_HEALTH_1; // SAT HEALTH POINTER status
  uint8_t SATELLITE_HEALTH_2; // SAT HEALTH POINTER status

  uint8_t MSN1_DATA_1; // MSN1 DATA POINTER status
  uint8_t MSN1_DATA_2; // MSN1 DATA POINTER status

  uint8_t MSN2_DATA_1; // MSN2 DATA POINTER status
  uint8_t MSN2_DATA_2; // MSN2 DATA POINTER status

  uint8_t MSN3_DATA_1; // MSN3 DATA POINTER status
  uint8_t MSN3_DATA_2; // MSN3 DATA POINTER status
                       // to make sure data is stored in internal flash
} SEEK_POINTER;

/*Private variable end*/

/*Private function prototypes declaration start*/

void incorrect_command(uint8_t *ack);
// int gpio_write1(uint32_t pin, uint8_t mode);
int handshake_MSN(uint8_t subsystem, uint8_t *ack);
int handshake_COM(uint8_t *ack);
int receive_telecommand_rx(uint8_t *COM_RX_DATA);
static int COM_TASK(int argc, char *argv[]);
int send_data_uart(char *dev_path, uint8_t *data, uint16_t size);
int receive_data_uart(char *dev_path, uint8_t *data, uint16_t size);

void Make_Beacon_Data(uint8_t type);
void serialize_beacon_b(uint8_t beacon_data[BEACON_DATA_SIZE]);
void serialize_beacon_a(uint8_t beacon_data[BEACON_DATA_SIZE]);
void digipeater_mode(uint8_t *data);
void parse_command(uint8_t COM_RX_DATA[COM_DATA_SIZE]);
int turn_msn_on_off(uint8_t subsystem, uint8_t state);
void adcs_operation(uint8_t mode);
void epdm_operation();
void cam_operation();
void watchdog_refresh_task(int fd);

int configure_watchdog(int fd, int timeout);
void send_beacon();
/*Private function prototypes declaration end */
int receive_telecommand_rx(uint8_t *COM_RX_DATA)
{ // TODO sth to do with parsing
  uint8_t useful_command[12];
  uint8_t main_cmd[3] = {'\0'};
  uint16_t rsv_table = 0;
  uint32_t address = 0;
  uint16_t pckt_no = 0;
  uint8_t HEADER, MCU_ID;

  uint8_t ack[85] = {0x53, 0xac, 0x04, 0x01, 0x62, 0x63, 0x7e, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x70, 0x71, 0x72, 0x1e, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x80, 0x7e};
  syslog(LOG_DEBUG, "waiting for telecommands from COM\n");
  int ret;
  // ret = 1;
  // TODO remove the comment line below and comment the upper line to int ret
  ret = receive_data_uart(COM_UART, COM_RX_DATA, COM_RX_CMD_SIZE); // telecommand receive
  syslog(LOG_DEBUG, "Received ret as %d and value :%s\n", ret, COM_RX_DATA);
  if (ret < 0)
  {
    syslog(LOG_DEBUG, "data not received from COM\n NACK sending\n");
    send_data_uart(COM_UART, NACK, 7);
    syslog(LOG_DEBUG, "data not received from COM\n NACK Sent\n");
    return ret;
  }
  else
  {
    // Todo uncomment the line later
    syslog(LOG_SYSLOG, "COmmand received %s\n", COM_RX_DATA);
    parse_command(COM_RX_DATA);

    uint8_t commands[COM_DATA_SIZE] = {11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 0x01, 0x01, 0xca, 0xd1, 0xf3, 0, 0, 0, 0, 0, 0, 00, 0, 0};
    printf("-------------parse command starting-------------\n");

    // // Check whether the received data in uart_com has initial 0x00 value or not if the initial is 0x00 then MCU_ID is supposed to be there at index 16, otherwise it is in index 17
    // parse_command(commands);

    // commands[16] = 0x01;
    // commands[17] = 0x1d;
    // commands[18] = 0xd1;
    // commands[19] = 0xf2;

    // commands[20] = 0x00;
    // commands[21] = 0x00;

    // commands[22] = 0x00;
    // commands[23] = 0x00;
    // commands[24] = 0x00;
    // commands[25] = 0x05;

    // commands[26] = 0x00;
    // commands[27] = 0x00;
    // commands[28] = 0x00;

    // commands[29] = 0x05;

    // for (int i = 0; i < 9; i++)
    //   sleep(1);
    // parse_command(commands);

    // commands[16] = 0x01;
    // commands[17] = 0xca;
    // commands[18] = 0xd2;
    // commands[19] = 0xf5;
    // for (int i = 0; i < 9; i++)
    //   sleep(1);
    // parse_command(commands);
    // commands[16] = 0x02;
    // commands[17] = 0xdf;
    // commands[18] = 0xab;
    // commands[19] = 0xd1;
    // for (int i = 0; i < 9; i++)
    //   sleep(1);
    // parse_command(commands);
    // commands[16] = 0x01;
    // commands[17] = 0xee;
    // commands[18] = 0xee;
    // commands[19] = 0xee;
    // for (int i = 0; i < 9; i++)
    //   sleep(1);
    // parse_command(commands);

    // commands[16] = 0x01;
    // commands[17] = 0xee;
    // commands[18] = 0xaa;
    // commands[19] = 0xaa;
    // for (int i = 0; i < 9; i++)
    //   sleep(1);
    // parse_command(commands);

    // commands[16] = 0x01;
    // commands[17] = 0x1a;
    // commands[18] = 0xe0;
    // commands[19] = 0x1e;
    // for (int i = 0; i < 9; i++)
    //   sleep(1);
    // parse_command(commands);
    // commands[16] = 0x03;
    // commands[17] = 0x0e;
    // commands[18] = 0x53;
    // commands[19] = 0xce;
    // for (int i = 0; i < 9; i++)
    //   sleep(1);
    // parse_command(commands);
    // commands[16] = 0x04;
    // commands[17] = 0xcc;
    // commands[18] = 0x5e;
    // commands[19] = 0xbd;
    // for (int i = 0; i < 9; i++)
    //   sleep(1);
    // parse_command(commands);
    // commands[16] = 0x05;
    // commands[17] = 0xac;
    // commands[18] = 0xcf;
    // commands[19] = 0xcf;
    // parse_command(commands);

    return 0; // todo remove this part
  }
  syslog(LOG_DEBUG, "Value of digipeating is %d %d\n", digipeating, COM_RX_DATA[HEADER]);

  syslog(LOG_DEBUG, "value of ret is %d\ndata received from COM\n sending ACK\n", ret);
  sleep(3);

  return ret;
}

void incorrect_command(uint8_t *ack)
{
  ack[1] = 0xac;
  ack[2] = 0x04;
  ack[4] = 0x63;
  ack[5] = 0x62;
  sleep(1);
  send_data_uart(COM_UART, ack, sizeof(ack));
  syslog(LOG_DEBUG, "The supplied command is incorrect");
  return;
}

void parse_command(uint8_t COM_RX_DATA[COM_DATA_SIZE])
{
  uint8_t ack[85] = {0x53, 0xac, 0x04, 0x01, 0x62, 0x63, 0x7e};
  syslog(LOG_DEBUG, "command received is :");
  for (int i = 0; i < COM_DATA_SIZE; i++)
  {
    printf("%d ", COM_RX_DATA[i]);
  }
  ack[83] = 0x7e;
  ack[82] = 0x7e;
  if (COM_RX_DATA[0] == 0x53)
  { // SEND 85 BYTES BACK
    // 53 0A 2B 53 02 53 digipeating format
    if (digipeating == 1 && COM_RX_DATA[1] == 0x0a && COM_RX_DATA[2] == 0x2b && COM_RX_DATA[3] == 0x53 && COM_RX_DATA[4] == 0x02 && COM_RX_DATA[5] == 0x53)
    {
      send_data_uart(COM_UART, COM_RX_DATA, sizeof(COM_RX_DATA));
      return 33;
    }
    else if (COM_RX_DATA[0] == 0x53 & COM_RX_DATA[1] == 0xac & COM_RX_DATA[2] == 0x04)
    {
      if (COM_RX_DATA[3] == 0X02 & COM_RX_DATA[4] == 0XFC & COM_RX_DATA[5] == 0XEE)
      { // FALSE COMMAND
        // uint8_t ack1[] = {0x53, 0xac, 0x04, 0x02, 0xFC, 0xEE};
        ack[3] = 0x02;
        ack[4] = 0xfc;
        ack[5] = 0xee;

        // send_data_uart(COM_UART, ack1, sizeof(ack1));
      }
      else
      {
        if (COM_RX_DATA[4] == 0x00 & COM_RX_DATA[5] == 0xdd)
        {
          syslog(LOG_DEBUG, "\n ********************Digipeater mode turned off********************\n");
          digipeating = 0;
        }
        else if (COM_RX_DATA[4] == 0x01 & COM_RX_DATA[5] == 0xdd)
        {
          syslog(LOG_DEBUG, "\n ********************Digipeater mode turned on********************\n");
          digipeating = 1;
        }
        for (int loop1 = 0; loop1 < 43; loop1++)
        {
          ack[loop1] = COM_RX_DATA[loop1];
        }
      }
      sleep(1);
      send_data_uart(COM_UART, ack, sizeof(ack));

      return 33;
    }
  }
  else if (COM_RX_DATA[0] == 0x42)
  {
    return;
  }
  else if (COM_RX_DATA[0] == 0x72)
  {
    syslog(LOG_DEBUG, "parse command starting\n");

    for (int j = 0; j < sizeof(COM_RX_DATA); j++)
    {
      syslog(LOG_DEBUG, "%02x|%c ,", COM_RX_DATA[j], COM_RX_DATA[j]);
    }
    if (COM_RX_DATA[0] == 0X53 && COM_RX_DATA[1] == 0xac && COM_RX_DATA[2] == 0X04)
    {
      syslog(LOG_DEBUG, "-----------------------Disabled digipeater mission\n");
    }
    else
    {
      uint8_t HEADER = 0, MCU_ID = 0;
      if (COM_RX_DATA[16] >= 0x01 && COM_RX_DATA[16] <= 0x05)
      {
        HEADER = 16;
        MCU_ID = COM_RX_DATA[HEADER];
      }
      else if (COM_RX_DATA[17] >= 0x01 && COM_RX_DATA[17] <= 0x05)
      {
        HEADER = 17;
        MCU_ID = COM_RX_DATA[HEADER];
      }

      syslog(LOG_DEBUG, "**********\n*************\nHere the com rx data is %02x,%02x,%02x,%02x\n********************\n********************\n",
             HEADER, COM_RX_DATA[HEADER], COM_RX_DATA[HEADER + 1], COM_RX_DATA[HEADER + 2]);

      // uint nack

      uint8_t cmds[3];

      cmds[0] = (uint8_t)COM_RX_DATA[HEADER + 1];
      cmds[1] = (uint8_t)COM_RX_DATA[HEADER + 2];
      cmds[2] = (uint8_t)COM_RX_DATA[HEADER + 3];
      syslog(LOG_DEBUG, "**********\n*************\nHere the com rx data is %02x,%02x,%02x,%02x\n********************\n********************\n",
             HEADER, cmds[0], cmds[0 + 1], cmds[0 + 2]);

      // else if (cmds[0] == 0x53 & cmds[1] == 0xac & cmds[2] == 0x04)
      // {
      //   // gpio_write(GPIO_COM_4V_EN, 1);
      //   syslog(LOG_DEBUG, "\n ********************Digipeater mode turned off********************\n");
      //   digipeating = 0;
      //   ack[0]=0x53;
      //   ack[1] = 0xac;
      //   ack[2]=0x04;
      //   ack[3] = 0x02;
      //   ack[4] = 0x00;
      //   ack[5] =0xdd;
      //   sleep(1);
      send_data_uart(COM_UART, ack, sizeof(ack));

      //   // send_
      //   // sleep(3);
      //   // gpio_write(GPIO_COM_4V_EN, 0);

      //   return 33;
      // }
      // else if (cmds[0] == 0xdf & cmds[1] == 0xab & cmds[2] == 0xd1)
      // {
      //   syslog(LOG_DEBUG, "\n ********************Digipeater mode turned on : can receive digipeating message********************\n");
      //   digipeating = 0;
      //   send_data_uart(COM_UART, COM_RX_DATA, sizeof(COM_RX_DATA));

      //   return 33;
      // }
      syslog(LOG_DEBUG, "MCU ID %d |%02x\n", cmds[0]);
      switch (MCU_ID)
      {
      case OBC_MCU: /*
                    Command to perform flash operations by the OBC
                    */

      { // __file_operations.cmd = COM_RX_DATA[HEADER + 1];
        syslog(LOG_DEBUG, "OBC MCU ID has been received\n");
        // break; // TO remove this later
        if (cmds[0] == 0xCA | cmds[0] == 0x1D)
        {
          struct FILE_OPERATIONS __file_operations = {
              .cmd = 0x00,
              .number_of_packets = {0}, // Initialize all elements to 0
              .filepath = {'\0'},       // Initialize as an empty string
              .address = {0},           // Initialize all elements to 0
              .rsv_table = {0},         // Initialize all elements to 0
              .mcu_id = 0xda};

          __file_operations.cmd = cmds[0];
          // __file_operations.select_file = ;
          if ((COM_RX_DATA[HEADER + 2] == 0xD1) || (COM_RX_DATA[HEADER + 2] == 0xD2))
          {
            __file_operations.select_flash = MAIN_FLASH_MEMORY;
            if ((COM_RX_DATA[HEADER + 2] == 0xD2))
              strcpy(__file_operations.filepath, MFM_MSN_STRPATH);
            else
              strcpy(__file_operations.filepath, MFM_MAIN_STRPATH);
          }
          if ((COM_RX_DATA[HEADER + 2] == 0xD3) || (COM_RX_DATA[HEADER + 2] == 0xD4))
          {
            __file_operations.select_flash = SHARED_FLASH_MEMORY;
            if ((COM_RX_DATA[HEADER + 2] == 0xD3))
              strcpy(__file_operations.filepath, SFM_MAIN_STRPATH);
            else
              strcpy(__file_operations.filepath, SFM_MSN_STRPATH);
          }
          char filename[7][30] = {"/flags.txt", "/satHealth.txt", "/satellite_Logs.txt", "/reservation_table.txt", "/cam.txt", "/epdm.txt", "/adcs.txt"};

          if ((cmds[2] == 0xF1))
          {
            __file_operations.select_file = FLAGS;
            __file_operations.mcu_id = 0xad;

            strcat(__file_operations.filepath, "/flags.txt");
            syslog(LOG_DEBUG, "Selected file is %s\n", __file_operations.select_file);
          }
          else if ((cmds[2] == 0xF2))
          {
            __file_operations.mcu_id = 218;
            __file_operations.select_file = SATELLITE_HEALTH;
            strcat(__file_operations.filepath, "/satHealth.txt");
            syslog(LOG_DEBUG, "Selected file is %s\n", __file_operations.select_file);
          }
          else if ((cmds[2] == 0xF3))
          {
            __file_operations.mcu_id = 0xda;

            __file_operations.select_file = SATELLITE_LOG;
            strcat(__file_operations.filepath, "/satHealth.txt");
          }
          else if ((cmds[2] == 0xF4))
          {
            __file_operations.mcu_id = 0xad;

            __file_operations.select_file = RESERVATION_TABLE;
            strcat(__file_operations.filepath, "/reservation_table.txt");
          }
          else if ((cmds[2] == 0xF5))
          {
            strcat(__file_operations.filepath, "/cam.txt");
            __file_operations.mcu_id = 0x0c;

            __file_operations.select_file = CAMERA_TXT;
          }
          else if ((cmds[2] == 0xF6))
          {
            __file_operations.mcu_id = 0x0b;

            __file_operations.select_file = EPDM_TXT;
            strcat(__file_operations.filepath, "/epdm.txt");
          }
          else if ((cmds[2] == 0xF7))
          {
            __file_operations.select_file = ADCS_TXT;
            __file_operations.mcu_id = 0x0d;
            strcat(__file_operations.filepath, "/adcs.txt");
          }

          // TODO check reservation table here

          __file_operations.rsv_table[0] = COM_RX_DATA[HEADER + 4];
          __file_operations.rsv_table[1] = COM_RX_DATA[HEADER + 5];

          // TODO check address here
          __file_operations.address[0] = COM_RX_DATA[HEADER + 6];
          __file_operations.address[1] = COM_RX_DATA[HEADER + 7];
          __file_operations.address[2] = COM_RX_DATA[HEADER + 8];
          __file_operations.address[3] = COM_RX_DATA[HEADER + 9];

          // TODO check for number of packets here
          __file_operations.number_of_packets[0] = COM_RX_DATA[HEADER + 10];
          __file_operations.number_of_packets[1] = COM_RX_DATA[HEADER + 11];

          syslog(LOG_DEBUG, "mcu id %d, cmd : %d, select_file:%d, select_flash: %d, rsv_table:%d, filepath:%s,address :%d %d %d %d, number_of packets:%d %d\n",
                 __file_operations.mcu_id, __file_operations.cmd, __file_operations.select_flash, __file_operations.select_file, __file_operations.rsv_table[1], __file_operations.rsv_table[0], __file_operations.filepath,
                 __file_operations.address[3], __file_operations.address[2], __file_operations.address[1], __file_operations.address[0],
                 __file_operations.number_of_packets[0], __file_operations.number_of_packets[1]);
          sleep(1);
          send_data_uart(COM_UART, ack, sizeof(ack));
          perform_file_operations(&__file_operations);
        }

        /*
        Command for disabling status of KILL SWITCH
        */
        else if (cmds[0] == 0xee && cmds[1] == 0xaa && cmds[2] == 0xaa)
        {
          sleep(1);
          send_data_uart(COM_UART, ack, sizeof(ack));

          syslog(LOG_DEBUG, "--------- kill switch deactivated\n");
        }

        /*
        Command for enabling status of KILL SWITCH
        */
        else if (cmds[0] == 0xee && cmds[1] == 0xee && cmds[2] == 0xee)
        {
          sleep(1);
          send_data_uart(COM_UART, ack, sizeof(ack));

          syslog(LOG_DEBUG, "---------kill switch activated\n");
        }
      }
      /* code */
      break;

      case COM_MCU:
        // Command to ENABLE Digipeater misison
        {
          syslog(LOG_DEBUG, "COM MCU ID has been received\n");
          if (cmds[0] == 0xDF && cmds[1] == 0xAB && cmds[2] == 0xD1)
          {

            syslog(LOG_DEBUG, "-------- Digipeater mission turned ON---------\n");
          }

          // Command to DISABLE Digipeater misison
          else if (cmds[0] == 0xFD && cmds[1] == 0xBA && cmds[2] == 0xD0)
          {
            incorrect_command(ack);
            syslog(LOG_DEBUG, "-----------------------Digipeater mission turned OFF---------------\n");
          }
        }
        /* code */
        break;

      case ADCS_MCU:
        // Command to DISABLE adcs(MSN1) misison
        {
          syslog(LOG_DEBUG, "ADCS MCU ID has been received\n");
          if (cmds[1] == 0x53 && cmds[2] == 0xCE)
          {
            sleep(1);
            send_data_uart(COM_UART, ack, sizeof(ack));
            syslog(LOG_DEBUG, "------------  ADCS mission turned on (command received using radio frequency)--------------\n");
            if (cmds[0] == 0xA0)
              adcs_operation(0x01);
            else
              adcs_operation(0x02);
            syslog(LOG_DEBUG, "------------  ADCS mission turned off (command received using radio frequency) --------------\n");
          }
          else
          {
            incorrect_command(ack);
            // 53,ac,04,01,63,62,7e
            ack[2] = 0xac;
            ack[3] = 0x04;
            ack[4] = 0x01;
            ack[5] = 0x63;
            ack[6] = 0x62;
            ack[7] = 0x7e;

            sleep(1);
            send_data_uart(COM_UART, ack, sizeof(ack));
            syslog(LOG_DEBUG, "----------------Incorrect command for ADCS mission---------------\n");
          }
        }
        /* code */
        break;

      case CAM_MCU:
        // Command to ENABLE/DISABLE or run camera(MSN2) mission
        {
          syslog(LOG_DEBUG, "CAM MCU ID has been received\n");
          if (cmds[0] == 0xCC && cmds[1] == 0x5E && cmds[2] == 0xBD)
          {
            turn_msn_on_off(2, 1);
            syslog(LOG_DEBUG, "------------------------  cam mission turned on (Command received from COM using RF)------------------\n");

            sleep(1);
            send_data_uart(COM_UART, ack, sizeof(ack));
            cam_operation();
            turn_msn_on_off(2, 0);

            syslog(LOG_DEBUG, "------------------------  cam mission turned off(Command received from COM using RF)--------------------\n");
          }
          else
          {
            incorrect_command(ack);
            ack[2] = 0xac;
            ack[3] = 0x04;
            ack[4] = 0x01;
            ack[5] = 0x63;
            ack[6] = 0x62;
            ack[7] = 0x7e;
            sleep(1);
            send_data_uart(COM_UART, ack, sizeof(ack));
            syslog(LOG_DEBUG, "------------Incorrect command\n");
          }
        }
        /* code */
        break;

      case EPDM_MCU:
        // Command to ENABLE/DISABLE or run epdm(MSN3) mission
        {
          syslog(LOG_DEBUG, "EPDM MCU ID has been received\n");
          if (cmds[0] == 0xEC && cmds[1] == 0xCF && cmds[2] == 0xCF)
          {
            sleep(1);
            send_data_uart(COM_UART, ack, sizeof(ack));

            syslog(LOG_DEBUG, "----------------EPDM  turned on ------------------\n");
            epdm_operation();
            syslog(LOG_DEBUG, "----------------EPDM  turned off -----------------\n");
          }
          else
          {
            incorrect_command(ack);

            ack[2] = 0xac;
            ack[3] = 0x04;
            ack[4] = 0x01;
            ack[5] = 0x63;
            ack[6] = 0x62;
            ack[7] = 0x7e;
            sleep(1);
            send_data_uart(COM_UART, ack, sizeof(ack));
            syslog(LOG_DEBUG, "------------Incorrect command received for EPDM-----------\n");
          }
        }
        /* code */
        break;

      default:
        ack[2] = 0xac;
        ack[3] = 0x04;
        ack[4] = 0x01;
        ack[5] = 0x63;
        ack[6] = 0x62;
        ack[7] = 0x7e;
        incorrect_command(ack);

        break;
      }

      // for (int i = 0; i < BEACON_DATA_SIZE; i++)
      // {
      //     send_data_uart(COM_UART, ack[i], 1);
      //     syslog(LOG_DEBUG, "%02x ",ack[i]);
      // }
    }
  }
}

static int COM_TASK(int argc, char *argv[])
{
  int ret = -1;
  uint8_t rx_data[COM_RX_CMD_SIZE] = {'\0'};
  gpio_write(GPIO_3V3_COM_EN, 0);
  gpio_write(GPIO_3V3_COM_EN, false); // Enable COM systems

  sleep(1);
  syslog(LOG_DEBUG, "***************************Turning on COM MSN...***************************\n");
  gpio_write(GPIO_3V3_COM_EN, 1);
  gpio_write(GPIO_3V3_COM_EN, true); // Enable COM systems

  // usleep(2000000);
  sleep(5);
  ret = handshake_COM(data); // tx rx data is flushed before closing the file
  // usleep(PRINT_DELAY * 100);
  sleep(1);
  if (ret == 0)
  {
    syslog(LOG_DEBUG, "Successful handshake with COM\n");
    COM_HANDSHAKE_STATUS = 1;
  }
  if (ret != 0)
  {
    syslog(LOG_DEBUG, "Unable to handshake with COM\n");
    ret = handshake_COM(data);
    if (ret == 0)
    {
      COM_HANDSHAKE_STATUS = 1;
    }
  }
  if (COM_HANDSHAKE_STATUS == 1)
  {
    send_beacon_data();
    //  stm32_rtc_initialize();
  }

  for (;;)
  {
    if (COM_HANDSHAKE_STATUS == 1)
    {
      receive_telecommand_rx(rx_data);
    }
    usleep(1000);
  }
}

void send_beacon()
{
  for (;;)
  {
    if (COM_HANDSHAKE_STATUS == 1)
    {
      send_beacon_data();
    }
    sleep(90);
    // usleep(100000);
  }
}

/****************************************************************************
 * COM TASK task
 *
 * COM will be in digipeater mode till a digipeating message is received and digipeated
 ****************************************************************************/
void digipeater_mode(uint8_t *data) // TODO beacon 2 paxi disable garna milni enable garna namilni tara 2 agadi chai enable garna namilni
{
  receive_data_uart(COM_UART, data, 29);
  for (int i = 29; i < 84; i++)
  {
    data[i] = 0xff;
  }
  // send_data_uart(COM_UART, data, 84);
  /*To delete*/
  if (send_data_uart(COM_UART, data, 84) > 0)
  {
    syslog(LOG_DEBUG, "***************************digipeating data is ***************************\n ");
    for (int i = 0; i < 85; i++)
      printf("%02x ", data);
    printf("***************************digipeating successful***************************\n :");
  }
  /*To delete*/
}

/****************************************************************************
 * COM handshake function
 ****************************************************************************/

int handshake_COM(uint8_t *ack)

{
  double fd;
  uint8_t data1[ACK_DATA_SIZE] = {'\0'};
  int i;
  int count = 0, ret;
  printf("Opening uart dev path : %s ret : %d", COM_UART, fd);
  usleep(PRINT_DELAY);
  fd = open(COM_UART, O_RDWR);
  if (fd < 0)
  {
    printf("error opening %s\n", COM_UART);
    usleep(PRINT_DELAY);
    return -1;
  }

  int wr1 = write(fd, data, ACK_DATA_SIZE); // writing handshake data
  if (wr1 < 0)
  {
    printf("Unable to send data through %d UART", COM_UART);
    usleep(PRINT_DELAY);
    return -1;
  }
  printf("\n%d bytes written\n", wr1);
  usleep(PRINT_DELAY);
  // ret = read(fd, data1, 10);   //try reading data from UART at once as well
  for (i = 0; i < ACK_DATA_SIZE; i++)
  {
    ret = read(fd, &data1[i], 1);
  }
  printf("data received from %s \n", COM_UART);
  usleep(PRINT_DELAY);
  for (int i = 0; i < ACK_DATA_SIZE; i++)
  {
    printf(" %x ", data1[i]);
  }
  printf("\n");
  usleep(PRINT_DELAY);
  if (data[0] == data1[0] && data[ACK_DATA_SIZE - 2] == data1[ACK_DATA_SIZE - 2])
  {
    printf("\n******Acknowledgement received******\n");
    usleep(PRINT_DELAY);
  }
  printf("handshake complete\n");
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("flused tx rx buffer\n");
  if (close(fd) < 0)
  {
    close(fd);
    printf("Failed to close COM UART: %s\n", strerror(errno));
  }
  sleep(1);
  return 0;
}

/****************************************************************************
 * MSN handshake function
 ****************************************************************************/

// int handshake_MSN(uint8_t subsystem, uint8_t *ack)
// {
//   int fd;
//   char devpath[15];
//   uint8_t data1[7] = {'\0'};
//   int i;
//   int count = 0, ret;

//   switch (subsystem)
//   {
//   case 1:
//     strcpy(devpath, ADCS_UART);
//     // gpio_write(GPIO_MSN1_EN, 1);// TODO uncomment later
//     printf("Turned on power line for ADCS\n");
//     break;
//   case 2:
//     strcpy(devpath, CAM_UART);
//     // gpio_write(GPIO_MSN2_EN, 1);
//     printf("Turned on power line for CAM\n");
//     break;
//   case 3:
//     strcpy(devpath, EPDM_UART);
//     // gpio_write(GPIO_BURNER_EN, 1);
//     printf("Turned on power line for EPDM\n");
//     break;
//   default:
//     printf("Unknown MSN subsystem selected\n");
//     return -1;
//     break;
//   }

//   printf("Opening uart dev path : %s \n", devpath);
//   usleep(PRINT_DELAY);
//   fd = open(devpath, O_RDWR);
//   if (fd < 0)
//   {
//     printf("error opening %s\n", devpath);
//     usleep(PRINT_DELAY);
//     return -1;
//   }

//   int wr1;
//   // = write(fd, ack, strlen(ack)); // writing handshake data
//   for (i = 0; i <= strlen(ack); i++)
//   {
//     write(fd, &ack[i], 1);
//     // usleep(5000);
//   }
//   if (wr1 < 0)
//   {
//     printf("Unable to send data through %d UART", devpath);
//     // usleep(PRINT_DELAY);
//     return -1;
//   }
//   printf("\n%d bytes written\n", strlen(ack));
//   usleep(1000 * 3000);
//   // ret = read(fd, data1, 7);
//   for (i = 0; i < 6; i++)
//   {
//     ret = read(fd, &data1[i], 1);
//   }
//   printf("data received from %s \n", devpath);
//   usleep(PRINT_DELAY);
//   for (int i = 0; i < 7; i++)
//   {
//     printf(" %x ", data1[i]);
//   }
//   printf("\n");
//   usleep(PRINT_DELAY);
//   if (data[0] == data1[0] && data[5] == data1[5])
//   {
//     printf("\n******Acknowledgement received******\n");
//     usleep(PRINT_DELAY);
//   }
//   else
//     return 1;
//   printf("handshake complete\n");
//   usleep(PRINT_DELAY);
//   printf("\n");
//   ioctl(fd, TCFLSH, 2);
//   ioctl(fd, TCDRN, NULL);
//   printf("flused tx rx buffer\n");
//   close(fd);
//   return 0;
// }

// int gpio_write1(uint32_t pin, uint8_t mode)
// {

//   gpio_config_s gpio_numval;
//   int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
//     close(fd);
//     return -1;
//   }
//   gpio_numval.gpio_num = pin;
//   gpio_numval.gpio_val = mode;
//   if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
//   {
//     syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
//     return -2;
//   }
//   int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
//   close(fd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Unable to write to gpio pin...\n");
//   }
//   return ret;
// }
int handshake_MSN_ADCS(uint8_t subsystem, uint8_t *ack)
{
  int fd;
  char devpath[15];
  uint8_t data1[7] = {'\0'};
  int i;
  ssize_t wr1, ret;
  uint8_t counter = 0;

  switch (subsystem)
  {
  case 1:
    strcpy(devpath, ADCS_UART);
    printf("Turned on power line for ADCS\n");
    break;
  case 2:
    strcpy(devpath, CAM_UART);
    printf("Turned on power line for CAM\n");
    break;
  case 3:
    strcpy(devpath, EPDM_UART);
    printf("Turned on power line for EPDM\n");
    break;
  default:
    printf("Unknown MSN subsystem selected\n");
    return -1;
  }

  printf("Opening UART dev path: %s\n", devpath);
  usleep(PRINT_DELAY);
  fd = open(devpath, O_WRONLY); // Open in non-blocking mode
  if (fd < 0)
  {
    printf("Error opening %s\n", devpath);
    usleep(PRINT_DELAY);
    return -1;
  }

  // Writing handshake data
  ret = write(fd, ack, 7);
  close(fd);
  // sleep(1);
  fd = open(devpath, O_RDWR | O_NONBLOCK); // Open in non-blocking mode

  printf("6 bytes written\n");
  // usleep(3000 * 1000); // 3 seconds delay

  // Reading data from UART
  // for (i = 0; i < 6; i++)
  // {
  //   ret = read(fd, &data1[i], 1);
  //   if (ret < 0)
  //   {
  //     if (errno == EAGAIN)
  //     {
  //       // printf("No data available for reading\n");
  //       // usleep(500000);
  //       usleep(500);
  //       i--; // Retry reading the same index
  //       continue;
  //     }
  //     printf("Error reading from %s\n", devpath);
  //     close(fd);
  //     return -1;
  //   }
  //   else{
  //     break;
  //   }
  // }
  // for (i = 0; i < 6; i++)
  int loop = 0;
  close(fd);
  fd = open(ADCS_UART, O_RDONLY);
  ret = read(fd, data1, 7);

  printf("Data received from %s:\n", devpath);
  usleep(PRINT_DELAY);
  for (int j = 0; j < 7; j++)
  {
    printf(" %x ", data1[j]);
  }
  printf("\n");
  usleep(PRINT_DELAY);

  if (data1[0] == ack[0] && data1[5] == ack[5])
  {
    printf("\n******Acknowledgement received******\n");
    usleep(PRINT_DELAY);
  }
  else
  {
    if (close(fd) < 0)
    {
      if (close(fd) < 0)
        printf("Failed to close COM uART: %s\n", strerror(errno));
    }
    return 1;
  }

  printf("Handshake complete\n");
  usleep(PRINT_DELAY);
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("Flushed TX/RX buffer\n");
  if (close(fd) < 0)
  {
    printf("Failed to close COM UART %s\n", strerror(errno));
  }
  return 0;
}

int handshake_MSN(uint8_t subsystem, uint8_t *ack)
{
  int fd;
  char devpath[15];
  uint8_t data1[7] = {'\0'};
  int i;
  ssize_t wr1, ret;
  uint8_t counter = 0;

  switch (subsystem)
  {
  case 1:
    strcpy(devpath, ADCS_UART);
    printf("Turned on power line for ADCS\n");
    break;
  case 2:
    strcpy(devpath, CAM_UART);
    printf("Turned on power line for CAM\n");
    break;
  case 3:
    strcpy(devpath, EPDM_UART);
    printf("Turned on power line for EPDM\n");
    break;
  default:
    printf("Unknown MSN subsystem selected\n");
    return -1;
  }

  printf("Opening UART dev path: %s\n", devpath);
  usleep(PRINT_DELAY);
  fd = open(devpath, O_WRONLY); // Open in non-blocking mode
  if (fd < 0)
  {
    printf("Error opening %s\n", devpath);
    usleep(PRINT_DELAY);
    return -1;
  }

  // Writing handshake data
  ret = write(fd, ack, 7);
  close(fd);
  // sleep(1);
  fd = open(devpath, O_RDONLY | O_NONBLOCK); // Open in non-blocking mode

  printf("6 bytes written\n");
  // usleep(3000 * 1000); // 3 seconds delay

  // Reading data from UART
  // for (i = 0; i < 6; i++)
  // {
  //   ret = read(fd, &data1[i], 1);
  //   if (ret < 0)
  //   {
  //     if (errno == EAGAIN)
  //     {
  //       // printf("No data available for reading\n");
  //       // usleep(500000);
  //       usleep(500);
  //       i--; // Retry reading the same index
  //       continue;
  //     }
  //     printf("Error reading from %s\n", devpath);
  //     close(fd);
  //     return -1;
  //   }
  //   else{
  //     break;
  //   }
  // }
  // for (i = 0; i < 6; i++)
  int loop = 0;
  while (1)
  {
    ret = read(fd, &data1[i], 1);
    if (ret < 0)
    {
      if (errno == EAGAIN)
      {
        // Uncomment for debugging
        // printf("No data available for reading\n");
        usleep(50); // usleep(5000); // Increase the sleep time to 500 ms
        i--;        // Retry reading the same index
        continue;
      }
      printf("Error reading from %s\n", devpath);
      close(fd);
      return -1;
    }
    loop++;
    // If we successfully read a character, check if we have more data
    if (ret == 1)
    {
      // Optionally print the character read for debugging
      printf("Read character: %c\n", data1[i]);
      break; // Exit the loop if a character was successfully read
    }
  }

  printf("Data received from %s:\n", devpath);
  usleep(PRINT_DELAY);
  for (int j = 0; j < 7; j++)
  {
    printf(" %x ", data1[j]);
  }
  printf("\n");
  usleep(PRINT_DELAY);

  if (data1[0] == ack[0] && data1[5] == ack[5])
  {
    printf("\n******Acknowledgement received******\n");
    usleep(PRINT_DELAY);
  }
  else
  {
    close(fd);
    return 1;
  }

  printf("Handshake complete\n");
  usleep(PRINT_DELAY);
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("Flushed TX/RX buffer\n");
  close(fd);
  return 0;
}

// static int COM_TASK(int argc, char *argv[]);

/*
list of commands for different task of OBC
*/

int gpio_write(uint32_t pin, uint8_t mode)
{

  gpio_config_s gpio_numval;
  int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
    close(fd);
    return -1;
  }
  gpio_numval.gpio_num = pin;
  gpio_numval.gpio_val = mode;
  if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
  {
    syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
    return -2;
  }
  int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
  close(fd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Unable to write to gpio pin...\n");
  }
  return ret;
}

/****************************************************************************
//  * Receive data from UART
//  ****************************************************************************/
int receive_data_uart(char *dev_path, uint8_t *data, uint16_t size)
{
  int fd, ret;
  fd = open(dev_path, O_RDONLY);
  if (fd < 0)
  {
    printf("Unable to open %s\n", dev_path);
    return fd;
  }

  printf("size of data to receive: %d\n", size);
  // ret = read(fd, data, sizeof(data)); //try receiving data without byte by byte method
  for (int i = 0; i < size; i++)
  {
    ret = read(fd, &data[i], 1);
  }
  if (ret < 0)
  {
    printf("data Not received from %s\n", dev_path);
    return ret;
  }
  printf("Data received from %s\n", dev_path);
  for (int i = 0; i < size; i++)
  {
    printf("%x ", data[i]);
  }
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("drained and flushed tx rx buffer\n");
  if (close(fd) < 0)
  {
    if (close(fd) < 0)
    {
      printf("Failed to close COM UART: %s\n", strerror(errno));
    }
  }
  return ret;
}
/****************************************************************************
 * COM RX telecommands
 *
 * COM works this way:
 * telecommand receive
 * ack send
 * execute command
 *
 * Useful commands (12 bytes)
 *  0:      MCU ID
 *  1-3:    main command
 *  4-5:    reservation table commands
 *  6-9:    address data (if data is being downloaded)
 *  10-11:  no of packets (if data is being downloaded)
 ****************************************************************************/

void serialize_beacon_a(uint8_t beacon_data[BEACON_DATA_SIZE])
{
  for (int i = 0; i <= BEACON_DATA_SIZE; i++)
  {
    beacon_data[83] = 0x00;
    // }
    // beacon_data[84] = s2s_beacon_type_a.;
    // beacon_data[85] = s2s_beacon_type_a.;
  }
  // uint8_t beacon_data[BEACON_DATA_SIZE];
  beacon_data[0] = s2s_beacon_type_a.HEAD;
  beacon_data[1] = s2s_beacon_type_a.TYPE;
  beacon_data[2] = s2s_beacon_type_a.TIM_DAY;
  beacon_data[3] = s2s_beacon_type_a.TIM_HOUR;

  beacon_data[4] = (s2s_beacon_type_a.BAT_V >> 8) & 0Xff;
  beacon_data[5] = s2s_beacon_type_a.BAT_V & 0xff;
  beacon_data[6] = (s2s_beacon_type_a.BAT_C >> 8) & 0Xff;
  beacon_data[7] = (s2s_beacon_type_a.BAT_C) & 0Xff;
  beacon_data[8] = (s2s_beacon_type_a.BAT_T >> 8) & 0Xff;
  beacon_data[9] = (s2s_beacon_type_a.BAT_T) & 0Xff;

  beacon_data[10] = s2s_beacon_type_a.RAW_C;
  beacon_data[11] = (s2s_beacon_type_a.SOL_TOT_V >> 8) & 0Xff;
  beacon_data[12] = (s2s_beacon_type_a.SOL_TOT_V) & 0Xff;
  beacon_data[13] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[14] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[15] = s2s_beacon_type_a.ANT_P_T;
  beacon_data[16] = s2s_beacon_type_a.BPB_T;
  beacon_data[17] = s2s_beacon_type_a.OBC_T;
  beacon_data[18] = s2s_beacon_type_a.X_T;
  beacon_data[19] = s2s_beacon_type_a.X1_T;
  beacon_data[20] = s2s_beacon_type_a.Y_T;
  beacon_data[21] = s2s_beacon_type_a.Y1_T;
  beacon_data[22] = 0; // s2s_beacon_type_a.SOL_P5_T;

  beacon_data[23] = s2s_beacon_type_a.SOL_P1_STAT << 7 & s2s_beacon_type_a.SOL_P2_STAT << 6 & s2s_beacon_type_a.SOL_P3_STAT << 5 & s2s_beacon_type_a.SOL_P4_STAT << 4 & s2s_beacon_type_a.MSN1_STAT << 3 & s2s_beacon_type_a.MSN2_STAT << 2 & s2s_beacon_type_a.MSN3_STAT << 1 & 0xff;
  beacon_data[24] = s2s_beacon_type_a.ANT_STAT << 4 & s2s_beacon_type_a.UL_STAT << 4;
  beacon_data[25] = s2s_beacon_type_a.OPER_MODE;
  beacon_data[26] = (s2s_beacon_type_a.OBC_RESET_COUNT >> 8) & 0xff;
  beacon_data[27] = s2s_beacon_type_a.OBC_RESET_COUNT & 0xff;
  beacon_data[28] = s2s_beacon_type_a.RST_RESET_COUNT >> 8 & 0xff; // TODO no reset mcu so no count needed
  beacon_data[29] = s2s_beacon_type_a.RST_RESET_COUNT & 0xff;
  // beacon_data[30] = s2s_beacon_type_a.LAST_RESET;
  beacon_data[30] = s2s_beacon_type_a.CHK_CRC;
}
// COM_APP

void serialize_beacon_b(uint8_t beacon_data[BEACON_DATA_SIZE])
{
  for (int i = 0; i <= BEACON_DATA_SIZE; i++)
  {
    beacon_data[83] = 0x00;
  }
  // uint8_t beacon_data[BEACON_DATA_SIZE];
  beacon_data[0] = s2s_beacon_type_b.HEAD;
  beacon_data[1] = s2s_beacon_type_b.TYPE;
  beacon_data[2] = s2s_beacon_type_b.TIM_DAY;

  beacon_data[3] = s2s_beacon_type_b.SOL_P1_V;
  beacon_data[4] = s2s_beacon_type_b.SOL_P2_V;
  beacon_data[5] = s2s_beacon_type_b.SOL_P3_V;
  beacon_data[6] = s2s_beacon_type_b.SOL_P4_V;
  beacon_data[7] = 0x00; // panel 5

  beacon_data[8] = s2s_beacon_type_b.SOL_P1_C;
  beacon_data[9] = s2s_beacon_type_b.SOL_P2_C;
  beacon_data[10] = s2s_beacon_type_b.SOL_P3_C;
  beacon_data[11] = s2s_beacon_type_b.SOL_P4_C;
  beacon_data[12] = 0x00;

  beacon_data[13] = ((s2s_beacon_type_b.GYRO_X >> 8) & 0xff) * 100;
  beacon_data[14] = ((s2s_beacon_type_b.GYRO_X) & 0xff) * 100;
  beacon_data[15] = ((s2s_beacon_type_b.GYRO_Y >> 8) & 0xff) * 100;
  beacon_data[16] = ((s2s_beacon_type_b.GYRO_Y) & 0xff) * 100;
  beacon_data[17] = (s2s_beacon_type_b.GYRO_Z >> 8 & 0xff) * 100;
  beacon_data[18] = (s2s_beacon_type_b.GYRO_Z & 0xff) * 100;

  beacon_data[19] = ((s2s_beacon_type_b.ACCL_X >> 8) & 0xff) * 100;
  beacon_data[20] = ((s2s_beacon_type_b.ACCL_X) & 0xff) * 100;
  beacon_data[21] = ((s2s_beacon_type_b.ACCL_Y >> 8) & 0xff) * 100;
  beacon_data[22] = ((s2s_beacon_type_b.ACCL_Y) & 0xff) * 100;
  beacon_data[23] = (s2s_beacon_type_b.ACCL_Z >> 8 & 0xff) * 100;
  beacon_data[24] = (s2s_beacon_type_b.ACCL_Z & 0xff) * 100;

  beacon_data[25] = ((s2s_beacon_type_b.MAG_X >> 8) & 0xff) * 100;
  beacon_data[26] = ((s2s_beacon_type_b.MAG_X) & 0xff) * 100;
  beacon_data[27] = ((s2s_beacon_type_b.MAG_Y >> 8) & 0xff) * 100;
  (beacon_data[28] = (s2s_beacon_type_b.MAG_Y) & 0xff) * 100;
  beacon_data[29] = (s2s_beacon_type_b.MAG_Z >> 8 & 0xff) * 100;
  beacon_data[30] = (s2s_beacon_type_b.MAG_Z & 0xff) * 100;
  beacon_data[31] = s2s_beacon_type_b.CHK_CRC; // TODO::  last rst
}

// COM_APP
void Make_Beacon_Data(uint8_t type)
{
  switch (type)
  {
  case 1:
    s2s_beacon_type_a.HEAD = 0x53;
    s2s_beacon_type_a.TYPE = 0;
    s2s_beacon_type_a.TIM_DAY = 01;
    s2s_beacon_type_a.TIM_HOUR = 01;
    s2s_beacon_type_a.BAT_V = sat_health.batt_volt;
    s2s_beacon_type_a.BAT_C = sat_health.batt_c;
    s2s_beacon_type_a.BAT_T = sat_health.temp_batt;

    s2s_beacon_type_a.RAW_C = sat_health.raw_c;
    s2s_beacon_type_a.SOL_TOT_V = sat_health.sol_t_v;
    s2s_beacon_type_a.SOL_TOT_C = sat_health.sol_t_c;

    s2s_beacon_type_a.BPB_T = sat_health.temp_bpb;
    s2s_beacon_type_a.OBC_T = sat_health.temp_obc;
    s2s_beacon_type_a.Y1_T = sat_health.temp_y1;
    s2s_beacon_type_a.Y_T = sat_health.temp_y;
    s2s_beacon_type_a.Z1_T = sat_health.temp_z1;
    s2s_beacon_type_a.Z_T = sat_health.temp_z;
    s2s_beacon_type_a.X1_T = sat_health.temp_x1;
    s2s_beacon_type_a.X_T = sat_health.temp_x;

    s2s_beacon_type_a.SOL_P1_STAT = sat_health.sol_p1_v >= 2 ? 1 : 0; // check from power, max power draw is 0.6 watt
    s2s_beacon_type_a.SOL_P2_STAT = sat_health.sol_p2_v >= 2 ? 1 : 0;
    s2s_beacon_type_a.SOL_P3_STAT = sat_health.sol_p3_v >= 2 ? 1 : 0;
    s2s_beacon_type_a.SOL_P4_STAT = sat_health.sol_p4_v >= 2 ? 1 : 0;

    s2s_beacon_type_a.ANT_STAT = critic_flags.ANT_DEP_STAT;
    s2s_beacon_type_a.KILL1_STAT = critic_flags.KILL_SWITCH_STAT;
    s2s_beacon_type_a.KILL2_STAT = critic_flags.KILL_SWITCH_STAT;
    s2s_beacon_type_a.UL_STAT = critic_flags.UL_STATE;

    // s2s_beacon_type_a.OBC_RESET_COUNT = ;  //TODO
    // s2s_beacon_type_a.LAST_RESET = ;       //TODO
    // s2s_beacon_type_a.CHK_CRC = ;          //TODO

  case 2:
    s2s_beacon_type_b.HEAD = 0x53;
    s2s_beacon_type_b.TYPE = 1;
    s2s_beacon_type_b.TIM_DAY = 01;

    s2s_beacon_type_b.SOL_P1_V = sat_health.sol_p1_v;
    s2s_beacon_type_b.SOL_P2_V = sat_health.sol_p2_v;
    s2s_beacon_type_b.SOL_P3_V = sat_health.sol_p3_v;
    s2s_beacon_type_b.SOL_P4_V = sat_health.sol_p4_v;

    s2s_beacon_type_b.SOL_P1_C = sat_health.sol_p1_c;
    s2s_beacon_type_b.SOL_P2_C = sat_health.sol_p2_c;
    s2s_beacon_type_b.SOL_P3_C = sat_health.sol_p3_c;
    s2s_beacon_type_b.SOL_P4_C = sat_health.sol_p4_c;

    s2s_beacon_type_b.MAG_X = (int16_t)sat_health.mag_x;
    s2s_beacon_type_b.MAG_Y = (int16_t)sat_health.mag_y;
    s2s_beacon_type_b.MAG_Z = (int16_t)sat_health.mag_z;

    s2s_beacon_type_b.GYRO_X = (int16_t)sat_health.gyro_x;
    s2s_beacon_type_b.GYRO_Y = (int16_t)sat_health.gyro_y;
    s2s_beacon_type_b.GYRO_Z = (int16_t)sat_health.gyro_z;

    s2s_beacon_type_b.ACCL_X = (int16_t)sat_health.accl_x;
    s2s_beacon_type_b.ACCL_Y = (int16_t)sat_health.accl_y;
    s2s_beacon_type_b.ACCL_Z = (int16_t)sat_health.accl_z;

    s2s_beacon_type_b.MAG_X = (int16_t)sat_health.mag_x;
    s2s_beacon_type_b.MAG_Y = (int16_t)sat_health.mag_y;
    s2s_beacon_type_b.MAG_Z = (int16_t)sat_health.mag_z;

    // s2s_beacon_type_b.CHK_CRC = ;  //TODO
  }
}

/****************************************************************************
 * Send data from UART through any UART path
 ****************************************************************************/
int send_data_uart(char *dev_path, uint8_t *data, uint16_t size)
{
  double fd;
  int i;
  int count = 0, ret;
  printf("Turning on  4V dcdc line..\n");
  gpio_write(GPIO_DCDC_4V_EN, 1);
  printf("Turning on COM 4V line..\n");
  gpio_write(GPIO_COM_4V_EN, 1);
  printf("Opening uart dev path : %s\n", dev_path);
  fd = open(dev_path, O_WRONLY);

  if (fd < 0)
  {
    printf("error opening %s\n", dev_path);
    return fd;
  }
  int wr1 = write(fd, data, size);
  if (wr1 < 0)
  {
    printf("Unable to write data\n");
    return wr1;
  }
  printf("\n%d bytes written\n", wr1);
  // printf("\ndata is %\n", wr1);
  for (int i = 0; i < size; i++)
  {
    {
      printf("%d ", data[i]);
    }
  }
  sleep(2);
  printf("Turning off  4v DCDC line..\n");
  int x = 0;
  while (x < 200000)
  {
    x += 200;
    usleep(200);
  }

  gpio_write(GPIO_DCDC_4V_EN, 0);
  // printf("Turning off COM 4V line..\n");
  gpio_write(GPIO_COM_4V_EN, 0);
  ioctl(fd, TCFLSH, 2);
  printf("flused tx rx buffer\n");
  ioctl(fd, TCDRN, NULL);
  printf("drained tx rx buffer\n");
  if (close(fd) < 0)
  {
    close(fd);
    printf("Failed to close COM_UART: %s\n", strerror(errno));
  }
  return wr1;
}

/*
@breif:
The function was created to perform delete operation of text file in littlefs, although the function seems to be unapplicable.
TODO :delete this later
*/
void delete_text_file(enum SELECT_FLASH select_flash, enum SELECT_FILE select_file)
{
  switch (select_flash)
  {
  case MAIN_FLASH_MEMORY:

    break;
  case SHARED_FLASH_MEMORY:
    break;
  }
}
/*
@brief :
The function performs truncate operation
*/
void truncate_text_file(struct FILE_OPERATIONS *file_operations)
{
  struct file truncate_ptr;
  int fd;
  fd = file_open(&truncate_ptr, file_operations->filepath, O_TRUNC);
  if (fd >= 0)
  {
    syslog(LOG_SYSLOG, "File named %s has been truncated successfully.\n", file_operations->filepath);
  }
  else
  {
    syslog(LOG_SYSLOG, "Error opening file: %s\n", file_operations->filepath);
  }
  file_close(&truncate_ptr);
  if (close(fd) < 0)
  {
    close(fd);
    printf("Failed to close COM UART: %s\n", strerror(errno));
  }
  return 0;
}

// Storage_manager app
void track_read_seek_pointer(struct FILE_OPERATIONS *file_pointer, int8_t seek_pointer[16])
{
  printf("track read seek pointer called \n");
  // SEEK_POINTER seek_pointer;
  int fd, index = 0;
  struct file fptr;
  // char file_name[200];
  // switch(file_pointer->)
  uint32_t address = file_pointer->address[3] << 24 | file_pointer->address[2] << 16 | file_pointer->address[1] << 8 | file_pointer->address[0] & 0xff;
  // if (address != 0)
  fd = open_file_flash(&fptr, "", file_pointer->filepath, O_RDWR);
  // ssize_t readBytes = file_read(&fptr, seek_pointer, sizeof(seek_pointer));
  // if (readBytes > 0)
  {
    if (file_pointer->filepath == "/mnt/fs/mfm/mtd_mainstorage/satHealth.txt")
      index = 0;
    else if (file_pointer->filepath == "/mnt/fs/mfm/mtd_mission/cam.txt")
      index = 4;
    else if (file_pointer->filepath == "/mnt/fs/mfm/mtd_mission/epdm.txt")
      index = 4 * 2;
    else if (file_pointer->filepath == "/mnt/fs/mfm/mtd_mission/adcs.txt")
      index = 3 * 4;
    else
      syslog(LOG_SYSLOG, "Some error while updating flags data\n");
  }
  seek_pointer[index] = file_pointer->address[0];
  seek_pointer[index + 1] = file_pointer->address[1];
  seek_pointer[index + 2] = file_pointer->address[2];
  seek_pointer[index + 3] = file_pointer->address[3];
  ssize_t writeBytes = file_write(&fptr, seek_pointer, sizeof(seek_pointer));
  if (writeBytes > 0)
  {
    syslog(LOG_SYSLOG, "Upadated seek pointer data saved to mfm\n");
    file_close(&fptr);
    fd = file_open(&fptr, file_pointer->filepath, O_CREAT | O_WRONLY);
    if (fd > 0)
    {
      writeBytes = file_write(&fptr, seek_pointer, sizeof(seek_pointer));
      if (writeBytes > 0)
      {
        syslog(LOG_SYSLOG, "updated seek pointer data to sfm\n");
      }
    }
  }
  file_close(&fptr);
}
// STORAGE manager app
void download_file_from_flash(struct FILE_OPERATIONS *file_operations, uint8_t *data_retrieved, uint8_t size_of_buffer)
{
  // printf("Sizzeof data-retrieved is %d %d\n", sizeof(data_retrieved1), strlen(data_retrieved1));
  struct file file_ptr;
  int fd = file_open(&file_ptr, file_operations->filepath, O_RDONLY);
  if (fd < 0)
  {
    syslog(LOG_SYSLOG, "File named %s reading mode failed fd:%d\n", file_operations->filepath, fd);
    return;
  }
  else
  {
    int8_t seek_pointer[16];
    uint8_t flash_data[BEACON_DATA_SIZE];
    uint32_t address = file_operations->address[0] << 24 | file_operations->address[1] << 16 | file_operations->address[2] << 8 | file_operations->address[3] & 0xff;
    uint16_t number_of_packets = file_operations->number_of_packets[0] << 8 | file_operations->number_of_packets[1] & 0xff;
    ssize_t read_bytes;
    int size_of_file = file_seek(&file_ptr, 0, SEEK_END);
    printf("---------------------------------------\n");
    printf("---------------------------------------\n");
    printf("Address is %d\n", address);
    printf("Number of packets is %d\n", number_of_packets);
    printf("TEXT file is %d", file_operations->select_file);
    // number_of_packets =0;

    printf("---------------------------------------\n");
    printf("---------------------------------------\n");

    int off; //= file_seek(&file_ptr, address, SEEK_SET);
    printf("\nSize of file is %d %d\n", size_of_file, address);
    uint8_t update_address = 0;
    uint32_t readBytes2;
    if (address == 0)
    {
      update_address = 1;
      // todo read the address from the text file
      struct file fl1;
      int fd2 = file_open(&fl1, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_CREAT | O_WRONLY);
      if (fd2 >= 0)
      {
        readBytes2 = file_read(&fl1, seek_pointer, sizeof(seek_pointer));
        if (readBytes2 < 0)
        {
          syslog(LOG_SYSLOG, "Error while reading the seek_pointer.txt in mfm\n");
          file_close(&fl1);
          readBytes2 = 0;
        }
      }
    }

    // uint8_t data_retrieved[412];
    // data_retrieved1 = data_retrieved;
    // (file_operations->address[3] << 24) |
    //                    (file_operations->address[2] << 16) |
    //                    (file_operations->address[1] << 8)  |
    //                    (file_operations->address[0] & 0xFF);
    int loop1 = 0;
    // for (int loop1 = 1; loop1 < number_of_packets; loop1++)

    do
    {
      if (size_of_file > 0 & size_of_file > address + 80)
      {
        off = file_seek(&file_ptr, address, SEEK_SET); // Set file pointer to the calculated address
        read_bytes = file_read(&file_ptr, data_retrieved, size_of_buffer);
        if (read_bytes >= 0)
        {
          syslog(LOG_SYSLOG, "Data retrieved from the flash address %d\n", address);

          printf("\n--------------------**************Read size = %zd\n", read_bytes);
          printf("\n\n--------------------------Data read from flash pkt no :%d  ----\n", loop1 + 1);
          for (int j = 0; j < size_of_buffer; j++)
          {
            flash_data[j + 3] = data_retrieved[j];
            printf("%02x|%c ", data_retrieved[j], data_retrieved[j]); // Print in hexadecimal format
          }
          loop1 += 1;
          flash_data[0] = 0x53;
          flash_data[1] = 0xac; // file_operations->mcu_id;//TODO check out mcu_id value and set this one
          flash_data[2] = 0x51;
          flash_data[3] = loop1;
          flash_data[BEACON_DATA_SIZE - 2] = 0x7e;
          flash_data[BEACON_DATA_SIZE - 1] = '\0';

          // printf("data sent is %s \n", flash_data);
          send_flash_data(flash_data);
          if (number_of_packets > 0)
            number_of_packets -= 1;
          address += size_of_buffer;
        }
        else
        {
          syslog(LOG_SYSLOG, "Failed to read data from the flash address %d\n", address);
          file_close(&file_ptr);

          break;
        }
      }
      else
      {
        break;
      }
      // TODO add else to perform the number of dat
    } while (number_of_packets > 0); // loop1 < number_of_packets |
    // todo : add seekpointer read index in internal and external flash memories
    file_close(&file_ptr);

    if (update_address == 1)
    {
      file_operations->address[0] = (uint8_t)address >> 24 & 0xff;
      file_operations->address[1] = (uint8_t)address >> 16 & 0xff;
      file_operations->address[2] = (uint8_t)address >> 8 & 0xff;
      file_operations->address[3] = (uint8_t)address & 0xff;
      track_read_seek_pointer(file_operations, seek_pointer);
    }
  }
  file_close(&file_ptr);
  // data_retrieved[read_bytes]= '\0';
  // printf("\n\n--------------------------Data received----\n");
  // for (int j = 0; j < sizeof(data_retrieved); j++)
  // {
  //   printf("%02x|%c ", data_retrieved[j],data_retrieved1[j]); // Print in hexadecimal format
  // }
  // printf("\n--------------------**************Size = %zu\n", sizeof(data_retrieved));
}

/*
@brief:
This function is used to perform file operation and is called only if the command has the same number as that of OBC_MCU
Two cases:
  1.if the command is 0xca then perform truncate operation
  2.if the main command is 0x1d then perform download operation i.e. read from flash
@params
file_operations: the file_operations pointer variable holds the data required to perform file operations

@struct parameters:
cmd: main command,
select_file: enum to select the required text file,
select_flash: enum to select the required partition to read the text file,
rsv_table: this contains the information how lately this command is to be executed i.e. time latency info
filepath: the full path to the textfile: MOUNT_POINT/TEXT_FILENAME
address: 4 byte of adress data here it might be the data download counter
number of packets: may be DNC(0x00,0x00) or number of packets in 2 bytes.
*/
void perform_file_operations(struct FILE_OPERATIONS *file_operations)
{
  int data_retrieved[122] = {'\0'};
  printf("*********************perform file operations has been called******\n");

  switch (file_operations->cmd)
  {
  case 0xca:
    truncate_text_file(file_operations);
    printf("-----Trucate text file called \n");
    break;
  case 0x1d:
    FLASH_OPERATION = true;
    MISSION_STATUS.FLASH_OPERATION = true;
    download_file_from_flash(file_operations, data_retrieved, SIZE_OF_DATA_DOWNLOAD);
    printf("*****Download command received**************\nsize:%d\n***********************\ncmd : %d, select_file:%d, select_flash: %d, rsv_table:%d, filepath:%s,address :%d %d %d %d, number_of packets:%d %d\n",
           sizeof(data_retrieved), file_operations->cmd, file_operations->select_flash, file_operations->select_file, file_operations->rsv_table[1], file_operations->rsv_table[0], file_operations->filepath,
           file_operations->address[3], file_operations->address[2], file_operations->address[1], file_operations->address[0],
           file_operations->number_of_packets[3], file_operations->number_of_packets[2], file_operations->number_of_packets[1], file_operations->number_of_packets[0]);

    printf("-------Data download function has been called\n");
    MISSION_STATUS.FLASH_OPERATION = false;
    FLASH_OPERATION = false;
    break;
  default:
    break;
  }
  printf("\n**************----\nhey filepath is %s\n***********\n", file_operations->filepath);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define REG_LOW_MASK 0xFF00
#define REG_HIGH_MASK 0x00FF
#define MPU6050_FS_SEL 32.8f
#define MPU6050_AFS_SEL 4096.0f
#define DEVNAME_SIZE 32

// mpu6500_imu_msg
void Antenna_Deployment(int argc, char *argv[]);

/*
Declaring structure necessary for collecting HK data
*/

// WDG_TASK

// void WDG_TASK(){
//   bool state = true;
//   uint32_t counter=0;
//   for(;;){
//     counter++;
//     gpio_write(GPIO_WD_WDI, state);
//     state = !state;
//     if(counter >= 1000){
//       while(1){

//       }
//     }
//     usleep(50);
//   }
// }
void global_reset()
{
  for (;;)
  {
    sleep(86400);
    gpio_write(GPIO_GBL_RST, true);
  }
}

/****************************************************************************
 * Name: main
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  int hand = 5;
  printf("************************************************\n");
  printf("***********S2S commander app************\n");

  printf("********ANtenna deployement starting************\n");

  // Antenna_Deployment(argc, argv);

  // watchdog code
  //  {
  //     if (g_wdog_task_started)
  //    {
  //      printf("[Watchdog TASK] Task already started.\n");
  //      return EXIT_SUCCESS;
  //    }
  //    else
  //    {
  //      int retval = task_create("WATCHDOG_TASK", 50, 4096, WDG_TASK, NULL);
  //      if (retval < 0)
  //      {
  //        printf("unable to create WATCHDOG_TASK task\n");
  //        for (int i = 0; i < 4; i++)
  //        {
  //          retval = task_create("WATCHDOG_TASK", 100, 4096, WDG_TASK, NULL);
  //          if (retval >= 0)
  //          {
  //            break;
  //            return 0;
  //          }
  //        }
  //        return -1;
  //      }
  //    }
  //  }

  // Setup();//TODO this setup is used to create a text file first if not created. the process will be handled by storage app
  // RUN_HK();

  /*TODO : REMOVE LATER Independent testing*/
  if (strcmp(argv[1], "reset") == 0)
  {
    int fd;
    syslog(LOG_DEBUG, "OBC reset command received\n***********OBC will reset in 10 seconds******\n");
    do
    {
      fd++;
      usleep(1000);
    } while (fd < 1000);

    {
      gpio_write(GPIO_GBL_RST, true);
    }
  }
  else if (strcmp(argv[1], "ant") == 0)
  {
    Antenna_Deployment(argc, argv);
    // RUN_ADC();
  }
  else if (strcmp(argv[1], "epdm") == 0)
  {
    epdm_operation();
  }
  else if (strcmp(argv[1], "cam") == 0)
  {
    cam_operation();
  }
  else if (strcmp(argv[1], "adcs") == 0)
  {
    adcs_operation(0x02);
  }

  /*TODO : REMOVE LATER Independent testing*/
  else
  {
    bool g_wdg_task_started = false;
    if (g_wdg_task_started)
    {
      printf("[WDG TASK] Task already started.\n");
      return EXIT_SUCCESS;
    }
    else
    {

      bool g_reset_task_started = false;
      if (g_reset_task_started)
      {
        printf("[Reset TASK] Task already started.\n");
        return EXIT_SUCCESS;
      }
      else
      {
        printf("[Reset TASK] Task  started.\n");
        int retval = task_create("RESET_TASK_APP", 100, 600, global_reset, NULL);
        if (retval < 0)
        {
          printf("unable to create RESET_TASK_APP task\n");
          for (int i = 0; i < 4; i++)
          {
            retval = task_create("RESET_TASK_APP", 100, 600, global_reset, NULL);
            if (retval >= 0)
            {
              g_reset_task_started = true;
              break;
              return 0;
            }
          }
          return -1;
        }
      }
      if (g_commander_task_started)
      {
        printf("[COMMANDER TASK] Task already started.\n");
        return EXIT_SUCCESS;
      }
      else
      {
        int retval = task_create("COMMANDER_TASK_APP", 100, 10096, COM_TASK, NULL);
        if (retval < 0)
        {
          printf("unable to create COMMANDER_TASK_APP task\n");
          for (int i = 0; i < 4; i++)
          {
            retval = task_create("COMMANDER_TASK_APP", 100, 10096, COM_TASK, NULL);
            if (retval >= 0)
            {
              g_commander_task_started = true;
              break;
              return 0;
            }
          }
          return -1;
        }
      }
      printf("************************************************\n");
      if (g_beacon_task_started)
      {
        printf("[BEACON TASK] Task already started.\n");
        return EXIT_SUCCESS;
      }
      else
      {
        int retval = task_create("BEACON_TASK_APP", 100, 1000, send_beacon, NULL);
        if (retval < 0)
        {
          printf("unable to create BEACON_TASK_APP task\n");
          for (int i = 0; i < 4; i++)
          {
            retval = task_create("BEACON_TASK_APP", 100, 1000, send_beacon, NULL);
            if (retval >= 0)
            {
              g_beacon_task_started = true;
              break;
              return 0;
            }
          }
          return -1;
        }
      }
      // #endif
      printf("************************************************\n");
      // }
      // TODO: after checking flags data are being written/read correctly, we'll enable satellite health things as well and have a basic complete work queue functions except UART
    }
    return 0;
  }
}

// //COM
int turn_msn_on_off(uint8_t subsystem, uint8_t state)
{
  gpio_write(GPIO_MSN3_EN, false);

  gpio_write(GPIO_DCDC_MSN_3V3_2_EN, state);
  stm32_gpiowrite(GPIO_MSN_3V3_EM_EN, state);

  gpio_write(GPIO_MSN1_EM_EN, false);
  gpio_write(GPIO_MSN2_EN, false);

  // gpio_write(GPIO_MSN_3V3_EM_EN, state);
  // gpio_write(GPIO_DCDC_MSN_3V3_2_EN, state);

  switch (subsystem)
  {
  case 1:
    printf("turning ADCS mission state: %d\n", state);
    gpio_write(GPIO_MSN1_EM_EN, state);
    sleep(1);
    gpio_write(GPIO_MSN_5V_EN, state);
    gpio_write(GPIO_DCDC_5V_EN, state);
    break;
  case 2:
    printf("Turning CAM mission state: %d\n", state);
    sleep(1);
    gpio_write(GPIO_MSN2_EN, state);
    gpio_write(GPIO_MSN_5V_EN, state);
    gpio_write(GPIO_DCDC_5V_EN, state);
    break;
  case 3:
    printf("Turning EPDM mission state: %d\n", state);
    gpio_write(GPIO_MSN3_EN, state);
    sleep(1);
    break;
  default:
    printf("Wrong subsystem selected\n");
    break;
  }
}
// #include

// //COM
void send_flash_data(uint8_t *beacon_data)
{
  int fd = open(COM_UART, O_RDWR);
  int ret2;
  if (fd < 0)
  {
    printf("unable to open: %s\n", COM_UART);
    return -1;
  }
  sleep(2);

  printf("Turning on  4v dcdc line..\n");
  gpio_write(GPIO_DCDC_4V_EN, 1);
  printf("Turning on COM 4V line..\n");
  gpio_write(GPIO_COM_4V_EN, 1);

  uint8_t ack_com[43];
  int ret = write(fd, beacon_data, BEACON_DATA_SIZE);
  usleep(10000);
  if (ret < 0)
  {
    printf("unable to send data\n");
    for (int i = 0; i < BEACON_DATA_SIZE - 1; i++)
    {
      ret = write(fd, &beacon_data[i], 1);
      syslog(LOG_DEBUG, "%02x ", beacon_data[i]);
      usleep(1000);
    }
    if (ret < 0)
    {
      printf("Unable to send data through byte method..\n");
      return -1;
    }
    ret2 = read(fd, ack_com, sizeof(ack_com));
  }
  else
  {
    if (ret2 >= 0)
    {
      printf("Flash data sent to COM successfully\n");
      printf("data is:::::::: %s\n", beacon_data);
    }
    else
    {
      ret = write(fd, beacon_data, BEACON_DATA_SIZE);
      usleep(10000);
      if (ret < 0)
      {
        printf("unable to send data\n");
        for (int i = 0; i < BEACON_DATA_SIZE; i++)
        {
          ret = write(fd, &beacon_data[i], 1);
          usleep(1000);
        }
        if (ret < 0)
        {
          printf("Unable to send data through byte method..\n");
          return -1;
        }
      }
    }
  }
}

// COM
/****************************************************************************
 * Send beacon data to COM
 * To be done every 90 seconds
 ****************************************************************************/
int send_beacon_data()
{
  if (MISSION_STATUS.FLASH_OPERATION == false)
  {

    if (COM_BUSY == 1)
    {
      return -1;
    }
    else
    {
      uint8_t beacon_data[BEACON_DATA_SIZE];
      switch (beacon_type)
      {
      case 0:

        serialize_beacon_a(beacon_data);
        beacon_data[1] = 0xb1;
        beacon_data[2] = 0x51;
        beacon_data[83] = 0x7e;
        break;
      case 1:
        serialize_beacon_b(beacon_data);
        beacon_data[1] = 0xb2;
        beacon_data[2] = 0x51;
        break;
      default:
        printf("wrong case selected\n");
        return -1;
        break;
      }
      beacon_data[0] = 0x53;
      beacon_data[83] = 0x7e;

      beacon_data[84] = '\0';
      int fd; //
      // fd= send_data_uart(COM_UART, beacon_data, sizeof(beacon_data));
      //  fd = send_data_uart(COM_UART, test, sizeof(test));

      printf("beacon data size %d\n", sizeof(beacon_data));
      fd = open(COM_UART, O_WRONLY);
      int count;
      if (fd < 0)
      {
        do
        {
          fd = open(COM_UART, O_WRONLY);
          count += 1;
        } while (open(COM_UART, O_WRONLY) >= 0 | count < 5);
        printf("unable to open: %s\n", COM_UART);
        return -1;
      }
      sleep(2);

      printf("Turning on  4v dcdc line..\n");
      gpio_write(GPIO_DCDC_4V_EN, 1);
      printf("Turning on COM 4V line..\n");
      gpio_write(GPIO_COM_4V_EN, 1);

      int ret = write(fd, beacon_data, BEACON_DATA_SIZE);
      usleep(10000);
      if (ret < 0)
      {
        printf("unable to send data\n");
        for (int i = 0; i < BEACON_DATA_SIZE; i++)
        {
          ret = write(fd, &beacon_data[i], 1);
          usleep(1000);
        }
        if (ret < 0)
        {
          printf("Unable to send data through byte method..\n");
          return -1;
        }
      }

      /*To delete*/
      if (beacon_status == 0)
      {
        printf("\nbeacon 1:\n");
        digipeating = 0;
      }
      else
      {
        printf("\nbeacon 2:\n");
        digipeating = 1;
      }
      beacon_type = !beacon_type;

      printf("Beacon Type %d sequence complete\n", beacon_type);
      // work_queue(HPWORK, &work_beacon, send_beacon_data, NULL, SEC2TICK(BEACON_DELAY));
    }
  }
  return 0;
}

void FirstFunction()
{
  CRITICAL_FLAGS rd_flags_int;
  struct file fp;

  CRITICAL_FLAGS rd_flags_mfm = {0xff};
  uint8_t mfm_have_data = 0;
  ssize_t read_size_mfm = 0;

  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  { // internal flash file opened successfully
    syslog(LOG_INFO, "Printing Internal flash flag data.\n");
    up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
    print_critical_flag_data(&rd_flags_int);
  }
  else
  {
    syslog(LOG_ERR, "Error opening internal flash atempt 1......\n ");
  }
  close(fd);
  int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
  if (fd1 >= 0)
  {
    read_size_mfm = file_read(&fp, &rd_flags_mfm, sizeof(CRITICAL_FLAGS));
  }
}

// //Commander //COM
// // TODO: add work queue to antenna deployment
void Antenna_Deployment(int argc, char *argv[])
{
  int i = 0;
  do
  {
    sleep(1);
    if (i > 25)
      printf("%d second has passesd\n", i);
    i++;
  } while (i < 30);

  printf("Entering antenna deployment sequence\n");
  int retval, retval1 = 0;
  // CRITICAL_FLAGS ant_check;
  // ant_check.ANT_DEP_STAT = critic_flags.ANT_DEP_STAT;
  // printf("Antenna Deployment Flag: %d\n", critic_flags.ANT_DEP_STAT);
  // TODO: add redundancy (check UL status along with antenna deployment status)
  // if (critic_flags.ANT_DEP_STAT == UNDEPLOYED && critic_flags.UL_STATE == UL_NOT_RX)
  // if(argv[2]=="1" | argv[2] ==1)
  {
    for (int i = 0; i <= 2; i++)
    {
      printf("Turning on burner circut\nAttempt: %d\n", i + 1);
      retval = gpio_write(GPIO_BURNER_EN, true);
      retval1 = gpio_write(GPIO_UNREG_EN, true);
      // RUN_ADC();
      // TODO : manage antenna deployement time
      sleep(8); // 8 seconds
      printf("Turning off burner circuit\n");
      gpio_write(GPIO_UNREG_EN, false);
      gpio_write(GPIO_BURNER_EN, false);
      // usleep(1000 * 1000 * 2); // 2 seconds
      sleep(10);
      printf("%d Antenna deployment sequence completed\n", i);
    }
  }
  // ant_check.ANT_DEP_STAT = DEPLOYED;
  // ant_check.UL_STATE
  // // store_flag_data(&ant_check);
  // // printf("Updated flag data...\n");
  // // check_flag_data();
  // print_critical_flag_data(&critic_flags);
}

void adcs_operation(uint8_t mode)
{
  if (MISSION_STATUS.ADCS_MISSION == false && MISSION_STATUS.EPDM_MISSION == false && MISSION_STATUS.CAM_MISSION == false)
  {
    MISSION_STATUS.ADCS_MISSION = true;
    int hand;
    turn_msn_on_off(1, 0);
    sleep(1);
    turn_msn_on_off(1, 1);
    sleep(4);
    // uint8_t data2[7] = {0x53,0x0e,0x0d,0x0e,0x01,0x7e};
    uint8_t data2[7] = {0x53, 0x0a, 0x0d, 0x0c, 0x01, 0x7e};
    data2[4] = mode;

    do
    {
      hand = handshake_MSN_ADCS(1, data);
      // hand = handshake_MSN(2, data);

    } while (hand < 0);
    hand = 0;
    uint8_t ret, fd;
    sleep(1);

    do
    {
      hand = handshake_MSN_ADCS(1, data2);
    } while (hand < 0);
    sleep(2);
    int p = 0;
    uint8_t data3, data4;
    uint32_t counter1 = 0;
    uint8_t cam[90] = {'\0'};
    printf("**************Starting data receive*************\n");
    int fd2 = open(ADCS_UART, O_RDONLY);

    while (1)
    {
      data4 = data3;
      ret = read(fd2, &data3, 1);
      syslog(LOG_DEBUG, "%02x ", data3);
      cam[counter1] = data3;
      if (data4 == 0xff && data3 == 0xd9)
      {
        syslog(LOG_DEBUG, "%02x ", data3);

        break;
      }
      if (counter1++ > 89)
        break;
      // if(data4== 0xff);
      counter1++;
      // break;
    }
    // cam[counter1]='\0';
    close(fd2);
    sleep(1);
    turn_msn_on_off(1, 0);
    MISSION_STATUS.ADCS_MISSION = false;
    usleep(10000);
    syslog(LOG_DEBUG, "TOtal data received %d\n ADCS operation success\n", counter1);
    sleep(1);
    cam[sizeof(cam) - 2] = 0xff;
    cam[sizeof(cam) - 1] = 0xd9;
    sleep(1);
    mission_data("/adcs.txt", &cam, counter1);
    uint8_t ack[85] = {0x53, 0xac, 0x04, 0x01, 0x05, 0x05, 0x7e, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x70, 0x71, 0x72, 0x1e, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x80, 0x7e};
    sleep(1);
    send_data_uart(COM_UART, ack, sizeof(ack));
  }
}
void cam_operation()
{
  if (MISSION_STATUS.ADCS_MISSION == false && MISSION_STATUS.EPDM_MISSION == false && MISSION_STATUS.CAM_MISSION == false)
  {
    MISSION_STATUS.CAM_MISSION = true;
    MISSION_STATUS.FLASH_OPERATION = true;
    int hand;
    turn_msn_on_off(2, 0);
    sleep(1);

    turn_msn_on_off(2, 1);
    sleep(4);
    // uint8_t data2[7] = {0x53,0x0e,0x0d,0x0e,0x01,0x7e};
    uint8_t data2[7] = {0x53, 0x0c, 0x0a, 0x0e, 0x01, 0x7e};

    do
    {
      hand = handshake_MSN(2, data);
    } while (hand < 0);
    hand = 0;
    uint8_t ret, fd;
    char data[240];
    sleep(1);

    // sleep(1);
    do
    {
      hand = handshake_MSN(3, data2);
    } while (hand < 0);
    sleep(3);
    // if(hand == 0)
    {
      syslog(LOG_DEBUG, "Command %s sent\n", data2);

      int p = 0;
      uint8_t data3, data4;
      uint32_t counter1 = 0;
      uint8_t cam[5500] = {'\0'};
      int fd2 = open(CAM_UART, O_RDONLY);
      // sleep(10);
      sleep(3);
      while (1)
      {
        data4 = data3;
        ret = read(fd2, &data3, 1);
        printf("%02x ", data3);
        cam[counter1] = data3;
        if (data4 == 0xff && data3 == 0xd9)
        {
          break;
        }
        // if(data4== 0xff);
        counter1++;
        // break;
      }
      // cam[counter1]='\0';
      close(fd2);

      turn_msn_on_off(2, 0);
      MISSION_STATUS.FLASH_OPERATION = false;
      MISSION_STATUS.CAM_MISSION = false;

      usleep(10000);
      syslog(LOG_DEBUG, "TOtal data received %d\n CAM operation success\n", counter1);
      sleep(2);
      mission_data("/cam.txt", &cam, counter1);
      uint8_t ack[85] = {0x53, 0xac, 0x04, 0x01, 0x05, 0x05, 0x7e, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x70, 0x71, 0x72, 0x1e, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x80, 0x7e};
      sleep(1);
      send_data_uart(COM_UART, ack, sizeof(ack));
    }
  }
}

void epdm_operation()
{
  int hand;
  int fd;
  if (MISSION_STATUS.ADCS_MISSION == false && MISSION_STATUS.CAM_MISSION == false)
  {
    MISSION_STATUS.EPDM_MISSION = true;
    char *dev_path = EPDM_UART;
    turn_msn_on_off(3, 0);
    sleep(1);
    turn_msn_on_off(3, 1);
    sleep(1);
    sleep(1);
    sleep(1);
    sleep(1);
    sleep(1);
    sleep(1);
    sleep(1);

    hand = handshake_MSN(3, data);

    uint8_t data2[] = {0x53, 0x0e, 0x0d, 0x0e, 0x01, 0x7e};
    uint8_t ret;
    do
    {
      hand = handshake_MSN(3, data2);
    } while (hand < 0);
    syslog(LOG_DEBUG, "Command %s sent\n", data2);

    int p = 0;
    uint8_t data3, data4;
    uint32_t counter1 = 0;
    uint8_t cam[4500] = {'\0'};
    int fd2 = open(CAM_UART, O_RDONLY);

    while (1)
    {
      data4 = data3;
      ret = read(fd2, &data3, 1);
      // syslog(LOG_DEBUG, "%02x ", data3);
      if (counter1 % 200 == 0)
      {
        syslog(LOG_DEBUG, "COUNTER : %d", counter1);
      }
      cam[counter1] = data3;
      if (data4 == 0xff && data3 == 0xd9)
      {
        break;
      }
      counter1++;
    }
    close(fd2);
    turn_msn_on_off(3, 0);
    MISSION_STATUS.EPDM_MISSION = false;

    // free(data2);
    syslog(LOG_DEBUG, "Total data received %d\n EPDM operation succeded\n", counter1);
    sleep(2);
    mission_data("/epdm.txt", &cam, counter1);
    uint8_t ack[85] = {0x53, 0xac, 0x04, 0x01, 0x05, 0x05, 0x7e, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x70, 0x71, 0x72, 0x1e, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x80, 0x7e};
    sleep(1);
    send_data_uart(COM_UART, ack, sizeof(ack));
    // free(cam);
    // free()
  }
}

int configure_watchdog(int fd, int timeout)
{
  /* Set the watchdog timeout */
  int ret = ioctl(fd, WDIOC_SETTIMEOUT, (unsigned long)timeout);
  if (ret < 0)
  {
    perror("Failed to set watchdog timeout");
    return -1;
  }

  /* Start the watchdog */
  ret = ioctl(fd, WDIOC_START, 0);
  if (ret < 0)
  {
    perror("Failed to start watchdog");
    return -1;
  }

  return 0;
}

void watchdog_refresh_task(int fd)
{
  while (1)
  {
    /* Refresh the watchdog */
    int ret = ioctl(fd, WDIOC_KEEPALIVE, 0);
    if (ret < 0)
    {
      perror("Failed to refresh watchdog");
    }

    /* Wait for the next refresh interval */
    sleep(2);
  }
}

// void RUN_ADC(){
//   // read_int_adc1();
//   // read_int_adc3();
//   ext_adc_main();
//   make_satellite_health();
//   store_sat_health_data(&sat_health);
//   print_satellite_health_data(&sat_health);
// }