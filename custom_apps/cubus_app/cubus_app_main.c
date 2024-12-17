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
#include <pthread.h>
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
#include <sensor/adc.h>

#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/lis3mdl.h>
#include <sensor/command.h>

#include <sensor/accel.h>
#include <sensor/gyro.h>
#include <sensor/flash_operation.h>

#include <time.h>
#include <fcntl.h>

#include <poll.h>
#include <sched.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
// wdog include
#include "watchdog.h"
int wdog_fd = -1;
// wdog
#define ANT_DEPLOY_TIME 60                  // 60 *30
#define VOLT_DIV_RATIO ((1100 + 931) / 931) // ratio of voltage divider used
float x[8], y[8];

int ads7953_receiver(int argc, FAR char *argv[]);

float convert1(data);
void subscribe_and_retrieve_data();
void ADC_Temp_Conv(float *adc_conv_buf, float *temp_buf, int channel);
void print_beacon_a();
void new_camera_operation();
void print_beacon_b();
void handle_reservation_command(int, struct reservation_command);
void set_time(uint32_t);
void get_top_rsv(struct reservation_command *res);
void flash_operation_data(uint16_t loop);
uint64_t get_time_data();
struct sensor_rgb sensor_rgb_0;
struct reservation_command TO_EXECUTE;
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
#define BEACON_DATA_SIZE 85 + 1
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

/*INT adc added
 */
int ret, elapsed = 0, required = 10;
#if defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1) || defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3)
static void adc_devpath(FAR struct adc_state_s *adc, FAR const char *devpath);
#endif
#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1
static struct adc_state_s g_adcstate1;
#endif

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3
static struct adc_state_s g_adcstate3;
#endif
#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1
int_adc_config_s adc1_config;
struct adc_msg_s int_adc1_sample[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE];
#endif

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3
int_adc_config_s adc3_config;
struct adc_msg_s int_adc3_sample[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE];
#endif
/**/
/*Private variable start*/
// Declare the instance of the struct
uint8_t COM_HANDSHAKE_STATUS = 0;
bool FLASH_OPERATION = false;
bool COM_UART_BUSY = false;
bool FLASH_UORB_RESPONDING = true;
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

bool FIRST_RSV_CMD = false;
uint8_t RSV_CMD[43] = {0x72, 0x9c, 0x64, 0xa6, 0x92, 0x40, 0xe0, 0x72, 0x9c, 0x64, 0xa6, 0x70, 0x72, 0x61, 0x03, 0xf0, '\0'};
uint32_t timer = 0;
// uint8_t KILL_SW_COUNTER = 0;
// uint64_t KILL_SW_LST_TIME = 0;
struct KILL_SW
{
  time_t timestamps[3];
  int count;
};
struct KILL_SW kill_sw;

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
CRITICAL_FLAGS critic_flags = {0x00};

struct sensor_accel imu_acc_data;
struct sensor_gyro imu_gyro_data;
struct mpu6500_imu_msg raw_imu;
struct command command_uorb;
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
  ADCS_TXT,
  CAMERA_NIR_TXT,
  DIGIPEATER_TXT
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

struct SEEK_POINTER
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
};

/*Private variable end*/

/*Private function prototypes declaration start*/
void make_satellite_health();
void print_satellite_health_data(satellite_health_s *sat_health);

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
void flash_read_operation_uorb();
void save_critics_flags(const CRITICAL_FLAGS *flags)
{
  int fd = open("/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_WRONLY | O_CREAT, 0644);
  if (fd < 0)
  {
    perror("Failed to open flags.txt for writing");
    return;
  }
  write(fd, flags, sizeof(CRITICAL_FLAGS));
  close(fd);
}

int load_critics_flags(CRITICAL_FLAGS *flags)
{
  int fd = open("/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_RDONLY);
  if (fd < 0)
  {
    perror("Failed to open flags.txt for reading");
    return -1;
  }
  ssize_t bytesRead = read(fd, flags, sizeof(CRITICAL_FLAGS));
  close(fd);
  if (bytesRead != sizeof(CRITICAL_FLAGS))
  {
    perror("Failed to read complete flags data");
    return -1;
  }
  return 0;
}

/*Private function prototypes declaration end */

/****************************************************************************
 * Send data from UART through any UART path
 ****************************************************************************/

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1
int read_int_adc1()
{

  UNUSED(ret);

  /* Check if we have initialized */
  if (!g_adcstate1.initialized)
  {
    /* Initialization of the ADC hardware must be performed by
     * board-specific logic prior to running this test.
     */

    /* Set the default values */

    adc_devpath(&g_adcstate1, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_DEVPATH);

    g_adcstate1.initialized = true;
  }

  g_adcstate1.count = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_NSAMPLES;

  /* Parse the command line */

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

  // printf("adc_main: g_adcstate.count: %d\n", g_adcstate1.count);

  // /* Open the ADC device for reading */

  // printf("adc_main: Hardware initialized. Opening the ADC device: %s\n",
  //        g_adcstate1.devpath);

  /* Opening internal ADC1 */
  adc1_config.fd = open(CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_DEVPATH, O_RDONLY);
  if (adc1_config.fd < 0)
  {
    // printf("adc_main: open %s failed: %d\n", g_adcstate1.devpath, errno);
    adc1_config.errval = 2;
    goto errout;
  }
  elapsed = 0;
  while (elapsed < required)
  {
    usleep(1);
    elapsed++;
  }
  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */
  // UNUSED(elapsed);
  // UNUSED(required);
  for (int j = 0; j < 1; j++)
  {
    /* Flush any output before the loop entered or from the previous pass
     * through the loop.
     */
    elapsed = 0;
    fflush(stdout);

#ifdef CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_SWTRIG
    /* Issue the software trigger to start ADC conversion */

    ret = ioctl(adc1_config.fd, ANIOC_TRIGGER, 0);
    if (ret < 0)
    {
      int errcode = errno;
      printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
    }
#endif

    /* Read up to CONFIG_CUSTOM_APPS_ADC_GROUPSIZE samples */

    adc1_config.readsize = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE * sizeof(struct adc_msg_s);
    adc1_config.nbytes = read(adc1_config.fd, int_adc1_sample, adc1_config.readsize);

    // printf("Readsize: %d \n nbytes: %d\n CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE : %d \n ADCSTATE READCOUNT: %d \r\n", adc1_config.readsize, adc1_config.nbytes, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE, g_adcstate1.count);

    /* Handle unexpected return values */
    if (adc1_config.nbytes < 0)
    {
      adc1_config.errval = errno;
      if (adc1_config.errval != EINTR)
      {
        printf("adc_main: read %s failed: %d\n",
               g_adcstate1.devpath, adc1_config.errval);
        adc1_config.errval = 3;
        goto errout_with_dev;
      }

      printf("adc_main: Interrupted read...\n");
    }
    else if (adc1_config.nbytes == 0)
    {
      printf("adc_main: No data read, Ignoring\n");
    }

    /* Print the sample data on successful return */

    else
    {
      int nsamples = adc1_config.nbytes / sizeof(struct adc_msg_s);
      if (nsamples * sizeof(struct adc_msg_s) != adc1_config.nbytes)
      {
        // printf("adc_main: read size=%ld is not a multiple of "
        //        "sample size=%d, Ignoring\n",
        //        (long)adc1_config.nbytes, sizeof(struct adc_msg_s));
      }
      else
      {
        // Print int adc values
        // printf("\nSample:\n");
        float chan;
        for (int i = 0; i < nsamples; i++)
        {
          // printf("%d: channel: %d value: %" PRId32 " \n",
          //        i, int_adc1_sample[i].am_channel, int_adc1_sample[i].am_data);
        }
        // sat_health.sol_p2_c = (int16_t)int_adc1_sample[15].am_data;
        // sat_health.sol_p1_c = (int16_t)int_adc1_sample[14].am_data;
        // // sat_health.raw_v = (int16_t)int_adc1_sample[12].am_data;
        // sat_health.sol_p3_c = (int16_t)int_adc1_sample[8].am_data;
        // sat_health.sol_p4_c = (int16_t)int_adc1_sample[7].am_data;
        // sat_health.sol_p5_c = (int16_t)int_adc1_sample[6].am_data;
        // sat_health.sol_t_c = (int16_t)int_adc1_sample[0].am_data;
        // // sat_health.rst_3v3_c = (int16_t)int_adc1_sample[i].am_data;
        // sat_health.raw_c = (int16_t)int_adc1_sample[12].am_data;
        // sat_health.v3_main_c = (int16_t)int_adc1_sample[4].am_data;
        // sat_health.v3_com_c = (int16_t)int_adc1_sample[3].am_data;
        // sat_health.v3_2_c = (int16_t)int_adc1_sample[13].am_data;
        // // sat_health.v5_c = (int16_t)int_adc1_sample[].am_data;
        // sat_health.unreg_c = (int16_t)int_adc1_sample[10].am_data;
        // sat_health.v4_c = (int16_t)int_adc1_sample[11].am_data;
        // sat_health.batt_c = (int16_t)int_adc1_sample[9].am_data;
        // sat_health.ba
      }
    }

    if (g_adcstate1.count && --g_adcstate1.count <= 0)
    {
      break;
    }
  }

  close(adc1_config.fd);
  return OK;

/* Error exits */
errout_with_dev:
  close(adc1_config.fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  close(adc1_config.fd);
  return adc1_config.errval;
}
#endif // CONFIG_EAMPLES_CUBUS_USE_ADC_1

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3
int read_int_adc3()
{

  UNUSED(ret);

  /* Check if we have initialized */
  if (!g_adcstate3.initialized)
  {
    /* Initialization of the ADC hardware must be performed by
     * board-specific logic prior to running this test.
     */

    /* Set the default values */

    adc_devpath(&g_adcstate3, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_DEVPATH);

    g_adcstate3.initialized = true;
  }

  g_adcstate3.count = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_NSAMPLES;

  /* Parse the command line */

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

  // printf("adc_main: g_adcstate.count: %d\n", g_adcstate3.count);

  // /* Open the ADC device for reading */

  // printf("adc_main: Hardware initialized. Opening the ADC device: %s\n",
  //        g_adcstate3.devpath);

  /* Opening internal ADC1 */
  adc3_config.fd = open("/dev/adc2", O_RDONLY);
  if (adc3_config.fd < 0)
  {
    printf("adc_main: open %s failed: %d\n", g_adcstate3.devpath, errno);
    adc3_config.errval = 2;
    goto errout;
  }
  elapsed = 0;
  while (elapsed < required)
  {
    usleep(1);
    elapsed++;
  }
  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */
  // UNUSED(elapsed);
  // UNUSED(required);
  for (int k = 0; k < 1; k++)
  {
    /* Flush any output before the loop entered or from the previous pass
     * through the loop.
     */
    elapsed = 0;
    fflush(stdout);

#ifdef CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_SWTRIG
    /* Issue the software trigger to start ADC conversion */

    ret = ioctl(adc3_config.fd, ANIOC_TRIGGER, 0);
    if (ret < 0)
    {
      int errcode = errno;
      printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
    }
#endif

    /* Read up to CONFIG_CUSTOM_APPS_ADC_GROUPSIZE samples */

    adc3_config.readsize = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE * sizeof(struct adc_msg_s);
    adc3_config.nbytes = read(adc3_config.fd, int_adc3_sample, adc3_config.readsize);

    // printf("Readsize: %d \n nbytes: %d\n CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE : %d \n ADCSTATE READCOUNT: %d \r\n", adc3_config.readsize, adc3_config.nbytes, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE, g_adcstate3.count);

    /* Handle unexpected return values */
    if (adc3_config.nbytes < 0)
    {
      adc3_config.errval = errno;
      if (adc3_config.errval != EINTR)
      {
        printf("adc_main: read %s failed: %d\n",
               g_adcstate3.devpath, adc3_config.errval);
        adc3_config.errval = 3;
        goto errout_with_dev;
      }

      printf("adc_main: Interrupted read...\n");
    }
    else if (adc3_config.nbytes == 0)
    {
      printf("adc_main: No data read, Ignoring\n");
    }

    /* Print the sample data on successful return */
    else
    {
      int nsamples = adc3_config.nbytes / sizeof(struct adc_msg_s);
      if (nsamples * sizeof(struct adc_msg_s) != adc3_config.nbytes)
      {
        // printf("adc_main: read size=%ld is not a multiple of "
        //        "sample size=%d, Ignoring\n",
        //        (long)adc3_config.nbytes, sizeof(struct adc_msg_s));
      }
      else
      {
        // printf("Sample:\n");
        // for (int i = 0; i < nsamples; i++)
        // {
        //   printf("%d: channel: %d value: %" PRId32 "\n",
        //          i, int_adc3_sample[i].am_channel, int_adc3_sample[i].am_data);
        // }
      }
    }

    if (g_adcstate3.count && --g_adcstate3.count <= 0)
    {
      break;
    }
  }

  close(adc3_config.fd);
  return OK;

/* Error exits */
errout_with_dev:
  close(adc3_config.fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  close(adc3_config.fd);
  return adc3_config.errval;
}
#endif // CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3

int send_data_uart(char *dev_path, uint8_t *data, uint16_t size)
{
  double fd;
  int i;
  int count = 0, ret;
  int wr1;
  fd = open_uart(COM_UART);
  if (fd < 0)
  {
    printf("error opening %s\n", dev_path);
    return fd;
  }
  else
  {

    printf("Turning on  4V dcdc line..\n");
    gpio_write(GPIO_DCDC_4V_EN, 1);
    printf("Turning on COM 4V line..\n");
    gpio_write(GPIO_COM_4V_EN, 1);
    printf("Opening uart dev path : %s\n", dev_path);
    wr1 = write(fd, data, size);
    if (wr1 < 0)
    {
      printf("Unable to write data\n");
      return wr1;
    }
    else
    {
      wr1 = write(fd, data, size);
    }

    // printf("\ndata is %\n", wr1);
    for (int i = 0; i < size; i++)
    {
      {
        printf("%02x ", data[i]);
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
    printf("\n%d bytes written\nwdog refreshed\n", wr1);
    // pet_counter = 0; // TODO rethink on this later internal wdog
  }
  close_uart(fd);
  return wr1;
}

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
  if (critic_flags.OPER_MODE == NRML_MODE)
  {
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
      // syslog(LOG_SYSLOG, "COmmand received %s\n", COM_RX_DATA);
      parse_command(COM_RX_DATA);

      uint8_t commands[COM_DATA_SIZE] = {11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 0x01, 0x01, 0xca, 0xd1, 0xf3, 0, 0, 0, 0, 0, 0, 00, 0, 0};
      printf("\n-------------parse command starting-------------\n");

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
    }

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
  pet_counter = 0;
  syslog(LOG_DEBUG, "The supplied command is incorrect");
  return;
}

void parse_command(uint8_t COM_RX_DATA[COM_DATA_SIZE])
{
  uint8_t ack[BEACON_DATA_SIZE] = {0x53, 0xac, 0x04, 0x01, 0x62, 0x63, 0x7e};
  syslog(LOG_DEBUG, "command received is :");
  for (int i = 0; i < COM_DATA_SIZE; i++)
  {
    printf("%02X ", COM_RX_DATA[i]);
  }
  ack[BEACON_DATA_SIZE - 2] = 0x7e;
  ack[BEACON_DATA_SIZE - 1] = 0x7e;
  if (COM_RX_DATA[0] == 0x53 && critic_flags.OPER_MODE == NRML_MODE)
  { // SEND 85 BYTES BACK
    // 53 0A 2B 53 02 53 digipeating format
    // if (digipeating == 1 && COM_RX_DATA[1] == 0x0a && COM_RX_DATA[2] == 0x2b && COM_RX_DATA[3] == 0x53 && COM_RX_DATA[4] == 0x02 && COM_RX_DATA[5] == 0x53)
    // {
    //   send_data_uart(COM_UART, COM_RX_DATA, sizeof(COM_RX_DATA));
    //   return 33;
    // }
    // else
    if (COM_RX_DATA[0] == 0x53 & COM_RX_DATA[1] == 0xac & COM_RX_DATA[2] == 0x04)
    {
      if (COM_RX_DATA[3] == 0X02 & COM_RX_DATA[4] == 0XFC & COM_RX_DATA[5] == 0XEE)
      { // FALSE COMMAND
        // uint8_t ack1[] = {0x53, 0xac, 0x04, 0x02, 0xFC, 0xEE};
        ack[3] = 0x02;
        ack[4] = 0xfc;
        ack[5] = 0xee;
        ack[6] = 0x7e;

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
  else if (COM_RX_DATA[0] == 0x44 && digipeating == 1 && COM_RX_DATA[1] == 0x0a)
  {
    printf("Digipeater command of size %d received\n", COM_RX_DATA[2]);
    printf("\nDigipeater full command is :\n");
    for (int i = 0; i < 22; i++)
    {
      printf("%02X ", COM_RX_DATA[i]);
    }
    printf("\n--------------------------------\n---------------------------------\nDigipeater msg is : \n");
    for (int i = 0; i < COM_RX_DATA[2]; i++)
    {
      printf("%02x ", COM_RX_DATA[i + 22]);
    }
    printf("\n---------\n");
    // COM_RX_DATA = 0x7e;
    uint8_t digipeater_data[86] = {'\0'};
    struct file file_ptr;
    int fd = file_open(&file_ptr, "/mnt/fs/mfm/mtd_mission/digipeater.txt", O_CREAT | O_WRONLY | O_APPEND);
    ssize_t write_bytes = file_write(&file_ptr, COM_RX_DATA, COM_RX_DATA[2]);
    file_write(&file_ptr, 0x7e, 1);
    file_close(&file_ptr);
    digipeater_data[0] = 0x53; // TODO: save data with callsign to FM
    // int fd = open(COM_UART, O_WRONLY);
    for (int i = 1; i < BEACON_DATA_SIZE; i++)
    {
      if (i <= 43)
        digipeater_data[i] = COM_RX_DATA[i];
      // write(fd, digipeater_data[i],1);
    }
    // send_data_uart(COM_UART, &digipeater_data,86);
    digipeater_data[BEACON_DATA_SIZE - 1] = 0x7e;
    digipeater_data[BEACON_DATA_SIZE - 2] = 0x7e;

    // TODO: Use storage_manager instead
    send_data_uart(COM_UART, digipeater_data, sizeof(digipeater_data));
    digipeating = 0;
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

      uint8_t cmds[5];

      cmds[0] = (uint8_t)COM_RX_DATA[HEADER + 1];
      cmds[1] = (uint8_t)COM_RX_DATA[HEADER + 2];
      cmds[2] = (uint8_t)COM_RX_DATA[HEADER + 3];
      cmds[3] = (uint8_t)COM_RX_DATA[HEADER + 4];
      cmds[4] = (uint8_t)COM_RX_DATA[HEADER + 5];
      syslog(LOG_DEBUG, "**********\n*************\nHere the com rx data is %02x,%02x,%02x,%02x,%02x,%02x\n********************\n********************\n",
             HEADER, cmds[0], cmds[0 + 1], cmds[0 + 2], cmds[0 + 3], cmds[0 + 4]);

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
      pet_counter = 0;

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
      if (cmds[3] == 0x00 && cmds[4] == 0x00)
      {
        switch (MCU_ID)
        {
        case OBC_MCU: /*
                      Command to perform flash operations by the OBC
                      */

          if (cmds[0] == 0x1a && cmds[1] == 0xe0 && cmds[2] == 0x1e)
          {
            printf("\n-------------------Satellite reset command received-----------------\n Resets in 2 seconds\n");
            sleep(1);
            gpio_write(GPIO_GBL_RST, true);
          }
          else if (cmds[0] == 0x75 && cmds[1] == 0x6e && cmds[2] == 0x69) // TIme updation using unix timestamp
          {
            printf("\n Command received to set the clock of OBC\n");
            // header+6

            uint32_t timestamp_received = 0;
            printf("Received command is \n");
            for (int i = 0; i < 20; i++)
            {
            }
            for (int i = 0; i < 4; i++)
            {
              printf("%02x ", COM_RX_DATA[HEADER + 9 - i]);
              timestamp_received |= (COM_RX_DATA[HEADER + 9 - i] << (8 * i));
            }
            set_time(timestamp_received);
          }
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
              char filename[7][30] = {"/flags.txt", "/satHealth.txt", "/satellite_Logs.txt", "/reservation_table.txt", "/cam_nir.txt", "/epdm.txt", "/adcs.txt"};

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
                __file_operations.mcu_id = 0xda;
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
                strcat(__file_operations.filepath, "/cam_rgb.txt");
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
              else if ((cmds[2] == 0xF8))
              {
                __file_operations.select_file = CAMERA_NIR_TXT;
                __file_operations.mcu_id = 0xca;
                strcat(__file_operations.filepath, "/cam_nir.txt");
              }
              else if ((cmds[2] == 0xF8))
              {
                __file_operations.select_file = DIGIPEATER_TXT;
                __file_operations.mcu_id = 0x0a;
                strcat(__file_operations.filepath, "/digipeater.txt");
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

              // uorb
              strcpy(command_uorb.path, __file_operations.filepath);
              command_uorb.command = __file_operations.cmd;
              command_uorb.num_of_packets = (uint16_t)__file_operations.number_of_packets[0] << 8 | __file_operations.number_of_packets[1];
              // command_uorb.packet_type = 0x00;
              command_uorb.address = (uint32_t)__file_operations.address[0] << 24 | __file_operations.address[1] << 16 | __file_operations.address[2] << 8 | __file_operations.address[3] & 0xff;
              command_uorb.executed = 0;
              MISSION_STATUS.FLASH_OPERATION = true;
              FLASH_OPERATION = true;
              flash_read_operation_uorb();//sends command using uorb 
              flash_operation_data(command_uorb.num_of_packets);
              
              MISSION_STATUS.FLASH_OPERATION = false;
              FLASH_OPERATION = false;
              // uorb
              //  perform_file_operations(&__file_operations);

              // uorb//

              // memset(command_uorb.data, '\0', sizeof(command_uorb.data));
              // int cmd_fd = orb_advertise(ORB_ID(command), &command_uorb);
              // int adc_instance = 1;
              // int cmd_fd = orb_advertise_multi_queue_persist(ORB_ID(command), &command_uorb,
              //                                                &adc_instance, sizeof(struct command));

              // if (cmd_fd >= 0)
              // {
              //   printf("THe command uorb advertise has been opened\n");
              //   int ret = orb_publish(ORB_ID(command), &command_uorb, sizeof(struct command));
              //   if (ret >= 0)
              //   {
              //     syslog(LOG_DEBUG, "THe data has been published");
              //   }
              // }
              // else
              // {
              //   syslog(LOG_ERR, "Raw ADC advertise failed. %i\n", cmd_fd);
              // }
              // orb_unadvertise(ORB_ID(command));
              // uorb//
            }

            /*
            Command for disabling status of KILL SWITCH
            */
            else if (cmds[0] == 0xee && cmds[1] == 0xaa && cmds[2] == 0xaa)
            {
              sleep(1);
              send_data_uart(COM_UART, ack, sizeof(ack));
              critic_flags.KILL_SWITCH_STAT = KILL_SW_OFF;
              store_flag_data(1, &critic_flags);
              gpio_write(GPIO_KILL_SW_EN, 0);
              // gpio_write(GPIO_)

              syslog(LOG_DEBUG, "--------- kill switch deactivated\n");
            }

            /*
            Command for enabling status of KILL SWITCH
            */
            else if (cmds[0] == 0xee && cmds[1] == 0xee && cmds[2] == 0xee)
            {
              sleep(1);
              send_data_uart(COM_UART, ack, sizeof(ack));
              gpio_write(GPIO_KILL_SW1_NEG, true);
              gpio_write(GPIO_KILL_SW1_POS, false);
              gpio_write(GPIO_KILL_SW_EN, true);

              gpio_write(GPIO_KILL_SW1_NEG, true);
              gpio_write(GPIO_KILL_SW1_POS, false);
              gpio_write(GPIO_KILL_SW_EN, true);

              critic_flags.KILL_SWITCH_STAT = KILL_SW_ON;
              syslog(LOG_DEBUG, "---------kill switch command received\n");

              if (0 == receive_command(&kill_sw))
              // store_flag_data(1,&critic_flags);
              {
                syslog(LOG_DEBUG, "---------kill switch activated\n");
              }
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
              gpio_write(GPIO_SFM_MODE, true);
              // turn_msn_on_off(2, 1);
              syslog(LOG_DEBUG, "------------------------  cam mission turned on (Command received from COM using RF)------------------\n");

              // sleep(1);
              send_data_uart(COM_UART, ack, sizeof(ack));
              // cam_operation();
              new_camera_operation();
              // int fd = open(CAM_UART, O_WRONLY); // Open in non-blocking mode
              // if (fd < 0)
              // {
              //   printf("Error opening %s\n", CAM_UART);
              //   usleep(PRINT_DELAY);
              //   return -1;
              // }
              // int r;
              // ret = read(fd, &r, sizeof(int));
              // if (ret > 0)
              // {
              //   printf("got mission off command\n");
              // }
              // close(fd);
              // turn_msn_on_off(2, 0);

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
      }
      else
      {
        struct reservation_command res;
        res.mcu_id = MCU_ID;
        for (int i = 0; i < 5; i++)
        {

          if (i < 3)
            res.cmd[i] = cmds[i];
          else
            res.time[i % 3] = cmds[i];
        }
        res.latest_time = 0x00;

        uint16_t t = 0x00;
        int adc_instance = 0;
        t = (uint16_t)cmds[3] << 8 | cmds[4];
        printf("Reservation command received with time:%d minutes", t);
        printf("The command is %02x %02x %02x\n", cmds[0], cmds[1], cmds[2]);
        int raw_afd = orb_advertise_multi_queue_persist(ORB_ID(reservation_command), &res,
                                                        &adc_instance, sizeof(struct reservation_command));

        if (raw_afd < 0)
        {
          syslog(LOG_ERR, "Raw ADC advertise failed. %i\n", raw_afd);
        }
        else
        {
          if (OK != orb_publish(ORB_ID(reservation_command), raw_afd, &res))
          {
            syslog(LOG_ERR, "Orb Publish failed\n");
          }
          else
          {
            syslog(LOG_DEBUG, "Reservation Command Orb Published\n");
            // sat_health.rsv_cmd+=1;
            sat_health.rsv_flag += 1;
            critic_flags.RSV_FLAG += 1;
            save_critics_flags(&critic_flags);
            // sort_reservation_command(1, false);

            // FIRST_RSV_CMD = true;
          }
        }
        ret = orb_unadvertise(raw_afd);
        if (ret < 0)
        {
          syslog(LOG_ERR, "adc raw Orb Unadvertise failed.\n");
        }
        // ORB_DECLARE(reservation_command);
        // ORB_DEFINE(reservation_command, struct reservation_command,print_satellite_health_data);
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
  gpio_write(GPIO_3V3_COM_EN, false); // Disable COM systems
  sleep(1);
  syslog(LOG_DEBUG, "***************************Turning on COM MSN...***************************\n");
  gpio_write(GPIO_3V3_COM_EN, 1);
  sleep(1);
  gpio_write(GPIO_3V3_COM_EN, true); // Enable COM systems

  // usleep(2000000);
  sleep(2);
  ret = handshake_COM(data); // tx rx data is flushed before closing the file
  // usleep(PRINT_DELAY * 100);
  sleep(1);
  if (ret == 0)
  {
    syslog(LOG_DEBUG, "Successful handshake with COM\n");
    COM_HANDSHAKE_STATUS = 1;
    uint64_t time1 = get_time_data();
    printf("\nThe received timestamp is : %d\n", time1);
    set_time(time1);
    // wdog_fd = open(DEVNAME, O_RDONLY);
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
    if (g_wdog_task_started)
    {
      printf("[Watchdog TASK] Task already started.\n");
      return EXIT_SUCCESS;
    }
    else
    {
      int retval = task_create("WATCHDOG_TASK", 100, 2896, watchdog_task, NULL);
      if (retval < 0)
      {
        printf("unable to create WATCHDOG_TASK task\n");
        for (int i = 0; i < 4; i++)
        {
          retval = task_create("WATCHDOG_TASK", 100, 4096, watchdog_task, NULL);
          if (retval >= 0)
          {
            if (retval < 0)
            {
              printf("unable to create WATCHDOG_TASK task\n");
            }
            return 0;

            break;
          }
        }
        return -1;
      }
      else
      {
        g_wdog_task_started = true;
      }
    }
    send_beacon_data();
    //  stm32_rtc_initialize();
  }
  struct reservation_command res;
  int fd_reservation = orb_subscribe(ORB_ID(reservation_command));
  uint32_t updated;
  uint32_t t2;

  time_t current_time;
  for (;;)
  {
    if (COM_HANDSHAKE_STATUS == 1)
    {
      if (sat_health.batt_volt > 370)
      {
        critic_flags.OPER_MODE = NRML_MODE;
      }
      if (sat_health.batt_volt <= 370)
      {
        critic_flags.OPER_MODE = SAFE_MODE;
      }
      receive_telecommand_rx(rx_data);
      get_top_rsv(&TO_EXECUTE);
      // pet_counter =0;
      // res.latest_time =10;
      // get_top_rsv(&res);

      handle_reservation_command(1, TO_EXECUTE);
    }
    // usleep(1000);
    sleep(10);
  }
}

void send_beacon(int argc, char *argv)
{
  int count_beacon = 0, fd_reservation, updated = 0, t2;
  struct reservation_command TO_EXECUTE, res;
  // int fd_reservation;
  fd_reservation = orb_subscribe(ORB_ID(reservation_command));

  for (;;)
  {
    if (COM_HANDSHAKE_STATUS == 1 && count_beacon % 88 == 0)
    {
      count_beacon = 0;
      read_int_adc1();
      read_int_adc3();
      //   // ads7953_receiver(argc, argv);

      make_satellite_health();
      print_satellite_health_data(&sat_health);
      send_beacon_data(); // TODO uncomment this
      pet_counter = 0;    // TODO remove this after uncommenting above
      count_beacon = 0;
    }
    count_beacon += 1;
    timer++;
    sleep(1); // 90 // TODO make it 90 later
    // usleep(100000);
  }
}

/****************************************************************************
 * COM TASK task
 *sub
 * COM will be in digipeater mode till a digipeating message is received and digipeated
 ****************************************************************************/
// void digipeater_mode(uint8_t *data) // TODO beacon 2 paxi disable garna milni enable garna namilni tara 2 agadi chai enable garna namilni
// {
//   receive_data_uart(COM_UART, data, 29);
//   for (int i = 29; i < 84; i++)
//   {
//     data[i] = 0xff;
//   }
//   // send_data_uart(COM_UART, data, 84);
//   /*To delete*/
//   if (send_data_uart(COM_UART, data, 84) > 0)
//   {
//     syslog(LOG_DEBUG, "***************************digipeating data is ***************************\n ");
//     for (int i = 0; i < 85; i++)
//       printf("%02x ", data);
//     printf("***************************digipeating successful***************************\n :");
//   }
//   /*To delete*/
// }

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
// strcpy(devpath, EPDM_UART);
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
  for (int i = 0; i < 7; i++)
  {
    ret = read(fd, &data1[i], 7);
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
  fd = open(devpath, O_RDONLY); // Open in non-blocking mode

  printf("6 bytes written\n");
  usleep(3000 * 1000); // 3 seconds delay

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
  // int loop = 0;
  // while (1)
  // {
  //   ret = read(fd, &data1[i], 1);
  //   if (ret < 0)
  //   {
  //     if (errno == EAGAIN)
  //     {
  //       // Uncomment for debugging
  //       // printf("No data available for reading\n");
  //       usleep(50); // usleep(5000); // Increase the sleep time to 500 ms
  //       i--;        // Retry reading the same index
  //       continue;
  //     }
  //     printf("Error reading from %s\n", devpath);
  //     close(fd);
  //     return -1;
  //   }
  //   loop++;
  //   // If we successfully read a character, check if we have more data
  //   if (ret == 1)
  //   {
  //     // Optionally print the character read for debugging
  //     printf("Read character: %c\n", data1[i]);
  //     break; // Exit the loop if a character was successfully read
  //   }
  // }
  ret = read(fd, ack, sizeof(ack));
  printf("Data received from %s:\n", devpath);
  usleep(PRINT_DELAY);
  for (int j = 0; j < 7; j++)
  {
    printf("%02X ", data1[j]);
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
  fd = open_uart(COM_UART); // open(dev_path, O_RDONLY);
  if (fd < 0)
  {
    printf("Unable to open %s\n", dev_path);
    return fd;
  }

  printf("\n---------Satellite waiting for command-------------------\nsize of data to receive: %d\n", size);
  // ret = read(fd, data, sizeof(data)); // try receiving data without byte by byte method
  for (int i = 0; i < size; i++)
  {
    ret = read(fd, &data[i], 1);
  }
  if (ret < 0)
  {
    printf("Data not received from %s\n", dev_path);
    return ret;
  }
  else
  {
    if (data[0] == 0x42 && data[1] == 0xac && data[2] == 0x04 && data[3] == 0x02 && data[4] == 0x4d)
    {
      pet_counter = 0;
      printf("\nACK received from COM(Data received by COM)\n");
    }
    else
    {
      printf("Command received from %s\n", dev_path);
      for (int i = 0; i < size; i++)
      {
        printf("%x ", data[i]);
      }
    }
  }
  printf("\n");
  ioctl(fd, TCFLSH, 2);
  ioctl(fd, TCDRN, NULL);
  printf("\ndrained and flushed tx rx buffer\n");
  // if (close(fd) < 0)
  // {
  //   if (close(fd) < 0)
  //   {
  //     printf("Failed to close COM UART: %s\n", strerror(errno));
  //   }
  // }
  close_uart(fd);
  usleep(10000);
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
    beacon_data[i] = 0x00;
    // }
    // beacon_data[84] = s2s_beacon_type_a.;
    // beacon_data[85] = s2s_beacon_type_a.;
  }
  // uint8_t beacon_data[BEACON_DATA_SIZE];
  beacon_data[0] = s2s_beacon_type_a.HEAD;
  beacon_data[1] = s2s_beacon_type_a.TYPE << 4 & s2s_beacon_type_a.TIM_DAY << 4 & 0xff;
  beacon_data[2] = (uint8_t)s2s_beacon_type_a.TIM_DAY & 0xff;
  beacon_data[4] = s2s_beacon_type_a.TIM_HOUR;

  beacon_data[3] = 0x01;

  beacon_data[1 + 4] = (s2s_beacon_type_a.BAT_V >> 8) & 0Xff;
  beacon_data[1 + 5] = s2s_beacon_type_a.BAT_V & 0xff;
  beacon_data[1 + 6] = (s2s_beacon_type_a.BAT_C >> 8) & 0Xff;
  beacon_data[1 + 7] = (s2s_beacon_type_a.BAT_C) & 0Xff;
  beacon_data[1 + 8] = (s2s_beacon_type_a.BAT_T >> 8) & 0Xff;
  beacon_data[1 + 9] = (s2s_beacon_type_a.BAT_T) & 0Xff;

  beacon_data[1 + 10] = s2s_beacon_type_a.RAW_C;
  beacon_data[1 + 11] = (s2s_beacon_type_a.SOL_TOT_V >> 8) & 0Xff;
  beacon_data[1 + 12] = (s2s_beacon_type_a.SOL_TOT_V) & 0Xff;
  beacon_data[1 + 13] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[1 + 14] = (s2s_beacon_type_a.SOL_TOT_C >> 8) & 0Xff;
  beacon_data[1 + 15] = s2s_beacon_type_a.ANT_P_T;
  beacon_data[1 + 16] = s2s_beacon_type_a.BPB_T;
  beacon_data[1 + 17] = s2s_beacon_type_a.OBC_T;
  beacon_data[1 + 18] = s2s_beacon_type_a.X_T;
  beacon_data[1 + 19] = s2s_beacon_type_a.X1_T;
  beacon_data[1 + 20] = s2s_beacon_type_a.Y_T;
  beacon_data[1 + 21] = s2s_beacon_type_a.Y1_T;
  beacon_data[1 + 22] = sat_health.sol_p5_v;

  beacon_data[1 + 23] = s2s_beacon_type_a.SOL_P1_STAT << 7 & s2s_beacon_type_a.SOL_P2_STAT << 6 & s2s_beacon_type_a.SOL_P3_STAT << 5 & s2s_beacon_type_a.SOL_P4_STAT << 4 & s2s_beacon_type_a.MSN1_STAT << 3 & s2s_beacon_type_a.MSN2_STAT << 2 & s2s_beacon_type_a.MSN3_STAT << 1 & 0xff;
  beacon_data[1 + 24] = s2s_beacon_type_a.ANT_STAT << 4 & s2s_beacon_type_a.UL_STAT << 4;
  beacon_data[1 + 25] = s2s_beacon_type_a.OPER_MODE;
  beacon_data[1 + 26] = (s2s_beacon_type_a.OBC_RESET_COUNT >> 8) & 0xff;
  beacon_data[1 + 27] = s2s_beacon_type_a.OBC_RESET_COUNT & 0xff;
  beacon_data[1 + 28] = s2s_beacon_type_a.RST_RESET_COUNT >> 8 & 0xff; // TODO no reset mcu so no count needed
  beacon_data[1 + 29] = s2s_beacon_type_a.RST_RESET_COUNT & 0xff;
  // beacon_data[1 + 30] = s2s_beacon_type_a.LAST_RESET;
  beacon_data[1 + 30] = s2s_beacon_type_a.CHK_CRC;
}
// COM_APP

void serialize_beacon_b(uint8_t beacon_data[BEACON_DATA_SIZE])
{
  for (int i = 0; i <= BEACON_DATA_SIZE; i++)
  {
    beacon_data[i] = 0x00;
  }
  // uint8_t beacon_data[BEACON_DATA_SIZE];
  beacon_data[0] = s2s_beacon_type_b.HEAD;
  beacon_data[1] = s2s_beacon_type_b.TYPE;
  beacon_data[2] = s2s_beacon_type_b.TIM_DAY;

  beacon_data[3] = 0x02;

  beacon_data[1 + 3] = s2s_beacon_type_b.SOL_P1_V;
  beacon_data[1 + 4] = s2s_beacon_type_b.SOL_P2_V;
  beacon_data[1 + 5] = s2s_beacon_type_b.SOL_P3_V;
  beacon_data[1 + 6] = s2s_beacon_type_b.SOL_P4_V;
  beacon_data[1 + 7] = s2s_beacon_type_b.SOL_P5_V; // panel 5

  beacon_data[1 + 8] = s2s_beacon_type_b.SOL_P1_C;
  beacon_data[1 + 9] = s2s_beacon_type_b.SOL_P2_C;
  beacon_data[1 + 10] = s2s_beacon_type_b.SOL_P3_C;
  beacon_data[1 + 11] = s2s_beacon_type_b.SOL_P4_C;
  beacon_data[1 + 12] = s2s_beacon_type_b.SOL_P5_C;

  beacon_data[1 + 13] = ((s2s_beacon_type_b.GYRO_X >> 8) & 0xff) * 100;
  beacon_data[1 + 14] = ((s2s_beacon_type_b.GYRO_X) & 0xff) * 100;
  beacon_data[1 + 15] = ((s2s_beacon_type_b.GYRO_Y >> 8) & 0xff) * 100;
  beacon_data[1 + 16] = ((s2s_beacon_type_b.GYRO_Y) & 0xff) * 100;
  beacon_data[1 + 17] = (s2s_beacon_type_b.GYRO_Z >> 8 & 0xff) * 100;
  beacon_data[1 + 18] = (s2s_beacon_type_b.GYRO_Z & 0xff) * 100;

  beacon_data[1 + 19] = ((s2s_beacon_type_b.ACCL_X >> 8) & 0xff) * 100;
  beacon_data[1 + 20] = ((s2s_beacon_type_b.ACCL_X) & 0xff) * 100;
  beacon_data[1 + 21] = ((s2s_beacon_type_b.ACCL_Y >> 8) & 0xff) * 100;
  beacon_data[1 + 22] = ((s2s_beacon_type_b.ACCL_Y) & 0xff) * 100;
  beacon_data[1 + 23] = (s2s_beacon_type_b.ACCL_Z >> 8 & 0xff) * 100;
  beacon_data[1 + 24] = (s2s_beacon_type_b.ACCL_Z & 0xff) * 100;

  beacon_data[1 + 25] = ((s2s_beacon_type_b.MAG_X >> 8) & 0xff) * 100;
  beacon_data[1 + 26] = ((s2s_beacon_type_b.MAG_X) & 0xff) * 100;
  beacon_data[1 + 27] = ((s2s_beacon_type_b.MAG_Y >> 8) & 0xff) * 100;
  beacon_data[1 + 28] = ((s2s_beacon_type_b.MAG_Y) & 0xff) * 100;
  beacon_data[1 + 29] = (s2s_beacon_type_b.MAG_Z >> 8 & 0xff) * 100;
  beacon_data[1 + 30] = (s2s_beacon_type_b.MAG_Z & 0xff) * 100;
  beacon_data[1 + 31] = s2s_beacon_type_b.CHK_CRC; // TODO::  last rst
}

// COM_APP
void Make_Beacon_Data(uint8_t type)
{
  // switch (type)
  {
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    int16_t hour = tm_info->tm_hour;
    int16_t minute = tm_info->tm_min;
    int16_t second = tm_info->tm_sec;

    int16_t year = tm_info->tm_year + 1900; // Year since 1900
    int16_t month = tm_info->tm_mon + 1;    // Months since January
    int16_t day = tm_info->tm_mday;
    // case 1:
    s2s_beacon_type_a.HEAD = 0x53;
    s2s_beacon_type_a.TYPE = 0x00;
    s2s_beacon_type_a.TIM_DAY = day;
    s2s_beacon_type_a.TIM_HOUR = hour;
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

    s2s_beacon_type_a.SOL_P1_STAT = sat_health.sol_p1_v >= 1000 ? 1 : 0; // check from power, max power draw is 0.6 watt
    s2s_beacon_type_a.SOL_P2_STAT = sat_health.sol_p2_v >= 1000 ? 1 : 0;
    s2s_beacon_type_a.SOL_P3_STAT = sat_health.sol_p3_v >= 1000 ? 1 : 0;
    s2s_beacon_type_a.SOL_P4_STAT = sat_health.sol_p4_v >= 1000 ? 1 : 0;
    s2s_beacon_type_a.SOL_P5_STAT = sat_health.sol_p5_v >= 1000 ? 1 : 0;

    s2s_beacon_type_a.ANT_STAT = critic_flags.ANT_DEP_STAT;
    s2s_beacon_type_a.KILL1_STAT = critic_flags.KILL_SWITCH_STAT;
    s2s_beacon_type_a.KILL2_STAT = critic_flags.KILL_SWITCH_STAT;
    s2s_beacon_type_a.UL_STAT = critic_flags.UL_STATE;

    // break;

    s2s_beacon_type_a.OBC_RESET_COUNT = critic_flags.RST_COUNT; // TODO
    s2s_beacon_type_a.LAST_RESET = 0xff;                        // TODO
    // s2s_beacon_type_a.CHK_CRC = ;          //TODO

    // case 2:
    s2s_beacon_type_b.HEAD = 0x53;
    s2s_beacon_type_b.TYPE = 0x01;
    s2s_beacon_type_b.TIM_DAY = day;

    s2s_beacon_type_b.SOL_P1_V = sat_health.sol_p1_v;
    s2s_beacon_type_b.SOL_P2_V = sat_health.sol_p2_v;
    s2s_beacon_type_b.SOL_P3_V = sat_health.sol_p3_v;
    s2s_beacon_type_b.SOL_P4_V = sat_health.sol_p4_v;
    s2s_beacon_type_b.SOL_P5_V = sat_health.sol_p5_v;

    s2s_beacon_type_b.SOL_P1_C = sat_health.sol_p1_c;
    s2s_beacon_type_b.SOL_P2_C = sat_health.sol_p2_c;
    s2s_beacon_type_b.SOL_P3_C = sat_health.sol_p3_c;
    s2s_beacon_type_b.SOL_P4_C = sat_health.sol_p4_c;
    s2s_beacon_type_b.SOL_P5_C = sat_health.sol_p5_c;

    s2s_beacon_type_b.MAG_X = (int16_t)(sat_health.mag_x);
    s2s_beacon_type_b.MAG_Y = (int16_t)(sat_health.mag_y);
    s2s_beacon_type_b.MAG_Z = (int16_t)(sat_health.mag_z);

    s2s_beacon_type_b.GYRO_X = (int16_t)(sat_health.gyro_x);
    s2s_beacon_type_b.GYRO_Y = (int16_t)(sat_health.gyro_y);
    s2s_beacon_type_b.GYRO_Z = (int16_t)(sat_health.gyro_z);

    s2s_beacon_type_b.ACCL_X = (int16_t)(sat_health.accl_x);
    s2s_beacon_type_b.ACCL_Y = (int16_t)(sat_health.accl_y);
    s2s_beacon_type_b.ACCL_Z = (int16_t)(sat_health.accl_z);

    sleep(1);
    // break;

    // s2s_beacon_type_b.CHK_CRC = ;  //TODO
  }
}

/****************************************************************************
 * Send data from UART through any UART path
 ****************************************************************************/
// int send_data_uart(char *dev_path, uint8_t *data, uint16_t size)
// {
//   if (COM_UART_BUSY == false)
//   {
//     COM_UART_BUSY = true;
//     double fd;
//     int i;
//     int count = 0, ret;
//     int wr1;

//     fd = open(dev_path, O_WRONLY);

//     if (fd < 0)
//     {
//       printf("error opening %s\n", dev_path);
//       return fd;
//     }
//     else
//     {
//       printf("Turning on  4V dcdc line..\n");
//       gpio_write(GPIO_DCDC_4V_EN, 1);
//       printf("Turning on COM 4V line..\n");
//       gpio_write(GPIO_COM_4V_EN, 1);
//       printf("Opening uart dev path : %s\n", dev_path);
//       wr1 = write(fd, data, size);
//       if (wr1 < 0)
//       {
//         printf("Unable to write data\n");
//         return wr1;
//       }
//       printf("\n%d bytes written\n", wr1);
//       // printf("\ndata is %\n", wr1);
//       for (int i = 0; i < size; i++)
//       {
//         {
//           printf("%d ", data[i]);
//         }
//       }
//       sleep(2);
//       printf("Turning off  4v DCDC line..\n");
//       int x = 0;
//       while (x < 200000)
//       {
//         x += 200;
//         usleep(200);
//       }

//       gpio_write(GPIO_DCDC_4V_EN, 0);
//       // printf("Turning off COM 4V line..\n");
//       gpio_write(GPIO_COM_4V_EN, 0);
//       ioctl(fd, TCFLSH, 2);
//       printf("flused tx rx buffer\n");
//       ioctl(fd, TCDRN, NULL);
//       printf("drained tx rx buffer\n");
//       if (close(fd) < 0)
//       {
//         close(fd);
//         printf("Failed to close COM_UART: %s\n", strerror(errno));
//       }
//     }
//     COM_UART_BUSY = false;

//     return wr1;
//   }
// }

pthread_mutex_t uart_mutex = PTHREAD_MUTEX_INITIALIZER;

int open_uart(const char *uart_dev)
{
  pthread_mutex_lock(&uart_mutex);
  int fd = open(uart_dev, O_RDWR);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Failed to open UART %s: %d", uart_dev, errno);
  }
  pthread_mutex_unlock(&uart_mutex);
  return fd;
}

void close_uart(int fd)
{
  pthread_mutex_lock(&uart_mutex);
  close(fd);
  pthread_mutex_unlock(&uart_mutex);
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
      int fd2 = file_open(&fl1, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_CREAT | O_RDWR);
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
      if (size_of_file > 0) //& size_of_file > address + 80
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
            printf("%02x ", data_retrieved[j]); // Print in hexadecimal format
          }
          loop1 += 1;
          flash_data[0] = 0x53;
          flash_data[1] = file_operations->mcu_id; // 0xac // file_operations->mcu_id;//TODO check out mcu_id value and set this one
          if (size_of_file > address + 80)
          {
            flash_data[2] = 0x51;
            flash_data[3] = loop1;
            flash_data[BEACON_DATA_SIZE - 2] = 0x7e;
            flash_data[BEACON_DATA_SIZE - 1] = '\0';
          }
          else
          {
            flash_data[2] = size_of_file;
            flash_data[size_of_file - 2] = 0x7e;
            flash_data[size_of_file - 1] = '\0';
          }

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
    printf("Turning on  4v dcdc line..\n");
    gpio_write(GPIO_DCDC_4V_EN, 1);
    printf("Turning on COM 4V line..\n");
    gpio_write(GPIO_COM_4V_EN, 1);
    download_file_from_flash(file_operations, data_retrieved, SIZE_OF_DATA_DOWNLOAD);
    printf("*****Download command received**************\nsize:%d\n***********************\ncmd : %d, select_file:%d, select_flash: %d, rsv_table:%d, filepath:%s,address :%d %d %d %d, number_of packets:%d %d\n",
           sizeof(data_retrieved), file_operations->cmd, file_operations->select_flash, file_operations->select_file, file_operations->rsv_table[1], file_operations->rsv_table[0], file_operations->filepath,
           file_operations->address[3], file_operations->address[2], file_operations->address[1], file_operations->address[0],
           file_operations->number_of_packets[3], file_operations->number_of_packets[2], file_operations->number_of_packets[1], file_operations->number_of_packets[0]);

    printf("-------Data download function has been called\n");
    printf("Turning off  4v dcdc line..\n");
    gpio_write(GPIO_DCDC_4V_EN, 0);
    printf("Turning off COM 4V line..\n");
    gpio_write(GPIO_COM_4V_EN, 0);
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
  printf("******Global reset called******");
  for (;;)
  {
    // sleep(300);
    sleep(75400);
    critic_flags.RST_COUNT = critic_flags.RST_COUNT + 1;
    // store_flag_data(1,&critic_flags);
    save_critics_flags(&critic_flags);
    print_critical_flag_data(&critic_flags);
    sleep(10000);
    gpio_write(GPIO_GBL_RST, true);
  }
}

int create_task(const char *name, int priority, int stack_size, main_t entry)
{
  const int MAX_RETRY_COUNT = 4;

  int retval;
  for (int i = 0; i < MAX_RETRY_COUNT; i++)
  {
    retval = task_create(name, priority, stack_size, entry, NULL);
    if (retval >= 0)
    {
      return retval;
    }
    printf("Failed to create %s task. Retry %d/%d\n", name, i + 1, MAX_RETRY_COUNT);
    sleep(1); // Wait before retrying
  }
  return -1;
}

/****************************************************************************
 * Name: main
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  int hand = 5;
  bool g_mpu_task_started = false;

  // watchdog code
  //  {

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
  else if (strcmp(argv[1], "truncate") == 0)
  {
    int fd;
    struct file file_pointer, file_pointer2;
    // fd = file_open(&file_pointer, "/mnt/fs/sfm/mtd_mission/cam_rgb.txt", O_TRUNC);
    uint16_t counter = file_seek(&file_pointer, 0, SEEK_END);
    // file_open(&file_pointer2, "/mnt/fs/sfm/mtd_mission/cam_nir.txt", O_TRUNC);
    // file_close(&file_pointer2);
    // file_close(&file_pointer);
    // fd = file_open(&file_pointer, "/mnt/fs/mfm/mtd_mission/cam_rgb.txt", O_TRUNC);
    // counter = file_seek(&file_pointer, 0, SEEK_END);
    // file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mission/cam_nir.txt", O_TRUNC);
    // file_close(&file_pointer2);
    // fd = file_open(&file_pointer, "/mnt/fs/sfm/mtd_mission/cam_rgb.txt", O_TRUNC);
    // counter = file_seek(&file_pointer, 0, SEEK_END);
    // file_open(&file_pointer2, "/mnt/fs/sfm/mtd_mission/cam_nir.txt", O_TRUNC);
    // file_close(&file_pointer2);

    // file_close(&file_pointer);
    file_open(&file_pointer2, "/mnt/fs/sfm/mtd_mainstorage/test.txt", O_TRUNC);
    file_close(&file_pointer2);
    file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mainstorage/test.txt", O_TRUNC);
    file_close(&file_pointer2);
    file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mission/test.txt", O_TRUNC);
    file_close(&file_pointer2);
    file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_TRUNC);
    file_close(&file_pointer2);
    //
    // file_open(&file_pointer2, "/mnt/fs/sfm/mtd_mainstorage/epdm.txt", O_TRUNC);
    // file_close(&file_pointer2);
    // file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mission/epdm.txt", O_TRUNC);
    // file_close(&file_pointer2);
    // file_open(&file_pointer2, "/mnt/fs/sfm/mtd_mission/epdm.txt", O_TRUNC);
    // file_close(&file_pointer2);
    // file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mainstorage/epdm.txt", O_TRUNC);
    // file_close(&file_pointer2);
    // file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mainstorage/sat_health.txt", O_TRUNC);
    // file_close(&file_pointer2);
    // file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mainstorage/satHealth.txt", O_TRUNC);
    // file_close(&file_pointer2);

    file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mainstorage/reservation_command.txt", O_TRUNC);
    file_close(&file_pointer2);
    // file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mainstorage/satHealth.txt", O_TRUNC);
    // file_close(&file_pointer2);
  }
  else if (strcmp(argv[1], "sfm") == 0)
  {
    // save_64_bit();

    // int r = atoi(argv[2]);
    // printf("The data is %d\n",r);
    // clear_int_flag();

    // flash_read_operation_uorb();
    // set_time(1733986468);
    // save_64_bit();
    // struct file fl1;
    // // struct SEEK_POINTER seek_pointer;
    // uint16_t seek_pointer[8];
    // for(int i=0;i<8;i++){
    //   seek_pointer[i] = i*3000;
    // }
    //  int fd2 = file_open(&fl1, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_CREAT | O_RDWR);
    //   if (fd2 >= 0)
    //   {
    //     ssize_t t =file_write(&fl1, &seek_pointer,sizeof(seek_pointer));
    //     printf("FIle size %d\n",t);
    //   }
    //   file_close(&fl1);
    //   fd2 = file_open(&fl1, "/mnt/fs/mfm/mtd_mainstorage/seek_pointer.txt", O_CREAT | O_RDWR);
    //   if (fd2 >= 0)
    //   {
    //     // file_write(&fl1, seek_pointer,sizeof(seek_pointer));
    //     ssize_t readBytes2 = file_read(&fl1, seek_pointer, sizeof(seek_pointer));
    //     if (readBytes2 < 0)
    //     {
    //       syslog(LOG_SYSLOG, "Error while reading the seek_pointer.txt in mfm\n");
    //       file_close(&fl1);
    //       readBytes2 = 0;

    //     }
    //     else{
    //       for(int i=0;i< 8;i++){
    //         printf("%d \n",seek_pointer[i]);
    //       }
    //     }
    //   }
    // new_camera_operation();
    // int retval = task_create("Camera_TASK_APP", 100, 30048, new_camera_operation, NULL);
  }

  // else if (strcmp(argv[1], "sfm1") == 0)
  // {
  //   gpio_write(GPIO_SFM_MODE, false);
  //   uint8_t ttt[6];
  //   int fd2 = open("/dev/ttyS5", O_RDWR);
  //   if (fd2 >= 0)
  //   {
  //     printf("UART successfully opened\n");
  //     write(fd2, "test", 4);
  //     read(fd2, ttt, 6);
  //     close(fd2);
  //   }
  //   // write(fd2,"testing",7);
  //   else
  //   {
  //     printf("UART driver failed\n");
  //   }
  //   // gpio_write(GPIO_MSN2_EN);
  //   // turn_msn_on_off(2, 0);
  // }
  // else if (strcmp(argv[1], "com") == 0)
  // {
  //   int retval = task_create("COMMANDER_TASK_APP", 100, 4096, COM_TASK, NULL);
  //   if (retval < 0)
  //   {
  //     printf("unable to create MPU6500_TASK_APP task\n");
  //     for (int i = 0; i < 4; i++)
  //     {
  //       retval = task_create("MPU6500_TASK_APP", 100, 9600, COM_TASK, NULL);
  //       if (retval >= 0)
  //       {
  //         g_mpu_task_started = true;
  //         break;
  //         return 0;
  //       }
  //     }
  //     return -1;
  //   }
  // }
  else if (strcmp(argv[1], "ext_flash_manual") == 0)
  {
    CRITICAL_FLAGS mfm_flags = {0xff};
    mfm_flags.ANT_DEP_STAT = DEPLOYED;
    mfm_flags.RST_COUNT = 0x00;
    mfm_flags.UL_STATE = UL_NOT_RX;
    store_flag_data(1, &mfm_flags);
  }
  else if (strcmp(argv[1], "internal") == 0)
  {
    CRITICAL_FLAGS rd_flags_int = {0x00};
    // check_flag_data();
    struct file truncate_ptr;
    int fd;
    char path1 = "/mnt/fs/mfm/mtd_mainstorage/flags.txt";
    fd = file_open(&truncate_ptr, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_TRUNC);
    if (fd >= 0)
    {
      syslog(LOG_SYSLOG, "File named %s has been truncated successfully.\n", path1);
    }
    else
    {
      syslog(LOG_SYSLOG, "Error opening file: %s\n", path1);
    }
    file_close(&truncate_ptr);
    if (close(fd) < 0)
    {
      close(fd);
      printf("Failed to close COM UART: %s\n", strerror(errno));
    }
    return 0;
  }
  // else if (strcmp(argv[1], "mpu") == 0)
  // {
  //   // printf("Starting MPU6500 data reader...\n");
  //   // read_mpu6500_data();
  //   if (g_mpu_task_started)
  //   {
  //     printf("[MPU6500 TASK] Task already started.\n");
  //     return EXIT_SUCCESS;
  //   }
  //   else
  //   {
  //     printf("[MPU6500 TASK] Task  started.\n");
  //     // subscribe_and_retrieve_data(mag_scaled)
  //     int retval = task_create("MPU6500_TASK_APP", 100, 2048, subscribe_and_retrieve_data, NULL);
  //     if (retval < 0)
  //     {
  //       printf("unable to create MPU6500_TASK_APP task\n");
  //       for (int i = 0; i < 4; i++)
  //       {
  //         retval = task_create("MPU6500_TASK_APP", 100, 2600, subscribe_and_retrieve_data, NULL);
  //         if (retval >= 0)
  //         {
  //           g_mpu_task_started = true;
  //           break;
  //           return 0;
  //         }
  //       }
  //       return -1;
  //     }
  //     else
  //     {
  //       return 0;
  //     }
  //   }
  //   // ads7953_receiver(argc, argv);
  //   // print_satellite_health_data(&sat_health);

  //   sleep(1);
  // }
  // else if (strcmp(argv[1], "camera") == 0)
  // {
  //   struct file cam;
  //   uint8_t cam_data[3500];
  //   MISSION_STATUS.FLASH_OPERATION = true;
  //   printf("_______________________________________________________\n");

  //   int cam_fd = open_file_flash(&cam, MFM_MAIN_STRPATH, "/cam.txt", O_RDONLY);
  //   int ccc = file_seek(&cam, 0, SEEK_END);
  //   int read_files = file_read(&cam, cam_data, ccc + 1);
  //   if (read_files >= 0)
  //   {
  //     for (int i = 0; i < ccc + 1; i++)
  //     {
  //       printf("%02x ", cam_data[i]);
  //     }
  //     printf("_______________________________________________________\n");
  //   }
  //   file_close(&cam);
  //   MISSION_STATUS.FLASH_OPERATION = false;
  // }

  else if (strcmp(argv[1], "int") == 0)
  {
    // for (int i = 0; i < 20; i++)
    while (1)
    {
      read_int_adc1();
      // read_int_adc3();
      make_satellite_health();
      // bool g_adc_task_started = false;
      // if (g_adc_task_started)
      // {
      //   printf("[Reset TASK] Task already started.\n");
      //   return EXIT_SUCCESS;
      // }
      // else
      // {
      //   printf("[ADC TASK] Task  started.\n");
      //   int retval = task_create("ADC_TASK_APP", 100, 1400, ads7953_receiver, NULL);
      //   if (retval < 0)
      //   {
      //     printf("unable to create ADC_TASK_APP task\n");
      //     for (int i = 0; i < 4; i++)
      //     {
      //       retval = task_create("ADC_TASK_APP", 100, 1600, ads7953_receiver, NULL);
      //       if (retval >= 0)
      //       {
      //         g_adc_task_started = true;
      //         break;
      //         return 0;
      //       }
      //     }
      //     return -1;
      //   }
      // }
      sleep(1);
    }
  }
  else if (strcmp(argv[1], "ant") == 0)
  {
    Antenna_Deployment(argc, argv);
    // RUN_ADC();
  }
  // else if (strcmp(argv[1], "epdm") == 0)
  // {
  //   epdm_operation();
  // }
  // else if (strcmp(argv[1], "cam") == 0)
  // {
  //   // cam_operation();
  //   new_camera_operation();
  // }
  // else if (strcmp(argv[1], "adcs") == 0)
  // {
  //   adcs_operation(0x01);
  // }

  /*TODO : REMOVE LATER Independent testing*/
  else
  {
    if (load_critics_flags(&critic_flags) != 0)
    {
      // Handle error or initialize flags
      memset(&critic_flags, 0, sizeof(CRITICAL_FLAGS));
    }
    if (critic_flags.ANT_DEP_STAT != DEPLOYED)
    {
      printf("antenna not deployed\n");
    }
    else
    {
      printf("antenna already deployed\n");
    }
    printf("************************************************\n");
    printf("***********S2S commander app************\n");

    printf("********ANtenna deployement starting************\n");

    Antenna_Deployment(argc, argv);
    print_critical_flag_data(&critic_flags);
    // sleep(10);
    if (critic_flags.ANT_DEP_STAT != DEPLOYED && critic_flags.UL_STATE != UL_RX)
    {
      Antenna_Deployment(argc, argv);
    }
    else
    {

      // printf("************************************************\n");
      // printf("***********Waiting 24 hours************\n");
      // printf("********ANtenna deployement starting************\n");

      // Antenna_Deployment(argc, argv);
      // if (critic_flags.ANT_DEP_STAT != DEPLOYED && critic_flags.UL_STATE != UL_RX)
      // {
      //   printf("Antenna still undeployed. Waiting for next 24 hours\n");
      //   int count = 0;
      //   do
      //   {
      //     printf("%d hours passed.\n", count);
      //     sleep(3600);
      //     count += 1;
      //   } while (count < 24);
      //   Antenna_Deployment(argc, argv);
      // }
      // else
      {
        bool g_watchdog_task_started = false;
        bool g_reset_task_started = false;

        if (g_watchdog_task_started)
        {
          printf("[WDG TASK] Task already started.\n");
        }
        else
        {
          int retval = create_task("RESET_TASK_APP", 100, 1500, global_reset);
          if (retval >= 0)
          {
            g_reset_task_started = true;
            printf("[Reset TASK] Task started.\n");
          }
          else
          {
            printf("Unable to create RESET_TASK_APP task\n");
          }
        }

        if (g_commander_task_started)
        {
          printf("[COMMANDER TASK] Task already started.\n");
        }
        else
        {
          int retval = task_create("COMMANDER_TASK_APP", 100, 56096, COM_TASK, NULL);

          // int retval = task_create("ADC_TASK_APP", 100, 12, ads7953_receiver, NULL);
          // int retval = create_task("COMMANDER_TASK_APP", 100, 10096, COM_TASK);
          if (retval >= 0)
          {
            g_commander_task_started = true;
          }
          else
          {
            printf("Unable to create COMMANDER_TASK_APP task\n");
          }
        }

        if (g_beacon_task_started)
        {
          printf("[BEACON TASK] Task already started.\n");
        }
        else
        {
          int retval = create_task("BEACON_TASK_APP", 100, 3800, send_beacon);
          if (retval >= 0)
          {
            g_beacon_task_started = true;
            // retval = create_task("UORB_TASK_APP", 100, 3800, flash_operation_data);
            // if (retval >= 0)
            // {
            //   g_beacon_task_started = true;
            // }
          }
          else
          {
            printf("Unable to create BEACON_TASK_APP task\n");
          }
        }

        if (g_mpu_task_started)
        {
          printf("[MPU6500 TASK] Task already started.\n");
        }
        else
        {
          int retval = create_task("MPU6500_TASK_APP", 100, 2048, subscribe_and_retrieve_data);
          if (retval >= 0)
          {
            g_mpu_task_started = true;
          }
          else
          {
            printf("Unable to create MPU6500_TASK_APP task\n");
          }
        }

        bool g_adc_task_started = false;
        if (g_adc_task_started)
        {
          printf("[ADC TASK] Task already started.\n");
        }
        else
        {
          int retval = create_task("ADC_TASK_APP", 100, 2848, ads7953_receiver);
          if (retval >= 0)
          {
            g_adc_task_started = true;
          }
          else
          {
            printf("Unable to create ADC_TASK_APP task\n");
          }
        }
        printf("************************************************\n");
      }
    }
  }
  return 0;
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

  gpio_write(GPIO_SFM_MODE, state);
}
// #include

// //COM
void send_flash_data(uint8_t *beacon_data)
{
  printf("Turning on 4v dcdc line..\n");
  gpio_write(GPIO_DCDC_4V_EN, 1);
  printf("Turning on COM 4V line..\n");
  gpio_write(GPIO_COM_4V_EN, 1);
  int fd = open_uart(COM_UART); // open(COM_UART, O_RDWR);
  int ret2;
  if (fd < 0)
  {
    printf("unable to open: %s\n", COM_UART);
    return -1;
  }
  else
  {
    sleep(2);

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
      pet_counter = 0; // TODO rethink on this later internal wdog

      if (ret2 >= 0)
      {
        printf("Flash data sent to COM successfully\n");
        printf("data is:::::::: %s\n", beacon_data);
        for (int i = 0; i < BEACON_DATA_SIZE; i++)
        {
          // ret = write(fd, &beacon_data[i], 1);
          printf("%02x ", beacon_data[i]);
          // usleep(1000);
        }
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
  sleep(1);
  printf("Turning off 4v dcdc line..\n");
  gpio_write(GPIO_DCDC_4V_EN, 0);
  printf("Turning off COM 4V line..\n");
  gpio_write(GPIO_COM_4V_EN, 0);
  close_uart(fd);
}

// COM
/****************************************************************************
 * Send beacon data to COM
 * To be done every 90 seconds
 ****************************************************************************/
int send_beacon_data()
{
  if (MISSION_STATUS.FLASH_OPERATION == false && MISSION_STATUS.ADCS_MISSION == false && MISSION_STATUS.CAM_MISSION == false && MISSION_STATUS.EPDM_MISSION == false)
  {

    if (COM_BUSY == 1)
    {
      return -1;
    }
    else
    {
      uint8_t beacon_data[BEACON_DATA_SIZE];
      Make_Beacon_Data(beacon_type);
      switch (beacon_type)
      {
      case 0:

        serialize_beacon_a(beacon_data);
        beacon_data[1] = 0xb1;
        beacon_data[2] = 0x21;
        beacon_data[83] = 0x7e;
        print_beacon_a();
        break;
      case 1:
        serialize_beacon_b(beacon_data);
        beacon_data[1] = 0xb2;
        beacon_data[2] = 0x21;
        print_beacon_b();
        break;
      default:
        printf("wrong case selected\n");
        return -1;
        break;
      }
      beacon_data[0] = 0x53;
      beacon_data[84] = 0x7e;

      beacon_data[85] = '\0';
      int fd; //
      // fd= send_data_uart(COM_UART, beacon_data, sizeof(beacon_data));
      //  fd = send_data_uart(COM_UART, test, sizeof(test));

      printf("beacon data size %d\n", sizeof(beacon_data));
      send_data_uart(COM_UART, beacon_data, sizeof(beacon_data));
      uint8_t x[43], ret2;
      save_64_bit();
      // ret2 = receive_data_uart(COM_UART, x,sizeof(x));
      // if(ret2 < 0){

      // }
      // fd = open(COM_UART, O_WRONLY);
      // int count;
      // if (fd < 0)
      // {
      //   do
      //   {
      //     fd = open(COM_UART, O_WRONLY);
      //     count += 1;
      //   } while (open(COM_UART, O_WRONLY) >= 0 | count < 5);
      //   printf("unable to open: %s\n", COM_UART);
      //   return -1;
      // }
      // sleep(2);

      // printf("Turning on  4v dcdc line..\n");
      // gpio_write(GPIO_DCDC_4V_EN, 1);
      // printf("Turning on COM 4V line..\n");
      // gpio_write(GPIO_COM_4V_EN, 1);

      // int ret = write(fd, beacon_data, BEACON_DATA_SIZE);
      // usleep(10000);
      // if (ret < 0)
      // {
      //   printf("unable to send data\n");
      //   for (int i = 0; i < BEACON_DATA_SIZE; i++)
      //   {
      //     ret = write(fd, &beacon_data[i], 1);
      //     usleep(1000);
      //   }
      //   if (ret < 0)
      //   {
      //     printf("Unable to send data through byte method..\n");
      //     return -1;
      //   }
      // }

      /*To delete*/
      if (beacon_data[1] == 0xb1)
      {
        printf("\n------------beacon 1 sent-------------\n");
        digipeating = 0;
      }
      else
      {
        printf("\n-------------beacon 2 sent-------------\n");
        digipeating = 1;
      }
      // printf("Beacon Type %d sequence complete\n", beacon_type);

      beacon_type = !beacon_type;

      // work_queue(HPWORK, &work_beacon, send_beacon_data, NULL, SEC2TICK(BEACON_DELAY));
    }
  }
  return 0;
}

// void FirstFunction()
// {
//   CRITICAL_FLAGS rd_flags_int;
//   struct file fp;

//   CRITICAL_FLAGS rd_flags_mfm = {0xff};
//   uint8_t mfm_have_data = 0;
//   ssize_t read_size_mfm = 0;

//   int fd = open("/dev/intflash", O_RDWR);
//   if (fd >= 0)
//   { // internal flash file opened successfully
//     syslog(LOG_INFO, "Printing Internal flash flag data.\n");
//     up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
//     print_critical_flag_data(&rd_flags_int);
//   }
//   else
//   {
//     syslog(LOG_ERR, "Error opening internal flash atempt 1......\n ");
//   }
//   close(fd);
//   int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
//   if (fd1 >= 0)
//   {
//     read_size_mfm = file_read(&fp, &rd_flags_mfm, sizeof(CRITICAL_FLAGS));
//   }
// }

// //Commander //COM
// // TODO: add work queue to antenna deployment
void Antenna_Deployment(int argc, char *argv[])
{
  uint16_t i = 0;
  int retval, retval1 = 0;
  CRITICAL_FLAGS rd_flags_int = {0};
  // CRITICAL_FLAGS rd_flags_mfm = {255, 255, 255, 255, 255, 255}; // = {0xff};
  ssize_t read_size_mfm = 0;

  check_flag_data();

  printf("\n----------------Antenna Deployment Flag: %d------------\n", critic_flags.ANT_DEP_STAT);

  // If the antenna is undeployed and no uplink received, perform deployment
  if (critic_flags.ANT_DEP_STAT != DEPLOYED || critic_flags.UL_STATE != UL_RX)
  {
    printf("****************************************\n*************************************************\n**************************************\n");
    printf("ANtenna not deployed\n-----------------Antenna deployment starting-----------------\n");
    printf("****************************************\n*************************************************\n**************************************\n");
    do
    {
      sleep(1); // 60
      i++;
      if (0 == i % 60)
      {
        printf("------------Antenna flag deploying in %d minutes-----------\n", (ANT_DEPLOY_TIME - i) / 60);
      }
    } while (i < ANT_DEPLOY_TIME);

    printf("Entering antenna deployment sequence\n");
    for (int i = 0; i <= 2; i++)
    {
      printf("Turning on burner circuit\nAttempt: %d\n", i + 1);
      retval = gpio_write(GPIO_BURNER_EN, true);
      retval1 = gpio_write(GPIO_UNREG_EN, true);
      sleep(8); // Antenna deployment time
      printf("Turning off burner circuit\n");
      gpio_write(GPIO_UNREG_EN, false);
      gpio_write(GPIO_BURNER_EN, false);
      sleep(10);
      printf("%d Antenna deployment sequence completed\n", i);
    }

    critic_flags.ANT_DEP_STAT = DEPLOYED;
    critic_flags.UL_STATE = UL_RX;

    sat_health.ant_dep_stat = critic_flags.ANT_DEP_STAT;
    sat_health.ul_state = critic_flags.UL_STATE;

    printf("Updated flag data...\n");
  }
  save_critics_flags(&critic_flags);
  // memset(critics_flags, "\0", sizeof(critic_flags));
  critic_flags.ANT_DEP_STAT = 0x00;
  load_critics_flags(&critic_flags);
  print_critical_flag_data(&critic_flags);
}

void adcs_operation(uint8_t mode)
{
  if (MISSION_STATUS.ADCS_MISSION == false && MISSION_STATUS.EPDM_MISSION == false && MISSION_STATUS.CAM_MISSION == false)
  {
    MISSION_STATUS.FLASH_OPERATION == false;
    MISSION_STATUS.ADCS_MISSION = true;
    sat_health.msn_flag = 0x21;
    struct file file_pointer, file_pointer2;

    int fd = file_open(&file_pointer, "/mnt/fs/sfm/mtd_mainstorage/adcs.txt", O_RDONLY);
    uint32_t initial_count;
    if (fd >= 0)
      initial_count = file_seek(&file_pointer, 0, SEEK_END);
    file_close(&file_pointer);

    int hand;
    // turn_msn_on_off(1, 0);
    sleep(1);
    turn_msn_on_off(1, 1);
    sleep(1);
    uint8_t data2[7] = {0x53, 0x0a, 0x0d, 0x0c, 0x01, 0x7e};
    data2[4] = mode;
    pet_counter = 0;

    // do
    // {
    //   hand = handshake_MSN_ADCS(1, data);
    //   // hand = handshake_MSN(2, data);

    // } while (hand < 0);
    hand = 0;
    uint8_t ret;
    pet_counter = 0;

    sleep(1);

    do
    {
      hand = handshake_MSN_ADCS(1, data2);
    } while (hand < 0);
    // sleep(2);
    int p = 0;
    uint8_t data3, data4;
    uint32_t counter1 = 0;
    uint8_t cam[1200] = {'\0'};
    // memset(cam,'\0',sizeof(cam));
    printf("**************Starting data receive*************\n");
    int fd2 = open(ADCS_UART, O_RDONLY);
    int t;

    while (1)
    {
      data4 = data3;
      ret = read(fd2, &data3, 1);
      printf("%02x ", data3);
      if (counter1 % 180 == 0)
      {
        printf("\n");
        pet_counter = 0;
      }
      cam[counter1] = data3;
      if (data4 == 0xff && data3 == 0xd9)
      {
        printf("%02x ", data3);

        pet_counter = 0;
        break;
      }
      // if (counter1 > t*6)
      //   break;
      // if(data4== 0xff);
      counter1++;
      // break;
    }
    // cam[counter1]='\0';
    close(fd2);
    pet_counter = 0;
    sleep(1);
    turn_msn_on_off(1, 0);
    MISSION_STATUS.ADCS_MISSION = false;
    MISSION_STATUS.FLASH_OPERATION = false;
    sat_health.msn_flag = 0x11;
    usleep(10000);
    syslog(LOG_DEBUG, "T0tal data received %d\n ADCS operation success\n", counter1);
    sleep(1);
    cam[counter1++] = 0xff;
    cam[counter1++] = 0xd9;
    cam[counter1++] = '\0';
    sleep(1);
    mission_data("/mtd_mainstorage/adcs.txt", &cam, counter1);
    mission_data("/mtd_mission/adcs.txt", &cam, counter1);
    // fd = file_open(&file_pointer, "/mnt/fs/mfm/mtd_mainstorage/adcs.txt", O_CREAT | O_RDWR | O_APPEND);
    // file_write(&file_pointer, cam, strlen(cam));
    // file_close(&file_pointer);
    // close(fd);
    // fd = file_open(&file_pointer, "/mnt/fs/mfm/mtd_mission/adcs.txt", O_CREAT | O_RDWR | O_APPEND);
    // file_write(&file_pointer, cam, strlen(cam));
    // file_close(&file_pointer);
    // close(fd);
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
    sleep(2);
    uint8_t data2[7] = {0x53, 0x0c, 0x0a, 0x0e, 0x01, 0x7e};
    do
    {
      hand = handshake_MSN(2, data);
    } while (hand < 0);
    hand = 0;
    uint8_t ret, fd;
    // char data[240];
    sleep(1);

    sleep(1);
    // do
    // {
    // hand = handshake_MSN(3, data2);
    // } while (hand < 0);
    // sleep(3);
    // int fd = open(CAM_UART, O_WRONLY); // Open in non-blocking mode
    // if (fd < 0)
    // {
    //   printf("Error opening %s\n", CAM_UART);
    //   usleep(PRINT_DELAY);
    //   return -1;
    // }

    // // Writing handshake data
    // ret = write(fd, data2, 7);
    // close(fd);
    // if (hand == 0)
    {
      syslog(LOG_DEBUG, "Command %s sent\n", data2);

      // int p = 0;
      // uint8_t data3, data4;
      // uint32_t counter1 = 0;
      // uint8_t cam[2] = {'\0'};
      // int fd2 = open(CAM_UART, O_RDONLY);
      // // sleep(10);
      // sleep(2);
      // while (1)
      // {
      //   data4 = data3;
      //   ret = read(fd2, &data3, 1);
      //   printf("%02x ", data3);
      //   cam[counter1] = data3;
      //   if (data4 == 0xff && data3 == 0xd9)
      //   {
      //     break;
      //   }
      //   // if(data4== 0xff);
      //   counter1++;
      //   // break;
      // }
      // // cam[counter1]='\0';
      // close(fd2);
      // counter1 = 0;
      // syslog(LOG_DEBUG, "__________________________________________");
      // syslog(LOG_DEBUG, "_____________________RGB camera_____________________");
      // fd2 = open(CAM_UART, O_RDONLY);
      // // sleep(10);
      // sleep(3);
      // while (1)
      // {
      //   data4 = data3;
      //   ret = read(fd2, &data3, 1);
      //   printf("%02x ", data3);
      //   cam[counter1] = data3;
      //   if (data4 == 0xff && data3 == 0xd9)
      //   {
      //     break;
      //   }
      //   // if(data4== 0xff);
      //   counter1++;
      //   // break;
      // }
      // // cam[counter1]='\0';
      // close(fd2);
      turn_msn_on_off(2, 0);
      MISSION_STATUS.FLASH_OPERATION = false;
      MISSION_STATUS.CAM_MISSION = false;

      // usleep(10000);
      // syslog(LOG_DEBUG, "TOtal data received %d\n CAM operation success\n", counter1);
      // sleep(2);
      // cam[counter1++] = 0xff;
      // cam[counter1++] = 0xd9;

      // mission_data("/cam.txt", &cam, counter1);
      // uint8_t ack[BEACON_DATA_SIZE] = {0x53, 0xac, 0x04, 0x01, 0x05, 0x05, 0x7e, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x70, 0x71, 0x72, 0x1e, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x80, 0x7e, 0x7e, 0x7e};
      // sleep(1);
      // send_data_uart(COM_UART, ack, sizeof(ack));
    }
  }
}

void epdm_operation()
{
  int hand;
  int fd;
  uint64_t initial_count = 0;
  struct file file_pointer, file_pointer2;

  if (MISSION_STATUS.ADCS_MISSION == false && MISSION_STATUS.CAM_MISSION == false)
  {
    uint8_t cam[37000] = {'\0'};

    sat_health.msn_flag = 0x21;
    MISSION_STATUS.EPDM_MISSION = true;
    fd = file_open(&file_pointer, "/mnt/fs/sfm/mtd_mainstorage/epdm.txt", O_RDONLY);
    if (fd >= 0)
      initial_count = file_seek(&file_pointer, 0, SEEK_END);
    file_close(&file_pointer);

    // gpio_write(GPIO_SFM_MODE, true);
    char *dev_path = EPDM_UART;
    // turn_msn_on_off(3, 0);
    // sleep(1);
    turn_msn_on_off(3, 1);
    sleep(1);

    hand = handshake_MSN(3, data);

    uint8_t data2[] = {0x53, 0x0e, 0x0d, 0x0e, 0x01, 0x7e};
    uint8_t ret;
    // sleep(2);
    // do
    // {
    //   hand = handshake_MSN(3, data2);
    // } while (hand < 0);
    syslog(LOG_DEBUG, "Command %s sent\n", data2);

    int p = 0;
    uint8_t data3, data4;
    uint32_t counter1 = 0;

    int fd2 = open(EPDM_UART, O_RDONLY);

    while (1)

    {
      data4 = data3;
      ret = read(fd2, &data3, 1);
      // syslog(LOG_DEBUG,
      // printf("%02x ", data3);
      if (counter1 % 400 == 0)
      {
        syslog(LOG_DEBUG, "COUNTER : %d\n pet_counter: %d", counter1, pet_counter);
        pet_counter = 0;
      }
      cam[counter1] = data3;
      if (data4 == 0xff && data3 == 0xd9 && counter1 > 35000)
      {
        // data4=000;
        break;
      }
      counter1++;
    }
    close(fd2);
    sleep(1);
    turn_msn_on_off(3, 0);

    // data consistency
    // int32_t ret;
    uint32_t count = 0;
    fd = file_open(&file_pointer, "/mnt/fs/sfm/mtd_mainstorage/epdm.txt", O_CREAT | O_RDWR | O_APPEND);
    uint32_t counter = 0;
    if (fd >= 0)
      counter = file_seek(&file_pointer, 0, SEEK_END);
    file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mission/epdm.txt", O_CREAT | O_WRONLY | O_APPEND);
    uint32_t counter2 = file_seek(&file_pointer2, 0, SEEK_END);
    file_close(&file_pointer2);
    printf("THe value of initial_count :%d and counter: %d and counter2 : %d\n", initial_count, counter, counter2);
    if (counter <= 0)
    {
      ret = open("/mnt/fs/sfm/mtd_mainstorage/epdm.txt", O_CREAT | O_WRONLY | O_APPEND);

      uint32_t writeBytes;
      if (ret >= 0)
        writeBytes = write(ret, cam, counter1);
      close(ret);
      if (writeBytes >= 1000)
        printf("The file of size %d has been written to SFM\n", writeBytes);

      ret = open("/mnt/fs/mfm/mtd_mission/epdm.txt", O_CREAT | O_WRONLY | O_APPEND);
      if (ret >= 0)
        writeBytes = write(ret, cam, counter1);
      close(ret);
      if (writeBytes >= 1000)
        printf("The file of size %d has been written to MFM\n", writeBytes);
    }
    else if (initial_count == counter || counter - initial_count < 10000)
    {
      ret = open("/mnt/fs/sfm/mtd_mainstorage/epdm.txt", O_WRONLY | O_APPEND);
      uint32_t writeBytes = 0;
      if (ret >= 0)
      {
        writeBytes = write(ret, cam, counter1);
      }
      close(ret);
      if (writeBytes >= 1000)
        printf("The file of size %d has been written to SFM\n", writeBytes);

      ret = open("/mnt/fs/mfm/mtd_mission/epdm.txt", O_WRONLY | O_APPEND);
      if (ret >= 0)
      {
        writeBytes = write(ret, cam, counter1);
      }
      close(ret);
      if (writeBytes >= 1000)
        printf("The file of size %d has been written to MFM\n", writeBytes);
    }
    else
    {
      do
      {
        if (file_seek(&file_pointer, counter2, SEEK_SET) >= 0)
        {
          if (counter - counter2 > 23500)
          {
            count = 23500;
          }
          else
          {
            count = counter - counter2;
          }
          counter2 += count;
          ret = file_read(&file_pointer, cam, count);
          printf("ret is %d %d", ret, counter - counter2);
          for (int32_t i = 0; i < count; i++)
          {
            printf("%02X ", cam[i]);
          }
          // if ( >= 0)
          ret = open("/mnt/fs/mfm/mtd_mission/test.txt", O_WRONLY | O_APPEND);
          uint32_t writeBytes = 0;
          if (ret >= 0)

            writeBytes = write(ret, cam, count);
          close(ret);
          // ssize_t writeBytes = file_write(&file_pointer2, &data1, 4000);
          printf("The file of size %d has been written\n", writeBytes);
        }
      } while (counter2 < counter);
    }
    // data consistency
    MISSION_STATUS.EPDM_MISSION = false;
    sat_health.msn_flag = 0x21;
    // free(data2);
    syslog(LOG_DEBUG, "Total data received %d\n EPDM operation succeded\nData has been saved to main flash memory.\n", counter1);
    uint8_t ack[BEACON_DATA_SIZE] = {0x53, 0xac, 0x04, 0x01, 0x05, 0x05, 0x7e, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x70, 0x71, 0x72, 0x1e, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x80, 0x7e, 0x7e};
    // sleep(1);
    send_data_uart(COM_UART, ack, sizeof(ack));
    // free(cam);
    // free()
  }
}

// int configure_watchdog(int fd, int timeout)
// {
//   /* Set the watchdog timeout */
//   int ret = ioctl(fd, WDIOC_SETTIMEOUT, (unsigned long)timeout);
//   if (ret < 0)
//   {
//     perror("Failed to set watchdog timeout");
//     return -1;
//   }

//   /* Start the watchdog */
//   ret = ioctl(fd, WDIOC_START, 0);
//   if (ret < 0)
//   {
//     perror("Failed to start watchdog");
//     return -1;
//   }

//   return 0;
// }

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

/****************************************************************************
 * Name: int_adc1_data_convert
 *
 * Parameters: *temp_buff -- pointer buffer where the data is stored after conversion
 *
 * Details:
 *   ADC_SUP -- 17th channel (doesn't have pinout) data is ADC_SUPP on vrefint
 *   Firstly, raw data is converted by using formula: channel_data * vrefint_data / 4095
 *   Then data is converted according to current/voltage sensors data. LMP8640 is used to convert all current data except: raw_current, solar_total_current, and battery_current
 *
 *    except batt_mon, everything is current data only ...
 * Channels and Corresponding data:
 *
 ****************************************************************************/
#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1
void int_adc1_data_convert(float *temp_buff)
{
  // printf("\n----------------------------------------------------------\n");

  // float temp_buff[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_NSAMPLES];
  float ADC_SUP = 1.2 * 4095 / (int_adc1_sample[14].am_data);
  // raw adc value here test baki
  // printf("ADC_SUP is %f\n", ADC_SUP);
  for (int i = 0; i < CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE; i++)
  {
    temp_buff[i] = (int_adc1_sample[i].am_data * 3.3 / 4095); // right now, using 1.2 as vrefint channel data... converting all the data to their respective voltages
    if (i == 4)
    {
      temp_buff[i] = (temp_buff[i] * (1100 + 931)) / 931; // this is for battery monitor (voltage data, no need for conversion)
    }
    // else if (i == 0 || i == 9 - 1 || i == 11 - 1) // this one is for battery current, solar panel total current and raw current respectively
    else if (int_adc1_sample[i].am_channel == 0 | int_adc1_sample[i].am_channel == 9 | int_adc1_sample[i].am_channel == 12)
    {
      // printf("THe value of temp_buff[%d] is %f", i, temp_buff[i]);
      temp_buff[i] = ((temp_buff[i] - 1.65) / SENS_TMCS) * 1000 * 1000;
      // printf("THe value of temp_buff[%d] is %f", i, temp_buff[i]);
    }
    else
    {
      // all current sensors use lmp8640 current sensor ...
      temp_buff[i] = (temp_buff[i] / (2 * RES_LMP8640 * GAIN_LMP8640)) * 1000;
    }
    // printf("Temp_buf[%d] : %f \n", i, temp_buff[i]);
  }
  // printf("\n----------------------------------------------------------\n");
}
#endif

// #ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1
// int read_int_adc1()
// {

//   UNUSED(ret);

//   /* Check if we have initialized */
//   if (!g_adcstate1.initialized)
//   {
//     /* Initialization of the ADC hardware must be performed by
//      * board-specific logic prior to running this test.
//      */

//     /* Set the default values */

//     adc_devpath(&g_adcstate1, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_DEVPATH);

//     g_adcstate1.initialized = true;
//   }

//   g_adcstate1.count = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_NSAMPLES;

//   /* Parse the command line */

//   /* If this example is configured as an NX add-on, then limit the number of
//    * samples that we collect before returning.  Otherwise, we never return
//    */

//   printf("adc_main: g_adcstate.count: %d\n", g_adcstate1.count);

//   /* Open the ADC device for reading */

//   printf("adc_main: Hardware initialized. Opening the ADC device: %s\n",
//          g_adcstate1.devpath);

//   /* Opening internal ADC1 */
//   adc1_config.fd = open(CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_DEVPATH, O_RDONLY);
//   if (adc1_config.fd < 0)
//   {
//     printf("adc_main: open %s failed: %d\n", g_adcstate1.devpath, errno);
//     adc1_config.errval = 2;
//     goto errout;
//   }
//   elapsed = 0;
//   while (elapsed < required)
//   {
//     usleep(1);
//     elapsed++;
//   }
//   /* Now loop the appropriate number of times, displaying the collected
//    * ADC samples.
//    */
//   // UNUSED(elapsed);
//   // UNUSED(required);
//   for (int j = 0; j < 1; j++)
//   {
//     /* Flush any output before the loop entered or from the previous pass
//      * through the loop.
//      */
//     elapsed = 0;
//     fflush(stdout);

// #ifdef CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_SWTRIG
//     /* Issue the software trigger to start ADC conversion */

//     ret = ioctl(adc1_config.fd, ANIOC_TRIGGER, 0);
//     if (ret < 0)
//     {
//       int errcode = errno;
//       printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
//     }
// #endif

//     /* Read up to CONFIG_CUSTOM_APPS_ADC_GROUPSIZE samples */

//     adc1_config.readsize = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE * sizeof(struct adc_msg_s);
//     adc1_config.nbytes = read(adc1_config.fd, int_adc1_sample, adc1_config.readsize);

//     printf("Readsize: %d \n nbytes: %d\n CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE : %d \n ADCSTATE READCOUNT: %d \r\n", adc1_config.readsize, adc1_config.nbytes, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE, g_adcstate1.count);

//     /* Handle unexpected return values */
//     if (adc1_config.nbytes < 0)
//     {
//       adc1_config.errval = errno;
//       if (adc1_config.errval != EINTR)
//       {
//         printf("adc_main: read %s failed: %d\n",
//                g_adcstate1.devpath, adc1_config.errval);
//         adc1_config.errval = 3;
//         goto errout_with_dev;
//       }

//       printf("adc_main: Interrupted read...\n");
//     }
//     else if (adc1_config.nbytes == 0)
//     {
//       printf("adc_main: No data read, Ignoring\n");
//     }

//     /* Print the sample data on successful return */

//     else
//     {
//       int nsamples = adc1_config.nbytes / sizeof(struct adc_msg_s);
//       if (nsamples * sizeof(struct adc_msg_s) != adc1_config.nbytes)
//       {
//         printf("adc_main: read size=%ld is not a multiple of "
//                "sample size=%d, Ignoring\n",
//                (long)adc1_config.nbytes, sizeof(struct adc_msg_s));
//       }
//       else
//       {
//         printf("Sample:\n");
//         for (int i = 0; i < nsamples; i++)
//         {
//           printf("%d: channel: %d value: %" PRId32 "\n",
//                  i, int_adc1_sample[i].am_channel, int_adc1_sample[i].am_data);
//         }
//       }
//     }

//     if (g_adcstate1.count && --g_adcstate1.count <= 0)
//     {
//       break;
//     }
//   }

//   close(adc1_config.fd);
//   return OK;

// /* Error exits */
// errout_with_dev:
//   close(adc1_config.fd);

// errout:
//   printf("Terminating!\n");
//   fflush(stdout);
//   close(adc1_config.fd);
//   return adc1_config.errval;
// }
// #endif // CONFIG_EAMPLES_CUBUS_USE_ADC_1

/****************************************************************************
 * Name: int_adc3_data_convert
 *
 * Parameters:  *temp_buff -- float pointer to the data that is converted
 *
 * Details:
 *    only one channel data is converted by internal adc 3 i.e. 4V_I
 *    it uses LMP8640 current sensor, so same formula given above is used ...
 ****************************************************************************/

#ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3
void int_adc3_data_convert(float *temp_buff_1)
{
  for (int i = 0; i < CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE; i++)
  {
    temp_buff_1[i] = (float)int_adc3_sample[0].am_data * 3.3 / 4095;
    temp_buff_1[i] = (float)(temp_buff_1[i] / (2 * RES_LMP8640 * GAIN_LMP8640)) * 1000;
    // printf("THe value of 3v3_2_i is %f\n", temp_buff_1[i]);
  }
}
#endif

/****************************************************************************
 * Name: adc_devpath
 ****************************************************************************/
#if defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1) || defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3)
static void adc_devpath(FAR struct adc_state_s *adc, FAR const char *devpath)
{
  /* Get rid of any old device path */

  if (adc->devpath)
  {
    free(adc->devpath);
  }

  /* Then set-up the new device path by copying the string */

  adc->devpath = strdup(devpath);
}
#endif

void make_satellite_health()
{
#if defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC1) || defined(CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC3)
  float int_adc1_temp[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC1_GROUPSIZE] = {'\0'};
  int_adc1_data_convert(int_adc1_temp);

  float int_adc3_temp[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC3_GROUPSIZE] = {'\0'};
  int_adc3_data_convert(int_adc3_temp);
  /* External ADC data */
  // sat_health.sol_t_v = (int16_t)ext_adc_data[0].processed_data;
  // sat_health.raw_v = (int16_t)ext_adc_data[1].processed_data;
  // sat_health.sol_p5_v = (int16_t)ext_adc_data[2].processed_data;
  // sat_health.sol_p4_v = (int16_t)ext_adc_data[3].processed_data;
  // sat_health.sol_p3_v = (int16_t)ext_adc_data[4].processed_data;
  // sat_health.sol_p1_v = (int16_t)ext_adc_data[5].processed_data;
  // sat_health.sol_p2_v = (int16_t)ext_adc_data[6].processed_data;

  // sat_health.ant_temp_out = (float)ext_adc_data[8].processed_data;
  // // sat_health.temp_batt = (int16_t)ext_adc_data[9].processed_data;
  // sat_health.temp_bpb = (int16_t)ext_adc_data[10].processed_data;
  // sat_health.temp_z = (int16_t)ext_adc_data[11].processed_data;

  /* Internal ADC1 data */

  // before calibration

  // sat_health.batt_c = (int16_t)(int_adc1_temp[9] * 100);
  // sat_health.sol_t_c = (int16_t)(int_adc1_temp[10] * 100);
  // sat_health.raw_c = (int16_t)(int_adc1_temp[11] * 100);

  // sat_health.unreg_c = (int16_t)(int_adc1_temp[0] * 100);
  // sat_health.v3_main_c = (int16_t)(int_adc1_temp[1] * 100);
  // sat_health.v3_com_c = (int16_t)(int_adc1_temp[2] * 100);
  // sat_health.v5_c = (int16_t)(int_adc1_temp[3] * 100);

  // sat_health.batt_volt = (int16_t)(int_adc1_temp[4] * 100);

  // sat_health.sol_p1_c = (int16_t)(int_adc1_temp[5] * 100);
  // sat_health.v3_2_c = (int16_t)(int_adc1_temp[6] * 100);
  // sat_health.sol_p4_c = (int16_t)(int_adc1_temp[7] * 100);
  // sat_health.sol_p5_c = (int16_t)(int_adc1_temp[8] * 100);

  // sat_health.sol_p2_c = (int16_t)(int_adc1_temp[12] * 100);
  // sat_health.sol_p3_c = (int16_t)(int_adc1_temp[13] * 100);

  // /* internal adc2 data*/
  // sat_health.v4_c = (int16_t)(int_adc3_temp[0] * 100);

  // before calibration

  // after calib
  // printf("%0.2f,int")
  // for (int i = 0; i < 13; i++)
  // {
  //   printf("int_adc1_data_convert[%d] %0.2f \n", i, int_adc1_temp[i]);
  // }
  sat_health.sol_t_c = (int16_t)(int_adc1_temp[0] * 100);

  sat_health.v5_c = (int16_t)(int_adc1_temp[1] * 100);
  sat_health.v3_com_c = (int16_t)(int_adc1_temp[2] * 100);
  sat_health.v3_main_c = (int16_t)(int_adc1_temp[3] * 100);
  sat_health.batt_volt = (int16_t)(int_adc1_temp[4] * 100);
  sat_health.sol_p5_c = (int16_t)(int_adc1_temp[5] * 100);
  sat_health.sol_p4_c = (int16_t)(int_adc1_temp[6] * 100);
  sat_health.sol_p3_c = (int16_t)(int_adc1_temp[7] * 100);
  sat_health.batt_c = (int16_t)(int_adc1_temp[8] * 100);
  sat_health.unreg_c = (int16_t)(int_adc1_temp[9] * 100);
  sat_health.v4_c = (int16_t)(int_adc1_temp[10] * 100);

  sat_health.raw_c = (int32_t)(int_adc1_temp[11] * 100);
  sat_health.sol_p1_c = (int16_t)(int_adc1_temp[12] * 100);

  sat_health.sol_p2_c = (int16_t)(int_adc1_temp[13] * 100);
  sat_health.v3_2_c = (int16_t)(int_adc3_temp[0] * 100);

  /* internal adc2 data*/
  // after calib

#endif
  sat_health.ant_dep_stat = critic_flags.ANT_DEP_STAT;
  sat_health.oper_mode = critic_flags.OPER_MODE;
  sat_health.kill_switch = critic_flags.KILL_SWITCH_STAT;
  sat_health.rst_counter = critic_flags.RST_COUNT;
  sat_health.rsv_flag = critic_flags.RSV_FLAG;
  sat_health.ul_state = critic_flags.UL_STATE;

  // print_satellite_health_data(&sat_health);
}

// void RUN_ADC()
// {
//   read_int_adc1();
//   read_int_adc3();
//   // ext_adc_main();
//   make_satellite_health();
//   // usleep(10000);
//   sleep(1);
//   // store_sat_health_data(&sat_health);
// }

void ADC_Temp_Conv(float *adc_conv_buf, float *temp_buf, int channel)
{
  float root = 0;

  if (channel == 16)
  { // Battery temperature channel
    float res = (adc_conv_buf[1] * 10000) / (2.5 * adc_conv_buf[1]);
    float tempk = 3976 * 298 / (3976 + (298 * log10(10000 / res)));
    temp_buf[1] = (tempk - 273) * 100;
  }
  else
  {
    root = sqrtf(
        (5.506 * 5.506) +
        (4 * 0.00176 * (870.6 + (adc_conv_buf[1] * 1000))));
    temp_buf[1] = (((5.506 * root) / (2 * (-0.00176))) - 30) * 100;
  }
}

float convert1(data)
{
  float result = (data / (2 * RES_LMP8640 * GAIN_LMP8640)) * 1000;
  return result;
}

int ads7953_receiver(int argc, FAR char *argv[])
{
  int raw_sub_fd = orb_subscribe(ORB_ID(ads7953_raw_msg));
  int temp_sub_fd = orb_subscribe(ORB_ID(sat_temp_msg));
  int volts_sub_fd = orb_subscribe(ORB_ID(sat_volts_msg));

  struct pollfd fds[] = {
      {.fd = raw_sub_fd, .events = POLLIN},
      {.fd = temp_sub_fd, .events = POLLIN},
      {.fd = volts_sub_fd, .events = POLLIN},
  };
  struct sensor_rgb rgb;
  int adc_instance = 0;
  int afd = orb_advertise_multi_queue_persist(ORB_ID(sensor_rgb), &sat_health,
                                              &adc_instance, sizeof(struct sensor_rgb));

  while (1)
  {
    // printf("----------------------------------------\n");
    // printf("Polling\n");

    // printf("----------------------------------------\n");

    // Poll for new data
    int poll_ret = poll(fds, 3, 1000);

    if (poll_ret == 0)
    {
      printf("Poll timeout\n");
      continue;
    }

    if (poll_ret < 0)
    {
      printf("Poll error: %d\n", errno);
      continue;
    }

    // Check for ads7953_raw_msg updates
    if (fds[0].revents & POLLIN)
    {
      struct ads7953_raw_msg raw_msg;
      orb_copy(ORB_ID(ads7953_raw_msg), raw_sub_fd, &raw_msg);

      // printf("ads7953_raw_msg:\n");
      // printf("  timestamp: %" PRIu64 "\n", raw_msg.timestamp);
      // for (int i = 0; i < 7; ++i)
      // {
      //   printf("  Voltage Channel %d: %u (%.4f V)\n", i, raw_msg.volts_chan[i], raw_msg.volts_chan_volts[i]);
      // }
      // for (int i = 0; i < 8; ++i)
      // {
      //   x[i] = raw_msg.temp_chan[i];
      //   printf("  Temperature Channel %d: %u (%.4f V)\n", i, raw_msg.temp_chan[i], raw_msg.temp_chan_volts[i]);
      // }
      ADC_Temp_Conv(&x, &y, 8);
    }

    // Check for sat_temp_msg updates
    if (fds[1].revents & POLLIN)
    {
      struct sat_temp_msg temp_msg;
      orb_copy(ORB_ID(sat_temp_msg), temp_sub_fd, &temp_msg);
      sat_health.temp_x = temp_msg.temp_2;
      sat_health.temp_x1 = temp_msg.temp_3;
      sat_health.temp_y = temp_msg.temp_4;
      sat_health.temp_y1 = temp_msg.temp_5;
      sat_health.ant_temp_out = temp_msg.temp_ant;
      // sat_health.temp_z = 0;
      // sat_health.temp_z1 = 0;
      sat_health.temp_bpb = temp_msg.temp_bpb;
      // sat_health.temp_obc = temp_msg.temp_obc;//TODO add temp of MCU
      // // sat_health.temp_com = temp_msg.;
      sat_health.temp_batt = temp_msg.batt_temp;
      // sat_health.batt_volt = ;

      // int8_t rsv_cmd;

      // int8_t ant_dep_stat;
      // int8_t ul_state;
      // int8_t oper_mode;
      // int8_t msn_flag;
      // int8_t rsv_flag;
      // int8_t kill_switch;

      // int16_t ant_temp_out;

      // printf("sat_temp_msg:\n");
      // printf("********************************************\n");
      // printf("  timestamp: %" PRIu64 " | ", temp_msg.timestamp);
      // printf("  batt_temp: %.4f \n ", temp_msg.batt_temp);
      // printf("  temp_bpb: %.4f \n ", temp_msg.temp_bpb);
      // printf("  temp_ant: %.4f \n", temp_msg.temp_ant);
      // printf("  temp_z_pos: %.4f\n", temp_msg.temp_z_pos);
      // printf("  temp_5: %.4f\n", temp_msg.temp_5);
      // printf("  temp_4: %.4f\n", temp_msg.temp_4);
      // printf("  temp_3: %.4f\n", temp_msg.temp_3);
      // printf("  temp_2: %.4f\n", temp_msg.temp_2);
    }

    // Check for sat_volts_msg updates
    if (fds[2].revents & POLLIN)
    {
      struct sat_volts_msg volts_msg;
      orb_copy(ORB_ID(sat_volts_msg), volts_sub_fd, &volts_msg);
      sat_health.sol_p1_v = (int16_t)(1000 * volts_msg.volt_sp1);
      sat_health.sol_p2_v = (int16_t)(1000 * volts_msg.volt_sp2);
      sat_health.sol_p3_v = (int16_t)(1000 * volts_msg.volt_sp3);
      sat_health.sol_p4_v = (int16_t)(1000 * volts_msg.volt_sp4);
      sat_health.sol_p5_v = (int16_t)(1000 * volts_msg.volt_sp5);
      sat_health.sol_t_v = (int16_t)(1000 * volts_msg.volt_SolT);
      sat_health.raw_v = (int16_t)(1000 * volts_msg.volt_raw);
      // printf("sat_volts_msg:\n");
      // printf("  timestamp: %" PRIu64 "\n", volts_msg.timestamp);
      // printf("  volt_SolT: %.4f V\n", volts_msg.volt_SolT);
      // printf("  volt_raw: %.4f V\n", volts_msg.volt_raw);
      // printf("  volt_sp5: %.4f V\n", volts_msg.volt_sp5);
      // printf("  volt_sp4: %.4f V\n", volts_msg.volt_sp4);
      // printf("  volt_sp3: %.4f V\n", volts_msg.volt_sp3);
      // printf("  volt_sp1: %.4f V\n", volts_msg.volt_sp1);
      // printf("  volt_sp2: %.4f V\n", volts_msg.volt_sp2);
    }

    // usleep(500000);
    // print_satellite_health_data(&sat_health);
    read_int_adc1();
    read_int_adc3();
    make_satellite_health();
    sat_health.oper_mode = critic_flags.OPER_MODE;
    if (critic_flags.RSV_FLAG != 0)
    {
      sat_health.rsv_flag = critic_flags.RSV_FLAG;
      if (sat_health.rsv_cmd == 0x00)
      {
        sat_health.rsv_cmd = 1;
      }
    }

    if (afd < 0)
    {
      syslog(LOG_ERR, "Orb advertise failed.\n");
    }
    else
    {
      if (OK != orb_publish(ORB_ID(sensor_rgb), afd, &sat_health))
      {
        syslog(LOG_ERR, "Orb Publish failed\n");
      }
      else
      {
        // syslog(LOG_ERR, "ORB published checkpoint\n");
      }
    }
    sleep(8);
  }
  ret = orb_unadvertise(afd);

  return 0;
}

void print_satellite_health_data(satellite_health_s *sat_health)
{
  printf(" *******************************************\r\n");
  printf(" |   Satellite operation mode:\t %s \t|\r\n", (sat_health->oper_mode ? "Normal Mode" : "Power Saving Mode"));
  printf(" |   X axis acceleration    \t %d m/s^2\t|\r\n", sat_health->accl_x);
  printf(" |   Y axis acceleration    \t %d m/s^2\t|\r\n", sat_health->accl_y);
  printf(" |   Z axis acceleration    \t %d m/s^2\t|\r\n", sat_health->accl_z);

  printf(" |   X axis Gyro data       \t %d deg/s\t|\r\n", sat_health->gyro_x);
  printf(" |   Y axis Gyro data       \t %d deg/s\t|\r\n", sat_health->gyro_y);
  printf(" |   Z axis gyro data       \t %d deg/s\t|\r\n", sat_health->gyro_z);

  printf(" |   X axis magnetic field  \t %d uT\t|\r\n", sat_health->mag_x);
  printf(" |   Y axis magnetic field  \t %d uT\t|\r\n", sat_health->mag_y);
  printf(" |   Z axis magnetic field  \t %d uT\t|\r\n", sat_health->mag_z);
  printf(" |----------------------------------------------------------|\r\n");

  printf(" |   Solar Panel 1 Voltage: \t %d mV\t|\r\n", sat_health->sol_p1_v);
  printf(" |   Solar Panel 2 Voltage: \t %d mV\t|\r\n", sat_health->sol_p2_v);
  printf(" |   Solar Panel 3 Voltage: \t %d mV\t|\r\n", sat_health->sol_p3_v);
  printf(" |   Solar Panel 4 Voltage: \t %d mV\t|\r\n", sat_health->sol_p4_v);
  printf(" |   Solar Panel 5 Voltage: \t %d mV\t|\r\n", sat_health->sol_p5_v);
  printf(" |   Solar Panel T Voltage: \t %d mV\t|\r\n", sat_health->sol_t_v);
  printf(" |----------------------------------------------------------|\r\n");

  printf(" |   Solar Panel 1 Current: \t %d mA\t|\r\n", sat_health->sol_p1_c);
  printf(" |   Solar Panel 2 Current: \t %d mA\t|\r\n", sat_health->sol_p2_c);
  printf(" |   Solar Panel 3 Current: \t %d mA\t|\r\n", sat_health->sol_p3_c);
  printf(" |   Solar Panel 4 Current: \t %d mA\t|\r\n", sat_health->sol_p4_c);
  printf(" |   Solar Panel 5 Current: \t %d mA\t|\r\n", sat_health->sol_p5_c);
  printf(" |   Solar Panel T Current: \t %d mA\t|\r\n", sat_health->sol_t_c);
  printf(" |----------------------------------------------------------|\r\n");

  printf(" |   Unreg Line Current:    \t %d mA\t|\r\n", sat_health->unreg_c);
  printf(" |   Main 3v3 Current:      \t %d mA\t|\r\n", sat_health->v3_main_c);
  printf(" |   COM 3v3 Current:       \t %d mA\t|\r\n", sat_health->v3_com_c);
  printf(" |   5 Volts line Current:  \t %d mA\t|\r\n", sat_health->v5_c);
  printf(" |   3v3 2 line Current:    \t %d mA\t|\r\n", sat_health->v3_2_c);
  printf(" |----------------------------------------------------------|\r\n");
  printf(" |   Raw Current:           \t %d mA\t|\r\n", sat_health->raw_c);
  printf(" |   Raw Voltage:           \t %d mV\t|\r\n", sat_health->raw_v);
  printf(" |----------------------------------------------------------|\r\n");
  printf(" |   Battery Total Voltage: \t %d mV\t|\r\n", sat_health->batt_volt);
  printf(" |   Battery Total Current: \t %d mA\t|\r\n", sat_health->batt_c);
  printf(" |   Battery Temperature:   \t %d C\t|\r\n", sat_health->temp_batt);
  printf(" |----------------------------------------------------------|\r\n");
  printf(" |   Solar Panel 1 Status   \t %s \t|\r\n", (sat_health->sol_p1_v) >= 1000 ? "Working" : "Not Working");
  printf(" |   Solar Panel 2 Status   \t %s \t|\r\n", (sat_health->sol_p2_v) >= 1000 ? "Working" : "Not Working");
  printf(" |   Solar Panel 3 Status   \t %s \t|\r\n", (sat_health->sol_p3_v) >= 1000 ? "Working" : "Not Working");
  printf(" |   Solar Panel 4 Status   \t %s \t|\r\n", (sat_health->sol_p4_v) >= 1000 ? "Working" : "Not Working");
  printf(" |   Solar Panel 5 Status   \t %s \t|\r\n", (sat_health->sol_p5_v) >= 1000 ? "Working" : "Not Working");
  printf(" |----------------------------------------------------------|\r\n");
  printf(" |   BPB Temperature            \t %d C\t|\r\n", sat_health->temp_bpb);
  printf(" |   Antenna Panel Temperature  \t %d C\t|\r\n", sat_health->ant_temp_out);
  printf(" |   Solar Panel Z Temperature  \t %d C\t|\r\n", sat_health->temp_z);
  printf(" |   Solar Panel X Temperature  \t %d C\t|\r\n", sat_health->temp_x);
  printf(" |   Solar Panel Y Temperature  \t %d C\t|\r\n", sat_health->temp_y);
  printf(" |----------------------------------------------------------|\r\n");

  // ORB_DEFINE(sensor_rgb, struct sensor_rgb, print_sensor_rgb_message);

  // ORB_DEFINE(sensor_rgb,struct orb_sensor_rgb, print_satellite_health_data);

  // printf(" |   Solar Panel 4 Temperature  \t %d C\t|\r\n", sat_health->temp_batt);(" *********************************************\r\n");
}

// int subscribe_and_retrieve_data()
// {
//   int fd;
//   int ret;

//   struct orb_mag_scaled_s mag_scaled;

//   /* Subscribe to the orb_mag_scaled topic */
//   fd = orb_subscribe(ORB_ID(orb_mag_scaled));
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Failed to subscribe to orb_mag_scaled topic.\n");
//     return;
//   }

//   /* Poll for new data */
//   struct pollfd fds;
//   fds.fd = fd;
//   fds.events = POLLIN;

//   while (1) // Infinite loop
//   {
//     if (poll(&fds, 1, -1) > 0) // Infinite timeout
//     {
//       if (fds.revents & POLLIN)
//       {
//         /* Copy the data from the orb */
//         ret = orb_copy(ORB_ID(orb_mag_scaled), fd, &mag_scaled);
//         if (ret < 0)
//         {
//           syslog(LOG_ERR, "ORB copy error, %d \n", ret);
//           continue;
//         }

//         // Print the received data
//         printf("Timestamp: %" PRIu64 "\n", mag_scaled.timestamp);
//         printf("Mag_X: %.4f Mag_Y: %.4f Mag_Z: %.4f\n", mag_scaled.mag_x, mag_scaled.mag_y, mag_scaled.mag_z);
//         printf("Acc_X: %.4f Acc_Y: %.4f Acc_Z: %.4f\n", mag_scaled.acc_x, mag_scaled.acc_y, mag_scaled.acc_z);
//         printf("Gyro_X: %.4f Gyro_Y: %.4f Gyro_Z: %.4f\n", mag_scaled.gyro_x, mag_scaled.gyro_y, mag_scaled.gyro_z);
//         printf("Temperature: %.2f\n", mag_scaled.temperature);
//       }
//     }
//     else
//     {
//       syslog(LOG_ERR, "Poll error.\n");
//     }
//   }

//   ret = orb_unsubscribe(fd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Orb unsubscribe Failed.\n");
//   }
//   return 0;
// }
void subscribe_and_retrieve_data(void)
{
  int fd;
  int ret;

  struct orb_mag_scaled_s mag_scaled;

  /* Subscribe to the orb_mag_scaled topic */
  fd = orb_subscribe(ORB_ID(orb_mag_scaled));
  if (fd < 0)
  {
    syslog(LOG_ERR, "Failed to subscribe to orb_mag_scaled topic.\n");
    return;
  }
  else
  {
    syslog(LOG_DEBUG, "[READING DATA FROM IMU SENSORS]\n");
  }

  /* Poll for new data */
  struct pollfd fds;
  fds.fd = fd;
  fds.events = POLLIN;

  while (1) // Infinite loop
  {
    if (poll(&fds, 1, -1) > 0) // Infinite timeout
    {
      if (fds.revents & POLLIN)
      {
        /* Copy the data from the orb */
        ret = orb_copy(ORB_ID(orb_mag_scaled), fd, &mag_scaled);
        if (ret < 0)
        {
          syslog(LOG_ERR, "ORB copy error, %d \n", ret);
          continue;
        }

        // Print the received data
        // printf("Timestamp: %" PRIu64 "\n", mag_scaled.timestamp);
        // printf("Mag_X: %.4f Mag_Y: %.4f Mag_Z: %.4f\n", mag_scaled.mag_x, mag_scaled.mag_y, mag_scaled.mag_z);
        // printf("Acc_X: %.4f Acc_Y: %.4f Acc_Z: %.4f\n", mag_scaled.acc_x, mag_scaled.acc_y, mag_scaled.acc_z);
        // printf("Gyro_X: %.4f Gyro_Y: %.4f Gyro_Z: %.4f\n", mag_scaled.gyro_x, mag_scaled.gyro_y, mag_scaled.gyro_z);
        // printf("Temperature: %.2f\n", mag_scaled.temperature);
        // sat_health.
        sat_health.accl_x = (int16_t)(mag_scaled.acc_x * 100);
        sat_health.accl_y = (int16_t)(mag_scaled.acc_y * 100);
        sat_health.accl_z = (int16_t)(mag_scaled.acc_z * 100);
        sat_health.gyro_x = (int16_t)(mag_scaled.gyro_x * 100);
        sat_health.gyro_y = (int16_t)(mag_scaled.gyro_y * 100);
        sat_health.gyro_z = (int16_t)(mag_scaled.gyro_z * 100);
        sat_health.mag_x = (int16_t)(mag_scaled.mag_x * 100);
        sat_health.mag_y = (int16_t)(mag_scaled.mag_y * 100);
        sat_health.mag_z = (int16_t)(mag_scaled.mag_z * 100);
        sat_health.temp_obc = mag_scaled.temperature * 100;
      }
    }
    else
    {
      syslog(LOG_ERR, "Poll error.\n");
    }
  }

  ret = orb_unsubscribe(fd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Orb unsubscribe Failed.\n");
  }
  return 0;
}
void print_beacon_a()
{
  printf("-------------------------------------------------------\nHEAD: 0x%02X\n", s2s_beacon_type_a.HEAD);
  printf("TYPE: %d\n", s2s_beacon_type_a.TYPE);
  printf("TIM_DAY: %d\n", s2s_beacon_type_a.TIM_DAY);
  printf("TIM_HOUR: %d\n", s2s_beacon_type_a.TIM_HOUR);
  printf("BAT_V: %d\n", s2s_beacon_type_a.BAT_V);
  printf("BAT_C: %d\n", s2s_beacon_type_a.BAT_C);
  printf("BAT_T: %d\n", s2s_beacon_type_a.BAT_T);
  printf("RAW_C: %d\n", s2s_beacon_type_a.RAW_C);
  printf("SOL_TOT_V: %d\n", s2s_beacon_type_a.SOL_TOT_V);
  printf("SOL_TOT_C: %d\n", s2s_beacon_type_a.SOL_TOT_C);
  printf("-------------------------------------------------------\n");
}
void print_beacon_b()
{
  printf("---- Beacon B Data ----\n");
  printf("HEAD: 0x%02X\n", s2s_beacon_type_b.HEAD);
  printf("TYPE: 0x%02X\n", s2s_beacon_type_b.TYPE);
  printf("TIM_DAY: 0x%02X\n", s2s_beacon_type_b.TIM_DAY);

  printf("\nSolar Panel Voltages:\n");
  printf("  SOL_P1_V: %d mV\n", s2s_beacon_type_b.SOL_P1_V);
  printf("  SOL_P2_V: %d mV\n", s2s_beacon_type_b.SOL_P2_V);
  printf("  SOL_P3_V: %d mV\n", s2s_beacon_type_b.SOL_P3_V);
  printf("  SOL_P4_V: %d mV\n", s2s_beacon_type_b.SOL_P4_V);

  printf("\nSolar Panel Currents:\n");
  printf("  SOL_P1_C: %d mA\n", s2s_beacon_type_b.SOL_P1_C);
  printf("  SOL_P2_C: %d mA\n", s2s_beacon_type_b.SOL_P2_C);
  printf("  SOL_P3_C: %d mA\n", s2s_beacon_type_b.SOL_P3_C);
  printf("  SOL_P4_C: %d mA\n", s2s_beacon_type_b.SOL_P4_C);

  printf("\nGyroscope Data (in deg/s * 100):\n");
  printf("  GYRO_X: %d deg/s\n", (s2s_beacon_type_b.GYRO_X));
  printf("  GYRO_Y: %d deg/s\n", (s2s_beacon_type_b.GYRO_Y));
  printf("  GYRO_Z: %d deg/s\n", (s2s_beacon_type_b.GYRO_Z));

  printf("\nAccelerometer Data (in m/s^2 * 100):\n");
  printf("  ACCL_X: %d\n", (s2s_beacon_type_b.ACCL_X));
  printf("  ACCL_Y: %d\n", (s2s_beacon_type_b.ACCL_Y));
  printf("  ACCL_Z: %d\n", (s2s_beacon_type_b.ACCL_Z));

  printf("\nMagnetometer Data (in uT):\n");
  printf("  MAG_X: %d uT\n", (s2s_beacon_type_b.MAG_X));
  printf("  MAG_Y: %d uT\n", (s2s_beacon_type_b.MAG_Y));
  printf("  MAG_Z: %d uT\n", (s2s_beacon_type_b.MAG_Z));

  printf("\nChecksum CRC: 0x%02X\n", s2s_beacon_type_b.CHK_CRC);
  printf("------------------------\n");
}

void new_camera_operation()
{
  struct file file_pointer, file_pointer2;
  uint8_t hand = 1;
  uint64_t counter1 = 0;
  uint64_t counter = 0, counter2 = 0;
  int32_t ret, count;
  uint8_t cam[37000] = {'\0'};

  uint8_t data2[] = {0x53, 0x01, 0x02, 0x03, 0x04, 0x7e, '\0'};
  turn_msn_on_off(2, 1);
  // usleep(1000000);
  sleep(2);
  int fd = open(CAM_UART, O_RDWR); // Open in non-blocking mode
  if (fd < 0)
  {
    printf("Error opening %s\n", CAM_UART);
    usleep(PRINT_DELAY);
    return -1;
  }

  // Writing handshake data
  for (int p = 0; p < 7; p++)
    ret = write(fd, data2[p], 1);
  if (ret > 0)
  {
    hand = 0;
  }
  // close(fd);
  // if (hand == 0)
  uint8_t data3[7] = {'\0'}, data4[7] = {'\0'}, uart_data, temp;

  for (int i = 0; i < 2; i++)
  {
    memset(data3, '\0', sizeof(data3));
    memset(cam, '\0', sizeof(cam));
    syslog(LOG_DEBUG, "Command %s sent\n", data2);
    int p = 0;
    counter = 0;
    // uint8_t cam[11500] = {'\0'};
    int fd2 = open(CAM_UART, O_RDONLY);
    // sleep(10);
    // sleep(1);
    ret = read(fd, data3, sizeof(data3));
    if (ret > 0)
    {
      printf("Handshake success %s", data3);
    }
    do
    {
      temp = uart_data;
      ret = read(fd, &uart_data, 1);
      if (ret > 0)
      {
        cam[counter1++] = uart_data;
        printf("%02x ", uart_data);
      }
      if (counter1 % 400 == 0)
      {
        pet_counter = 0;
      }
      // cam[counter1] = data3;
      if (temp == 0xff && uart_data == 0xd9)
      {
        // data4=000;
        break;
      }
    } while (1);
    close(fd);
    gpio_write(GPIO_SFM_MODE, false);
    if (i == 0)
    {
      fd = file_open(&file_pointer, "/mnt/fs/mfm/mtd_mission/cam_nir.txt", O_CREAT | O_RDWR | O_APPEND);
      if (file_seek(&file_pointer, 0, SEEK_END) > 16000000)
      {
        if (file_seek(&file_pointer, 0, SEEK_SET) >= 0)
        {
          printf("The camera nir.txt file size exceeded limit so setting seek pointer to 0\n");
        }
      }
      if (fd >= 0)
      {
        counter2 = file_write(&file_pointer, cam, counter1);
        printf("\n\nData of length %d has been written to camnir.txt\n\n", counter2);
        file_close(&file_pointer);
      }
    }
    else
    {
      file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mission/cam_rgb.txt", O_CREAT | O_WRONLY | O_APPEND);
      if (file_seek(&file_pointer, 0, SEEK_END) > 16000000)
      {
        if (file_seek(&file_pointer, 0, SEEK_SET) >= 0)
        {
          printf("The camera rgb.txt file size exceeded limit so setting seek pointer to 0\n");
        }
      }
      if (fd >= 0)
      {
        counter2 = file_write(&file_pointer2, cam, counter1);
        printf("\n\nData of length %d has been written to camrgb.txt\n\n", counter2);
        file_close(&file_pointer2);
      }
    }
    // printf("Data of length %d has been written to camnir.txt\n", counter2);
  }
  pet_counter = 0;
  // for (int i = 0; i < 25; i++)
  //   sleep(1);
  uint8_t data1[3500] = {'\0'};

  if (counter == 0)
  {
    file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mission/cam_nir.txt", O_CREAT | O_WRONLY | O_APPEND);
    if (fd >= 0)
    {
      counter2 = file_write(&file_pointer2, cam, counter1);
      printf("Data of length %d has been written to camnir.txt\n", counter2);
    }
    file_close(&file_pointer2);
  }
  else
  {
    do
    {
      if (file_seek(&file_pointer, counter2, SEEK_SET) >= 0)
      {
        if (counter - counter2 > 3500)
        {
          count = 3500;
        }
        else
        {
          count = counter - counter2;
        }
        counter2 += count;
        ret = file_read(&file_pointer, data1, count);
        printf("ret is %d %d", ret, counter - counter2);
        for (int32_t i = 0; i < count; i++)
        {
          printf("%02X ", data1[i]);
          if (count % 500 == 0)
          {
            usleep(4000);
          }
        }
        // if ( >= 0)
        gpio_write(GPIO_SFM_MODE, false);
        ret = open("/mnt/fs/mfm/mtd_mission/cam_nir.txt", O_WRONLY | O_APPEND);
        ssize_t writeBytes = write(ret, data1, count);
        close(ret);
        // ssize_t writeBytes = file_write(&file_pointer2, &data1, 4000);
        printf("The file of size %d has been written\n", writeBytes);
      }
    } while (counter2 < counter);
  }
  pet_counter = 0;

  // counter, count, counter2 = 0;
  // fd = file_open(&file_pointer, "/mnt/fs/sfm/mtd_mission/cam_rgb.txt", O_CREAT | O_RDWR | O_APPEND);
  // counter = file_seek(&file_pointer, 0, SEEK_END);
  // file_open(&file_pointer2, "/mnt/fs/mfm/mtd_mission/cam_rgb.txt", O_CREAT | O_WRONLY | O_APPEND);
  // counter2 = file_seek(&file_pointer2, 0, SEEK_END);
  // file_close(&file_pointer2);
  // do
  // {
  //   if (file_seek(&file_pointer, counter2, SEEK_SET) >= 0)
  //   {
  //     if (counter - counter2 > 8000)
  //     {
  //       count = 8000;
  //     }
  //     else
  //     {
  //       count = counter - counter2;
  //     }
  //     counter2 += count;
  //     ret = file_read(&file_pointer, data1, count);
  //     printf("ret is %d %d", ret, counter - counter2);
  //     for (int32_t i = 0; i < count; i++)
  //     {
  //       printf("%02X ", data1[i]);
  //       if (count % 500 == 0)
  //       {
  //         usleep(4000);
  //       }
  //     }
  //     // if ( >= 0)
  //     gpio_write(GPIO_SFM_MODE, false);
  //     ret = open("/mnt/fs/mfm/mtd_mission/cam_rgb.txt", O_WRONLY | O_APPEND);
  //     ssize_t writeBytes = write(ret, data1, count);
  //     close(ret);
  //     // ssize_t writeBytes = file_write(&file_pointer2, &data1, 4000);
  //     printf("The file of size %d has been written\n", writeBytes);
  //   }
  // } while (counter2 < counter);

  turn_msn_on_off(2, 0);
}

void handle_reservation_command(int fd_reservation, struct reservation_command res)
{
  if (critic_flags.OPER_MODE == NRML_MODE)
  {

    //   //  = orb_subscribe(ORB_ID(reservation_command));
    //   int updated;
    //   uint32_t t2;
    //   struct reservation_command res;
    //   orb_check(fd_reservation, &updated);
    //   // current_time = time(NULL);
    //   // t2 = (uint32_t)current_time;
    //   // if (sat_health.rsv_cmd == 0 && critic_flags.RSV_FLAG != 0x00)
    //   if (updated)
    {
      // t2 = 0;
      // t2 = orb_copy(ORB_ID(reservation_command), fd_reservation, &res);
      if (res.time[0] == 0 && res.time[1] == 0)
      {
        // res = {'\0'};
        // memset(res, '\0',sizeof(res));
        // res.cmd[0] = 0;
        // res.cmd[1] = 0;
        // res.cmd[2] = 0;
      }
      else
      {
        if (res.cmd[0] == TO_EXECUTE.cmd[0] && res.cmd[1] == TO_EXECUTE.cmd[1] && res.cmd[2] == TO_EXECUTE.cmd[2] && res.cmd[3] == TO_EXECUTE.cmd[3] && res.time[0] == TO_EXECUTE.time[0])
        {
          // timer =0;
        }
        else
        {
          TO_EXECUTE = res;

          // Get the current time
          // timer = 0;

          // sath_hea
        }
      }
      // if(res.latest_time - orb_absolute_time() <= 0){

      // }
    }
    uint32_t t1 = (uint32_t)TO_EXECUTE.cmd[3] << 8 | TO_EXECUTE.cmd[4];
    TO_EXECUTE.latest_time = t1 * 60;
    // t2 = TO_EXECUTE.latest_time - t2;
    if (TO_EXECUTE.mcu_id != 0 && TO_EXECUTE.mcu_id < 10)
    {
      RSV_CMD[16] = TO_EXECUTE.mcu_id;
      RSV_CMD[17] = TO_EXECUTE.cmd[0];
      RSV_CMD[18] = TO_EXECUTE.cmd[1];
      RSV_CMD[19] = TO_EXECUTE.cmd[2];
      printf("---------------------------------------------\n");
      printf("-_________________________________________________________________________\n");
      printf("Here we have reservation command as %s and time remaining is %d %d\n", RSV_CMD, timer, TO_EXECUTE.latest_time);
      printf("_________________________________________________________________________-\n");
      printf("---------------------------------------------\n");
      // TO_EXECUTE.cmd[0] = 0;
      // TO_EXECUTE.cmd[1] = 0;
      // TO_EXECUTE.cmd[2] = 0;
      // TO_EXECUTE.time[0] = 0;
      // TO_EXECUTE.time[1] = 0;

      res.cmd[0] = 0;
      res.cmd[1] = 0;
      res.cmd[2] = 0;
      res.time[0] = 0;
      res.time[1] = 0;

      sat_health.rsv_flag -= 1;
      sat_health.rsv_cmd = 0x00;
    }
  }
  if (RSV_CMD[16] != 0x00 && RSV_CMD[16] == TO_EXECUTE.mcu_id && RSV_CMD[17] != 0x00 && RSV_CMD[18] != 0x00 && timer > TO_EXECUTE.latest_time)
  {
    parse_command(&RSV_CMD);
    TO_EXECUTE.cmd[0] = 0;
    TO_EXECUTE.cmd[1] = 0;
    TO_EXECUTE.cmd[2] = 0;
    TO_EXECUTE.time[0] = 0;
    TO_EXECUTE.time[1] = 0;
    RSV_CMD[16] = 0;
    RSV_CMD[17] = 0;
    RSV_CMD[18] = 0;
    RSV_CMD[19] = 0;
    RSV_CMD[20] = 0;
    timer = 0;
    struct file file_ptr;
    uint16_t temp = 0;

    int fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, "/reservation_command.txt", O_RDONLY);
    struct reservation_command res_temp[16];
    uint16_t t = 0x00;

    uint16_t file_size = file_seek(&file_ptr, 0, SEEK_END);

    // Read reservation commands
    for (int i = 0; i < (file_size / 10); i++)
    {
      file_seek(&file_ptr, temp, SEEK_SET);
      temp += sizeof(struct reservation_command);
      if (file_read(&file_ptr, &res_temp[i], sizeof(struct reservation_command)) > 0)
      {
        // printf("Number of res command read is %d and temp is %d\n", i + 1, temp);
        // printf("_________________________________________\n");
        // printf(" MCU id : %02x Cmd[0] :%02x , Cmd[1]:%02x ,Cmd[2]:%02x, Cmd[3]:%02x Cmd[4]:%02x Cmd[5]:%d\n",
        //        res_temp[i].mcu_id, res_temp[i].cmd[0], res_temp[i].cmd[1], res_temp[i].cmd[2], res_temp[i].cmd[3], res_temp[i].cmd[4], t);
        // printf("_________________________________________\n");
      }
    }
    file_close(&file_ptr);

    int n = file_size / 10;

    {
      // Bubble sort, starting from index 1
      for (int i = 1; i < n - 1; i++)
      {
        for (int j = 1; j < n - i; j++)
        {
          uint16_t t1 = (uint16_t)res_temp[j].cmd[3] << 8 | res_temp[j].cmd[4];
          uint16_t t2 = (uint16_t)res_temp[j + 1].cmd[3] << 8 | res_temp[j + 1].cmd[4];
          if (t1 > t2)
          {
            // Swap res_temp[j] and res_temp[j + 1]
            struct reservation_command temp = res_temp[j];
            res_temp[j] = res_temp[j + 1];
            res_temp[j + 1] = temp;
          }
        }
      }

      // Print sorted commands, starting from index 1
      for (int i = 1; i < n; i++)
      {
        printf("Number of res command read is %d and temp is %d\n", i + 1, temp);
        printf("_________________Sorted________________________\n");
        printf("MCU id : %02x Cmd[0] :%02x , Cmd[1]:%02x ,Cmd[2]:%02x, Cmd[3]:%02x Time[0]:%02x Time[1]:%d executed:%d\n",
               res_temp[i].mcu_id, res_temp[i].cmd[0], res_temp[i].cmd[1], res_temp[i].cmd[2], res_temp[i].cmd[3], res_temp[i].cmd[4], res_temp[i].cmd[5], res_temp[i].executed, t);
        printf("__________________Sorted_______________________\n");
      }

      // Write sorted commands back to the file, starting from index 1

      if (n == 1)
      {
        fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, "/reservation_command.txt", O_TRUNC);
        file_close(&file_ptr);
      }
      else
      {
        fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, "/reservation_command.txt", O_TRUNC | O_WRONLY | O_APPEND);
        if (fd >= 0)
        {
          for (int i = 1; i < n; i++)
          {
            file_write(&file_ptr, &res_temp[i], sizeof(struct reservation_command));
          }
        }
        if (file_close(&file_ptr) < 0)
        {
          file_close(&file_ptr);
        }
      }
    }

    // sort_reservation_command(1, true);
  }
  // if(timer >= TO_EXECUTE.latest_time)
  // timer+=91;
}

void set_time(uint32_t received_timestamp)
{
  struct timespec ts;
  ts.tv_sec = received_timestamp;
  ts.tv_nsec = 0; // Set nanoseconds to zero
  int ret = clock_settime(CLOCK_REALTIME, &ts);
  if (ret < 0)
  {
    perror("clock_settime");
  }
  ret = clock_gettime(CLOCK_REALTIME, &ts);
  if (ret < 0)
  {
    perror("clock_gettime");
    return 1;
  }

  // Convert to struct tm
  struct tm *tm_info;
  tm_info = localtime(&ts.tv_sec);

  // Format and print the time
  char buffer[80];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", tm_info);
  printf("Current time: %s\n", buffer);
}

void save_64_bit()
{
  uint64_t test = 0;
  time_t now = time(NULL);
  test = now;
  struct file file1;
  file_open(&file1, "/mnt/fs/mfm/mtd_mainstorage/time.txt", O_CREAT | O_WRONLY);
  int tt = file_write(&file1, &test, sizeof(test));
  if (tt > 0)
  {
    printf("The time data has been updated accprdingly. %d\n", tt);
  }
  file_close(&file1);
}

uint64_t get_time_data()
{
  struct file file1;
  uint64_t test = 0, t;
  t = file_open(&file1, "/mnt/fs/mfm/mtd_mainstorage/time.txt", O_RDONLY);
  // ssize_t t = file_seek(&file1, 0, SEEK_END);
  file_read(&file1, &test, sizeof(test));
  file_close(&file1);
  printf("the latest size is %d data is %d\n", t, test);
  return test;
}

void flash_read_operation_uorb()
{
  int adc_instance = 10;
  int raw_afd = orb_advertise_multi_queue_persist(ORB_ID(command), &command_uorb,
                                                  &adc_instance, sizeof(struct command));
  int count = 0;
  {
    command_uorb.timestamp = (uint64_t)time(NULL);
    if (raw_afd < 0)
    {
      syslog(LOG_ERR, "Raw ADC advertise failed. %i\n", raw_afd);
    }
    else
    {
      if (OK != orb_publish(ORB_ID(command), raw_afd, &command_uorb))
      {
        syslog(LOG_ERR, "Orb Publish failed\n");
      }
      else
      {
        // syslog(LOG_DEBUG, "Reservation Command Orb Published\n");
        // printf("----Sent data : %d %d %d %d\n", command_uorb.timestamp, command_uorb.command, command_uorb.packet_number, command_uorb.num_of_packets);
        printf("\nData sent :\nTimestamp : %d \nPath: %s \nAddress:%d \nCommand:%d \nExecuted:%d \n",
               command_uorb.timestamp,
               command_uorb.path,
               command_uorb.command,
               command_uorb.num_of_packets,
               command_uorb.address,
               command_uorb.executed);
      }
    }
  }
  ret = orb_unadvertise(raw_afd);
}
void flash_operation_data(uint16_t loop) {
    struct flash_operation flash;
    int updated = 0;
    int flash_fd = orb_subscribe_multi(ORB_ID(flash_operation), 10);
    uint32_t start_time = time(NULL);
    uint32_t stop_time = 0;
   do{
        orb_check(flash_fd, &updated);
        if (updated) {
            orb_copy(ORB_ID(flash_operation), flash_fd, &flash);

            // Print the received flash operation data
            printf("\nUORB DATA\nTimestamp: %llu \nPacket_Type: %d\nPacket Number: %d\n",
                   flash.timestamp, flash.packet_type, flash.packet_number);

            for (int i = 0; i < 80; i++) {
                printf("%02x ", flash.data[i]);
            }
            send_flash_data(flash_data);
            printf("\n------------------------------------\n");

            // Reset the updated flag
            // updated = 0;
          loop--;
        }
        sleep(1);
        stop_time = time(NULL);
        FLASH_UORB_RESPONDING= true;
        if(stop_time - start_time > 400){
          FLASH_UORB_RESPONDING= false;
          break;
        }
        // printf("the value of counter is %d\n",loop);
    }while(loop>=0 && loop<65505);

    orb_unsubscribe(flash_fd);
}
