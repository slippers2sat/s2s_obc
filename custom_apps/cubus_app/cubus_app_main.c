/****************************************************************************
 * apps/custom_apps/spi_test/spi_test_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <nuttx/sensors/ioctl.h>

#include "adc.h"

#define IOCTL_MODE  1
// #define READ_MODE   1

#define MAX_CHANNELS  12                // maximum channel for external ADC
/****************************************************************************
 * Public Functions
 ****************************************************************************/
// struct 

/****************************************************************************
 * spi_test_main
 ****************************************************************************/
static void adc_devpath(FAR struct adc_state_s *adc, FAR const char *devpath);
static int int_adc_main();
static int ext_adc_main();


struct satellite_health_s {
	uint64_t timestamp;
	float accl_x;
	float accl_y;
	float accl_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
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
	uint16_t batt_volt;
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
	uint16_t v3_main_c;
	uint16_t v3_com_c;
	uint16_t v3_2_c;
	uint16_t v5_c;
	uint16_t unreg_c;
	uint16_t v4_c;
	int16_t batt_c;
	uint8_t rsv_cmd;
	uint8_t ant_dep_stat;
	uint8_t ul_state;
	uint8_t oper_mode;
	uint8_t msn_flag;
	uint8_t rsv_flag;
	uint8_t kill_switch;
};
/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct adc_state_s g_adcstate;

int elapsed =0;
int required  = 10;

struct adc_msg_s sample[CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE];
size_t readsize;
ssize_t nbytes;
int fd;
int errval = 0;
int ret;

uint8_t raw_data[2] = {'\0'};
uint16_t combined_data[MAX_CHANNELS] = {'\0'};

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  printf("Custom_apps CUBUS external ADC: %d\n", CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC);
  #if defined(CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC)
    ext_adc_main();
  #endif  //CONFIG_CUSTOM_APPS_CUBUS_USE_EXT_ADC

  #ifdef CONFIG_CUSTOM_APPS_CUBUS_USE_INT_ADC
    int_adc_main();
  #endif  //CUSTOM_APPS_CUBUS_USE_INT_ADC

  return 0;
}



/****************************************************************************
 * Name: adc_devpath
 ****************************************************************************/

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

/****************************************************************************
 * Name: adc_main
 ****************************************************************************/

static int int_adc_main()
{
  UNUSED(ret);

  /* Check if we have initialized */

  if (!g_adcstate.initialized)
    {
      /* Initialization of the ADC hardware must be performed by
       * board-specific logic prior to running this test.
       */

      /* Set the default values */

      adc_devpath(&g_adcstate, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_DEVPATH);

      g_adcstate.initialized = true;
    }

  g_adcstate.count = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_NSAMPLES;

  /* Parse the command line */

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

  printf("adc_main: g_adcstate.count: %d\n", g_adcstate.count);

  /* Open the ADC device for reading */

  printf("adc_main: Hardware initialized. Opening the ADC device: %s\n",
         g_adcstate.devpath);
  
  fd = open(CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_DEVPATH, O_RDONLY);
  if (fd < 0)
    {
      printf("adc_main: open %s failed: %d\n", g_adcstate.devpath, errno);
      errval = 2;
      goto errout;
    }
    elapsed = 0;
    while(elapsed<required){
        usleep(1000);
        elapsed++;
    }
  /* Now loop the appropriate number of times, displaying the collected
   * ADC samples.
   */
  // UNUSED(elapsed);
  // UNUSED(required);
  for (int i =0; i <g_adcstate.count+1;i++ )
    {
      /* Flush any output before the loop entered or from the previous pass
       * through the loop.
       */
      elapsed = 0;
      fflush(stdout);

#ifdef CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_SWTRIG
      /* Issue the software trigger to start ADC conversion */

      ret = ioctl(fd, ANIOC_TRIGGER, 0);
      if (ret < 0)
        {
          int errcode = errno;
          printf("adc_main: ANIOC_TRIGGER ioctl failed: %d\n", errcode);
        }
#endif

      /* Read up to CONFIG_CUSTOM_APPS_ADC_GROUPSIZE samples */

      readsize = CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE * sizeof(struct adc_msg_s);
      nbytes = read(fd, sample, readsize);

      printf(" Readsize: %d \n nbytes: %d\n CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE : %d \n ADCSTATE READCOUNT: %d \r\n",readsize, nbytes, CONFIG_CUSTOM_APPS_CUBUS_INT_ADC_GROUPSIZE, g_adcstate.count);

      // if(nbytes < 40){
      //   printf("nbytes is less than 40. Reading again \n");
      //   nbytes = read(fd, sample, readsize);
      // }
      /* Handle unexpected return values */

      if (nbytes < 0)
        {
          errval = errno;
          if (errval != EINTR)
            {
              printf("adc_main: read %s failed: %d\n",
                     g_adcstate.devpath, errval);
              errval = 3;
              goto errout_with_dev;
            }

          printf("adc_main: Interrupted read...\n");
        }
      else if (nbytes == 0)
        {
          printf("adc_main: No data read, Ignoring\n");
        }

      /* Print the sample data on successful return */

      else
        {
          int nsamples = nbytes / sizeof(struct adc_msg_s);
          if (nsamples * sizeof(struct adc_msg_s) != nbytes)
            {
              printf("adc_main: read size=%ld is not a multiple of "
                     "sample size=%d, Ignoring\n",
                     (long)nbytes, sizeof(struct adc_msg_s));
            }
          else
            {
              printf("Sample:\n");
              for (int i = 0; i < nsamples; i++)
                {
                  printf("%d: channel: %d value: %" PRId32 "\n",
                         i + 1, sample[i].am_channel, sample[i].am_data);
                }
            }
        }

      if (g_adcstate.count && --g_adcstate.count <= 0)
        {
          break;
        }
    }
  close(fd);
  return OK;

  /* Error exits */

errout_with_dev:
  close(fd);

errout:
  printf("Terminating!\n");
  fflush(stdout);
  close(fd);
  return errval;
}

static int ext_adc_main(){
   printf("Going to Test the External ADC\n");
  fd = open(EXT_ADC_PATH, O_RDONLY);
  if(fd < 0){
    printf("Unable to open external ADC driver\n");
    return -1;
  }
  printf("opened external ADC driver successfully\n Setting Manual Select mode...\n");

  /* Get the set of BUTTONs supported */
  ret = ioctl(fd, ANIOC_ADC_MANUAL_SELECT, NULL);
  usleep(10);

  printf("Setting ADC Select mode ... \n");
  ret = ioctl(fd, ANIOC_ADC_AUTO_2_SELECT, NULL);
  usleep(1000);

  printf("Setting ADC Program mode ...\n");
  ret = ioctl(fd, ANIOC_ADC_AUTO_2_PROGRAM, NULL);
  usleep(1000);

  #ifdef IOCTL_MODE
  for(int i=0;i<MAX_CHANNELS;i++){
    printf("Reading data from ADC %i \n", i);
    ioctl(fd, ANIOC_ADC_AUTO_2_SELECT_READ,raw_data);
    combined_data[i] = raw_data[0] << 8 | raw_data[1];
    printf("Raw data: %x \n",combined_data[i]);
    // usleep(100);
  }

  #else //ifndef IOCTL MODE
  for (int i=0;i<MAX_CHANNELS;i++){
    int ret1 = read(fd, &raw_data, 2);
    if(ret1<0){
      printf("Data not received from ADC");
      return -1;
    }
    printf("No of Bytes available: %d",ret1);
    combined_data[i] = raw_data[0] << 8 | raw_data[1];
    printf("\n\n\n");
  }
  #endif  //IOCTL MODE
  return 0;
}