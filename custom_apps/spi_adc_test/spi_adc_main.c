/****************************************************************************
 * custom_apps/spi_adc_test/spi_test_main.c
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
#include <stdio.h>
#include <assert.h>
#include <fcntl.h>

#include <nuttx/sensors/lis3mdl.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/analog/ads7953.h>

#define IOCTL_MODE  1
// #define READ_MODE   1

#define MAX_CHANNELS  12
#define EXT_ADC_PATH  "/dev/ext_adc0"
/****************************************************************************
 * Public Functions
 ****************************************************************************/
// struct 

/****************************************************************************
 * spi_test_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  uint8_t raw_data[2] = {'\0'};
  uint16_t combined_data[MAX_CHANNELS] = {'\0'};
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
  #endif  //if not defined IOCTL MODE

  return 0;
}