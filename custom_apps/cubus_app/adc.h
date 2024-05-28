/****************************************************************************
 * apps/custom_apps/cubus_app/adc.h
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

#ifndef __APPS_CUSTOM_APPS_ADC_ADC_H
#define __APPS_CUSTOM_APPS_ADC_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/ads7953.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_NSH_BUILTIN_APPS - Build the ADC test as an NSH built-in function.
 *  Default: Built as a standalone program
 * CONFIG_CUSTOM_APPS_ADC_DEVPATH - The default path to the ADC device.
 *   Default: /dev/adc0
 * CONFIG_CUSTOM_APPS_ADC_NSAMPLES - This number of samples is
 *   collected and the program terminates.  Default:  Samples are collected
 *   indefinitely.
 * CONFIG_CUSTOM_APPS_ADC_GROUPSIZE - The number of samples to read at once.
 *   Default: 4
 */

#ifndef CONFIG_ADC
#  error "ADC device support is not enabled (CONFIG_ADC)"
#endif

#ifndef CONFIG_CUSTOM_APPS_ADC_DEVPATH
#  define CONFIG_CUSTOM_APPS_ADC_DEVPATH "/dev/adc0"
#endif

#ifndef CONFIG_CUSTOM_APPS_ADC_GROUPSIZE
#  define CONFIG_CUSTOM_APPS_ADC_GROUPSIZE 4
#endif

#ifdef CONFIG_STM32_ADC1
#define ADC1_PATH   "/dev/adc0"
#endif  //CONFIG_STM32_ADC1

#ifdef CONFIG_STM32_ADC2
#define ADC2_PATH   "/dev/adc1"
#endif

#ifdef CONFIG_STM32_ADC3
#define ADC3_PATH   "dev/adc2"
#endif  // CONFIG_STM32_ADC3

#define EXT_ADC_PATH    "/dev/ext_adc0"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct adc_state_s
{
  bool      initialized;
  FAR char *devpath;
  int       count;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __APPS_CUSTOM_APPS_ADC_ADC_H */