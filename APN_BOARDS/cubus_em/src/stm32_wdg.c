/****************************************************************************
 * boards/arm/stm32/stm32f42a-minimal/src/stm32_wdg.c
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

// #include <sys/types.h>
// #include <errno.h>
// #include <debug.h>
#include <nuttx/board.h>
// #include <syslog.h>
#include <nuttx/kthread.h> 

#include "stm32f427a.h"
#include "stm32.h"

#include <arch/board/board.h>
bool wdog_task_started = false;


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void wdt_toggle_task(void *arg)
{
  static bool gpio_state = true;
  stm32_gpiowrite(GPIO_WD_WDI, gpio_state);
  stm32_gpiowrite(GPIO_WD_WDI, !gpio_state);
  irqstate_t flags;
  while (1)
  {
    // Toggle GPIO state every 500ms
    // flags = enter_critical_section();
    stm32_gpiowrite(GPIO_WD_WDI, gpio_state);
    // syslog(LOG_DEBUG,"\nGPio toggle state is %d\n", gpio_state);
    gpio_state = !gpio_state;
    // leave_critical_section(flags);
    usleep(500000);

    // sleep(1); // 500ms delay
  }
}
/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/
int toggle_wdg(){
  stm32_configgpio(GPIO_WD_WDI);
  stm32_gpiowrite(GPIO_WD_WDI, true);
  usleep(500000);
  stm32_gpiowrite(GPIO_WD_WDI, false);
  syslog(LOG_DEBUG, "TOggled wdg");
}

int stm32_wdg_setup(void)
{
  stm32_configgpio(GPIO_WD_WDI);

  stm32_gpiowrite(GPIO_WD_WDI, true);
  usleep(10000);
  stm32_gpiowrite(GPIO_WD_WDI, false);
  // syslog(LOG_DEBUG,"GPio setup");


  if (wdog_task_started == false)
  {
    // pid_t pid = task_create("[WDT_toggle_task]", 1, 904, wdt_toggle_task, NULL);
    pid_t pid = kthread_create(
    "WDT TOggle thread",      // Thread name
    50,  // Highest priority
    905,                  // Stack size
    wdt_toggle_task,     // Entry function
    NULL                 // Argument
    );

    if (pid < 0)
    {
      //printf("ERROR: Failed to create wdt_toggle_task\n");
      // return -1;
      pid = task_create("[WDT_toggle_task]", 1, 1204, wdt_toggle_task, NULL);
    }
    else
    {
      wdog_task_started = true;
      // syslog(LOG_DEBUG,"[WDT_Toggle_Task]WDT toggle task created successfully\n");
      stm32_gpiowrite(GPIO_WD_WDI, true);
      stm32_gpiowrite(GPIO_WD_WDI, false);
    }
  }
  else
  {
    printf("[WDT_Toggle_Task]WDT toggle task already created\n");
  }
  return OK;
}
