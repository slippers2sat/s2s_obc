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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/board.h>

#include "stm32f427a.h"
#include "stm32.h"

#include <arch/board/board.h>

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

    // Initialize GPIO pin
    // gpio_pin_configure(GPIO_PIN, GPIO_DIRECTION_OUT);
    // stm32_gpioconfig(GPIO_WD_WDI);
    stm32_gpiowrite(GPIO_WD_WDI,gpio_state?true:false);
    

    while (1)
    {
        // Toggle GPIO state every 500ms
        gpio_state = !gpio_state;
        stm32_gpiowrite(GPIO_WD_WDI,gpio_state);
        

        // printf("GPIO state: %d\n", gpio_state);  // Optional: print the GPIO state

        // Wait for 500ms before toggling again
        usleep(500000);  // 500ms delay
    }
}
/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int stm32_wdg_setup(void)
{
  pid_t pid = task_create("[WDT_toggle_task]", 100, 804, wdt_toggle_task, NULL);
    if (pid < 0)
    {
        printf("ERROR: Failed to create wdt_toggle_task\n");
        // return -1;
        pid = task_create("[WDT_toggle_task]", 90, 1804, wdt_toggle_task, NULL);
    }

    printf("[WDT_Toggle_Task]WDT toggle task created successfully\n");

  return OK;
}
