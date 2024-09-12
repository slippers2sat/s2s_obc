
/****************************************************************************
 * custom_apps/storage_manager/storage_manager_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <uORB/uORB.h>
#include <sensor/mag.h>
#include <nuttx/sensors/sensor.h>
// #include <sensor/adc.h>
static struct work_s work_storage;

static bool g_storage_manager_started;

void read_and_print_mag_data(void)
{

  //adc
  // struct ads7953_raw_msg e_ads7953_0;
  // struct sat_temp_msg sat_temps;
  // struct sat_volts_msg sat_volts;
  // int ads,sat_temp,sat_volt;
  //
  int sub_fd;
  struct orb_mag_scaled_s mag_data;
  bool updated;

  sub_fd = orb_subscribe(ORB_ID(orb_mag_scaled));
  // ads = orb_subscibe(ORB_ID(e_ads7953_0));
  if (sub_fd < 0)
  {
    syslog(LOG_ERR, "Failed to subscribe to orb_mag_scaled topic\n");
    return;
  }

  // while (1)
  {
    orb_check(sub_fd, &updated);
    if (updated)
    {
      orb_copy(ORB_ID(orb_mag_scaled), sub_fd, &mag_data);

      syslog(LOG_INFO, "\n\n\r****-------[Storage maanager ]Mag Data:\n");

      syslog(LOG_INFO, "  ACCELEROMETER X: %.4f Y: %.4f Z: %.4f\n", mag_data.acc_x, mag_data.acc_y, mag_data.acc_z);
      syslog(LOG_INFO, "  GYROSCOPE X: %.4f Y: %.4f Z: %.4f\n", mag_data.gyro_x, mag_data.gyro_y, mag_data.gyro_z);

      syslog(LOG_INFO, "  MAGNETOMETER X: %.4f Y: %.4f Z: %.4f\n", mag_data.mag_x, mag_data.mag_y, mag_data.mag_z);
      syslog(LOG_INFO, "  Temp:  %.4f\n", mag_data.temperature);
    }
    // sleep(20); // Sleep for 500 ms
  }

  orb_unsubscribe(sub_fd);
  if(  work_queue(HPWORK, &work_storage, read_and_print_mag_data, NULL, SEC2TICK(30)) >=0){
    syslog(LOG_DEBUG, "Workqueue running\n");
  }
}

int storage_manager_daemon(int argc, FAR char *argv[])
{
  g_storage_manager_started = true;
  read_and_print_mag_data();
  return 0;
}

int main(int argc, FAR char *argv[])
{
  int ret;

  printf("[storage_manager] Starting task.\n");
  if (g_storage_manager_started)
  {
    printf("[storage_manager] Task already started.\n");
    return EXIT_SUCCESS;
  }

  // ret = task_create("storage_manager_daemon", SCHED_PRIORITY_DEFAULT, CONFIG_CUSTOM_APPS_STORAGE_MANAGER_STACKSIZE, storage_manager_daemon, NULL);
  ret = task_create("storage_manager_daemon", SCHED_PRIORITY_DEFAULT, CONFIG_CUSTOM_APPS_STORAGE_MANAGER_STACKSIZE, storage_manager_daemon, NULL);

  if (ret < 0)
  {
    int errcode = errno;
    printf("[storage_manager] ERROR: Failed to start storage_manager_daemon: %d\n", errcode);
    return EXIT_FAILURE;
  }

  printf("[storage_manager] storage_manager_daemon started\n");
  return EXIT_SUCCESS;
}
