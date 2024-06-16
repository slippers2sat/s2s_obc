/****************************************************************************
 * custom_apps/custom_hello/custom_hello.c
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
#include "cushello.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * custom_hello_main
 ****************************************************************************/

int  main(int argc, FAR char *argv[])
{
  
  syslog(LOG_INFO, "Hello World application for writing data to flash.\n");

  syslog(LOG_INFO, "opening a uORB topic to subscribe messages.\n");

  struct sensor_mag mag0;
  struct pollfd fds;
  int fd;
  int ret;
  int i;

  fd = orb_subscribe_multi(ORB_ID(sensor_mag), 0);

  fds.fd = fd;
  fds.events = POLLIN;

  while (1)
  {
    if (poll(&fds, 1, 3000) > 0)
    {
      if(fds.revents & POLLIN)
      {
        ret = orb_copy_multi(fd, &mag0, sizeof(struct sensor_mag));
        if (ret < 0)
        {
          syslog(LOG_ERR, "ORB copy error, %d \n", ret);
          return ret;
        }

        syslog(LOG_INFO, "Copied data from orb_object.\n");

        printf("Timestamp: %lli \t", mag0.timestamp);
        printf("Temperature: %0.02f \t", mag0.temperature);
        printf("X : %0.02f \t", mag0.x);
        printf("Y : %0.02f \t", mag0.y);
        printf("Z : %0.02f \t\n", mag0.z);

      }
    }
  }
  
  return 0;
}
