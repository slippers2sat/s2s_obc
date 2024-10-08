/****************************************************************************
 * apps/examples/watchdog/watchdog_main.c
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
#include "watchdog.h"
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/timers/watchdog.h>

#include "watchdog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME_SIZE 16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct wdog_example_s
{
  uint32_t pingtime;
  uint32_t pingdelay;
  uint32_t timeout;
  char devname[DEVNAME_SIZE];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void wdog_help(void)
{
  printf("Usage: wdog [-h] [-d <pingtime] [-p <pingdelay>]\n");
  printf("  [-t <timeout>]\n");
  printf("\nInitialize the watchdog to the <timeout>. Start the watchdog\n");
  printf("timer.  Ping for the watchdog for <pingtime> seconds\n");
  printf("then let it expire.\n");
  printf("\nOptions include:\n");
  printf("  [-d <pingtime>] = Selects the <delay> time in milliseconds.\n");
  printf("Default: %d\n", CONFIG_EXAMPLES_WATCHDOG_PINGTIME);
  printf("  [-i </dev/watchdogx>] = Selects the watchdog timer instance.\n");
  printf("Default: %s\n", CONFIG_EXAMPLES_WATCHDOG_DEVPATH);
  printf("  [-p <pingdelay] = Time delay between pings in milliseconds.\n");
  printf("Default: %d\n", CONFIG_EXAMPLES_WATCHDOG_PINGDELAY);
  printf("  [-t timeout] = Time in milliseconds that the example will\n");
  printf("ping the watchdog before letting the watchdog expire.\n");
  printf("Default: %d\n", CONFIG_EXAMPLES_WATCHDOG_TIMEOUT);
  printf("  [-h] = Shows this message and exits\n");
}

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
  {
    *value = arg[1];
    return 2;
  }
  else
  {
    *value = &ptr[2];
    return 1;
  }
}

static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

static void parse_args(FAR struct wdog_example_s *wdog, int argc, FAR char **argv)
{
  FAR char *ptr;
  FAR char *string;
  long value;
  int index;
  int nargs;

  wdog->pingtime = CONFIG_EXAMPLES_WATCHDOG_PINGTIME;
  wdog->pingdelay = CONFIG_EXAMPLES_WATCHDOG_PINGDELAY;
  wdog->timeout = CONFIG_EXAMPLES_WATCHDOG_TIMEOUT;
  strlcpy(wdog->devname, CONFIG_EXAMPLES_WATCHDOG_DEVPATH, sizeof(wdog->devname));

  for (index = 1; index < argc;)
  {
    ptr = argv[index];
    if (ptr[0] != '-')
    {
      printf("Invalid options format: %s\n", ptr);
      exit(EXIT_SUCCESS);
    }

    switch (ptr[1])
    {
    case 'p':
      nargs = arg_decimal(&argv[index], &value);
      if (value < 1)
      {
        printf("Ping delay out of range: %ld\n", value);
        exit(EXIT_FAILURE);
      }

      wdog->pingdelay = (uint32_t)value;
      index += nargs;
      break;

    case 'i':
      nargs = arg_string(&argv[index], &string);
      strlcpy(wdog->devname, string, sizeof(wdog->devname));
      index += nargs;
      break;

    case 'd':
      nargs = arg_decimal(&argv[index], &value);
      if (value < 1 || value > INT_MAX)
      {
        printf("Ping time out of range: %ld\n", value);
        exit(EXIT_FAILURE);
      }

      wdog->pingtime = (uint32_t)value;
      index += nargs;
      break;

    case 't':
      nargs = arg_decimal(&argv[index], &value);
      if (value < 1 || value > INT_MAX)
      {
        printf("Duration out of range: %ld\n", value);
        exit(EXIT_FAILURE);
      }

      wdog->timeout = (int)value;
      index += nargs;
      break;

    case 'h':
      wdog_help();
      exit(EXIT_SUCCESS);

    default:
      printf("Unsupported option: %s\n", ptr);
      wdog_help();
      exit(EXIT_FAILURE);
    }
  }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

// int main(int argc, FAR char *argv[])
// {
//   struct wdog_example_s wdog;
//   int fd;
//   int ret;

//   /* Parse the command line */
//   parse_args(&wdog, argc, argv);

//   /* Open the watchdog device for reading */
//   fd = open(wdog.devname, O_RDONLY);
//   if (fd < 0)
//   {
//     printf("wdog_main: open %s failed: %d\n", wdog.devname, errno);
//     goto errout;
//   }

//   /* Set the watchdog timeout */
//   ret = ioctl(fd, WDIOC_SETTIMEOUT, (unsigned long)wdog.timeout);
//   if (ret < 0)
//   {
//     printf("wdog_main: ioctl(WDIOC_SETTIMEOUT) failed: %d\n", errno);
//     goto errout_with_dev;
//   }

//   /* Start the watchdog timer */
//   ret = ioctl(fd, WDIOC_START, 0);
//   if (ret < 0)
//   {
//     printf("wdog_main: ioctl(WDIOC_START) failed: %d\n", errno);
//     goto errout_with_dev;
//   }

//   /* Ping the watchdog every 2 seconds */
//   while (1)
//   {
//     /* Sleep for 2 seconds */
//     sleep(2);

//     /* Refresh the watchdog */
//     ret = ioctl(fd, WDIOC_KEEPALIVE, 0);
//     if (ret < 0)
//     {
//       printf("wdog_main: ioctl(WDIOC_KEEPALIVE) failed: %d\n", errno);
//       goto errout_with_dev;
//     }

//     printf("Watchdog refreshed\n");
//   }

//   /* Stop the watchdog (if supported) */
//   ret = ioctl(fd, WDIOC_STOP, 0);
//   if (ret < 0)
//   {
//     printf("wdog_main: ioctl(WDIOC_STOP) failed: %d\n", errno);
//   }

//   close(fd);
//   return OK;

// errout_with_dev:
//   close(fd);
// errout:
//   return ERROR;
// }
