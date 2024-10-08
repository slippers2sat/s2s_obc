
#include <nuttx/config.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <nuttx/timers/watchdog.h>  // Add this line
#include <errno.h>
#include <syslog.h>

/* Watchdog refresh interval in seconds */
#define WATCHDOG_REFRESH_INTERVAL 1

int configure_watchdog1(int fd, int timeout)
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

void watchdog_refresh_task1(int fd)
{
  while (1)
  {
    /* Refresh the watchdog */
    int ret = ioctl(fd, WDIOC_KEEPALIVE, 0);  // Use WDIOC_KEEPALIVE
    if (ret < 0)
    {
      perror("Failed to refresh watchdog");
    }

    /* Wait for the next refresh interval */
    sleep(WATCHDOG_REFRESH_INTERVAL);
  }
}

int main(int argc, FAR char *argv[])
{
  int fd = open("/dev/iwdgO", O_RDWR);
  if (fd < 0)
  {
    perror("Failed to open watchdog device");
    return 1;
  }

  /* Set the watchdog timeout to 10 seconds */
  if (configure_watchdog(fd, 10) < 0)
  {
    close(fd);
    return 1;
  }

  /* Create the watchdog refresh task */
  int ret = task_create("watchdog_refresh", SCHED_PRIORITY_DEFAULT,
                        CONFIG_DEFAULT_TASK_STACKSIZE, (main_t)watchdog_refresh_task1,
                        (FAR char * const *)&fd);
  if (ret < 0)
  {
    int errcode = errno;
    syslog(LOG_ERR, "[watchdog] ERROR: Failed to start watchdog refresh task: %d\n", errcode);
    close(fd);
    return 1;
  }

  /* Your main application code here */
  while (1)
  {
    printf("Main application running...\n");
    sleep(2);
  }

  close(fd);
  return 0;
}
