#include <nuttx/config.h>
#include <nuttx/timers/rtc.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <time.h>

// STM32-specific headers
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <arch/chip/chip.h>

// Define the RTC device path
#define RTC_DEVICE "/dev/rtc0"

void rtc_init(void)
{
    // Ensure that the backup domain is accessible
    stm32_pwr_enablebkp(true);

    // Initialize the RTC hardware
    struct rtc_lowerhalf_s *lower = stm32_rtc_initialize();
    if (!lower)
    {
        printf("Failed to initialize STM32 RTC\n");
        return;
    }

    // Register the RTC driver at /dev/rtc0
    int ret = rtc_initialize(0, lower);
    if (ret < 0)
    {
        printf("Failed to register RTC driver: %d\n", ret);
        return;
    }

    // Open the RTC device to verify it's working
    int rtc_fd = open(RTC_DEVICE, O_RDWR);
    if (rtc_fd < 0)
    {
        printf("Failed to open RTC device: %d\n", errno);
        return;
    }

    // Example: Read the current RTC time to verify it works
    struct rtc_time current_time;
    if (ioctl(rtc_fd, RTC_RD_TIME, &current_time) < 0)
    {
        printf("Failed to read RTC time: %d\n", errno);
        close(rtc_fd);
        return;
    }

    printf("Current RTC time: %02d:%02d:%02d\n", current_time.tm_hour, current_time.tm_min, current_time.tm_sec);

    // Close the RTC device
    close(rtc_fd);
}
