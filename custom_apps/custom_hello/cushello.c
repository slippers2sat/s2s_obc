#include <nuttx/config.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <nuttx/wdog.h>
#include <sys/ioctl.h>
#include <nuttx/timers/watchdog.h>

int main(int argc, char *argv[])
{
    int wd;
    int timeout = 5;  // Set a timeout of 5 seconds

    // Open the watchdog device
    wd = open("/dev/wwdg0", O_WRONLY);
    if (wd < 0)
    {
        printf("ERROR: Failed to open watchdog: %d\n", errno);
        return -1;
    }

    // Set the timeout (if supported)
    ioctl(wd, WDIOC_SETTIMEOUT, &timeout);

    printf("Watchdog started with %d seconds timeout.\n", timeout);

    // Here we wait to test the watchdog
    sleep(timeout + 9);  // Wait longer than the timeout to trigger the watchdog

    // The watchdog should reset the system if not fed in time
    // Uncomment the next line to feed the watchdog
    // ioctl(wd, WDIOC_KEEPALIVE, 0); // Uncomment to prevent reset

    // Close the watchdog device (not reached if the watchdog works)
    close(wd);
    return 0;
}
