#ifndef __APPS_EXAMPLES_WATCHDOG_MAIN_H
#define __APPS_EXAMPLES_WATCHDOG_MAIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVNAME_SIZE 16

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct wdog_example_s
{
  uint32_t pingtime;
  uint32_t pingdelay;
  uint32_t timeout;
  char devname[DEVNAME_SIZE];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: wdog_help
 *
 * Description:
 *   Show help information for the watchdog example application.
 *
 ****************************************************************************/

void wdog_help(void);

/****************************************************************************
 * Name: arg_string
 *
 * Description:
 *   Extract a string argument from the command line.
 *
 * Input Parameters:
 *   arg - Pointer to the command line argument
 *   value - Pointer to the extracted value
 *
 * Returned Value:
 *   Number of arguments consumed
 *
 ****************************************************************************/

int arg_string(FAR char **arg, FAR char **value);

/****************************************************************************
 * Name: arg_decimal
 *
 * Description:
 *   Extract a decimal argument from the command line.
 *
 * Input Parameters:
 *   arg - Pointer to the command line argument
 *   value - Pointer to the extracted value
 *
 * Returned Value:
 *   Number of arguments consumed
 *
 ****************************************************************************/

int arg_decimal(FAR char **arg, FAR long *value);

/****************************************************************************
 * Name: parse_args
 *
 * Description:
 *   Parse command line arguments for the watchdog example application.
 *
 * Input Parameters:
 *   wdog - Pointer to the watchdog example state structure
 *   argc - Number of arguments
 *   argv - Array of arguments
 *
 ****************************************************************************/

void parse_args(FAR struct wdog_example_s *wdog, int argc, FAR char **argv);

#endif /* __APPS_EXAMPLES_WATCHDOG_MAIN_H */
