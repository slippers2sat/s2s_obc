#ifndef __ADS7953_RAW_MSG_H
#define __ADS7953_RAW_MSG_H

// #include <nuttx/uORB/uORB.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <nuttx/analog/ads7953.h>
#include <nuttx/analog/ioctl.h> 
#include <nuttx/sensors/adc.h>

struct ads7953_raw_msg
{
  uint64_t timestamp;
  uint16_t volts_chan[7];
  float volts_chan_volts[7];
  uint16_t temp_chan[8];
  float temp_chan_volts[8];
};

#ifdef CONFIG_DEBUG_UORB
void print_ads7953_raw_msg(FAR const struct orb_metadata *meta, FAR const void *buffer);
#endif

ORB_DECLARE(ads7953_raw_msg);

#endif // __ADS7953_RAW_MSG_H
