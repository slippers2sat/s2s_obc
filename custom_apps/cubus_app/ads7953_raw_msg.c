#include <uORB/uORB.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <poll.h>
#include <sensor/adc.h>
#include <nuttx/sensors/sensor.h>


#include <nuttx/config.h>
#include <stdio.h>
#include <syslog.h>

#include <time.h>
#include <fcntl.h>

#include <poll.h>
#include <sched.h>
#include <math.h>

#define VOLT_DIV_RATIO ((1100 + 931) / 931) // ratio of voltage divider used
      float x[8], y[8];


void ADC_Temp_Conv(float *adc_conv_buf, float *temp_buf, int channel) {
    float root = 0;

    if (channel == 16) { // Battery temperature channel
        float res = (adc_conv_buf[1] * 10000) / (2.5 * adc_conv_buf[1]);
        float tempk = 3976 * 298 / (3976 + (298 * log10(10000 / res)));
        temp_buf[1] = (tempk - 273) * 100;
    } else {
        root = sqrtf(
            (5.506 * 5.506) +
            (4 * 0.00176 * (870.6 + (adc_conv_buf[1] * 1000)))
        );
        temp_buf[1] = (((5.506 * root) / (2 * (-0.00176))) - 30) * 100;
    }
}
void convert1(data){
   float root = sqrtf(
            (5.506 * 5.506) +
            (4 * 0.00176 * (870.6 + (data * 1000)))
        );
        float result= (((5.506 * root) / (2 * (-0.00176))) - 30) * 100;
}


int ads7953_receiver(int argc, FAR char *argv[])
{
  int raw_sub_fd = orb_subscribe(ORB_ID(ads7953_raw_msg));
  int temp_sub_fd = orb_subscribe(ORB_ID(sat_temp_msg));
  int volts_sub_fd = orb_subscribe(ORB_ID(sat_volts_msg));

  struct pollfd fds[] = {
    { .fd = raw_sub_fd, .events = POLLIN },
    { .fd = temp_sub_fd, .events = POLLIN },
    { .fd = volts_sub_fd, .events = POLLIN },
  };

  while (1)
  {
    // Poll for new data
    int poll_ret = poll(fds, 3, 1000);

    if (poll_ret == 0)
    {
      printf("Poll timeout\n");
      continue;
    }

    if (poll_ret < 0)
    {
      printf("Poll error: %d\n", errno);
      continue;
    }

    // Check for ads7953_raw_msg updates
    if (fds[0].revents & POLLIN)
    {
      struct ads7953_raw_msg raw_msg;
      orb_copy(ORB_ID(ads7953_raw_msg), raw_sub_fd, &raw_msg);

      // printf("ads7953_raw_msg:\n");
      // printf("  timestamp: %" PRIu64 "\n", raw_msg.timestamp);
      for (int i = 0; i < 7; ++i)
      {
        // printf("  Voltage Channel %d: %u (%.4f V)\n", i, raw_msg.volts_chan[i], raw_msg.volts_chan_volts[i]);
      }
      for (int i = 0; i < 8; ++i)
      {
        x[i]= raw_msg.temp_chan[i];
        // printf("  Temperature Channel %d: %u (%.4f V)\n", i, raw_msg.temp_chan[i], raw_msg.temp_chan_volts[i]);
      }
      ADC_Temp_Conv(&x, &y, 8);
    }

    // Check for sat_temp_msg updates
    if (fds[1].revents & POLLIN)
    {
      struct sat_temp_msg temp_msg;
      orb_copy(ORB_ID(sat_temp_msg), temp_sub_fd, &temp_msg);

      // printf("sat_temp_msg:\n");
      printf("********************************************\n");
      printf("  timestamp: %" PRIu64 " | ", temp_msg.timestamp);
      printf("  batt_temp: %.4f | ", temp_msg.batt_temp);
      printf("  temp_bpb: %.4f | ", temp_msg.temp_bpb);
      printf("  temp_ant: %.4f \n", temp_msg.temp_ant);
      // printf("  temp_z_pos: %.4f\n", temp_msg.temp_z_pos);
      // printf("  temp_5: %.4f\n", temp_msg.temp_5);
      // printf("  temp_4: %.4f\n", temp_msg.temp_4);
      // printf("  temp_3: %.4f\n", temp_msg.temp_3);
      // printf("  temp_2: %.4f\n", temp_msg.temp_2);
      // printf("\nSatellite temperature_converted values: \n");
      // printf("  batt_temp: %.4f |",y[0]);
      // printf("  temp_bpb: %.4f |",y[1]);
      // printf("  temp_ant: %.4f\n",y[2]);
      // printf("  temp_z_pos: %.4f\n",y[3]);
      // printf("  temp_5: %.4f\n",y[4]);
      // printf("  temp_4: %.4f\n",y[5]);
      // printf("  temp_3: %.4f\n",y[6]);
      // printf("  temp_2: %.4f\n",y[7]);

    }

    // Check for sat_volts_msg updates
    if (fds[2].revents & POLLIN)
    {
      struct sat_volts_msg volts_msg;
      orb_copy(ORB_ID(sat_volts_msg), volts_sub_fd, &volts_msg);

      // printf("sat_volts_msg:\n");
      // printf("  timestamp: %" PRIu64 "\n", volts_msg.timestamp);
      // printf("  volt_SolT: %.4f V\n", volts_msg.volt_SolT);
      // printf("  volt_raw: %.4f V\n", volts_msg.volt_raw);
      // printf("  volt_sp5: %.4f V\n", volts_msg.volt_sp5);
      // printf("  volt_sp4: %.4f V\n", volts_msg.volt_sp4);
      // printf("  volt_sp3: %.4f V\n", volts_msg.volt_sp3);
      // printf("  volt_sp1: %.4f V\n", volts_msg.volt_sp1);
      // printf("  volt_sp2: %.4f V\n", volts_msg.volt_sp2);
    }

    // usleep(500000);
    sleep(1);
  }

  return 0;
}

int main(int argc, FAR char *argv[])
{
  printf("[ads7953_receiver] Starting task.\n");

  int ret = task_create("ads7953_receiver", SCHED_PRIORITY_DEFAULT,
                        2048, ads7953_receiver,
                        NULL);

  if (ret < 0)
  {
    int errcode = errno;
    printf("[ads7953_receiver] ERROR: Failed to start ads7953_receiver: %d\n", errcode);
    return EXIT_FAILURE;
  }

  printf("[ads7953_receiver] ads7953_receiver started\n");
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int adc_subscriber(int argc, FAR char *argv[])
{
  // Subscribe to the sat_int_adc_msg topic
  int sub_fd = orb_subscribe(ORB_ID(sat_int_adc_msg));
  if (sub_fd < 0)
  {
    fprintf(stderr, "Failed to subscribe to sat_int_adc_msg: %d\n", errno);
    return EXIT_FAILURE;
  }

  struct pollfd fds[] = {
    { .fd = sub_fd, .events = POLLIN },
  };

  printf("Subscribed to sat_int_adc_msg topic\n");

  for (;;)
  {
    // Wait for data to be available
    int ret = poll(fds, 1, 1000); // 1000 ms timeout
    if (ret < 0)
    {
      fprintf(stderr, "Poll error: %d\n", errno);
      continue;
    }

    if (ret == 0)
    {
      printf("Poll timeout, no data\n");
      continue;
    }

    if (fds[0].revents & POLLIN)
    {
      struct sat_int_adc_msg adc_msg;
      // Copy the data from the topic
      if (orb_copy(ORB_ID(sat_int_adc_msg), sub_fd, &adc_msg) == OK)
      {
        // Process the received data
        printf("Received ADC Data:\n");
        // printf("  C_batt: %.2f mA\n", adc_msg.C_batt);
        // printf("  C_SolT: %.2f mA\n", adc_msg.C_SolT);
        // printf("  C_raw: %.2f mA\n", adc_msg.C_raw);
        printf("  C_unreg: %.2f mA | ", adc_msg.C_unreg);
        // printf("  C_3v3_main: %.2f mA\n", adc_msg.C_3v3_main);
        // printf("  C_3v3_com: %.2f mA\n", adc_msg.C_3v3_com);
        // printf("  C_5v: %.2f mA\n", adc_msg.C_5v);
        // printf("  volt_batt: %.2f V\n", adc_msg.volt_batt);
        // printf("  C_sp1: %.2f mA\n", adc_msg.C_sp1);
        // printf("  C_3v3_2: %.2f mA\n", adc_msg.C_3v3_2);
        // printf("  C_sp4: %.2f mA\n", adc_msg.C_sp4);
        // printf("  C_sp5: %.2f mA\n", adc_msg.C_sp5);
        // printf("  C_sp2: %.2f mA\n", adc_msg.C_sp2);
        // printf("  C_sp3: %.2f mA\n", adc_msg.C_sp3);
        // printf("  C_4v: %.2f mA\n", adc_msg.C_4v);
        printf("  timestamp: %" PRIu64 "\n", adc_msg.timestamp);
      }
    }
  }

  // Clean up and close the subscription
  orb_unsubscribe(sub_fd);
  return EXIT_SUCCESS;
}

/****************************************************************************
 * Name: adc_subscriber_main
 ****************************************************************************/

// int main(int argc, FAR char *argv[])
// {
  
//   return adc_subscriber(argc, argv);
// }