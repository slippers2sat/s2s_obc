// // /****************************************************************************
// //  * custom_apps/mpu6500/mpu6500_main.c
// //  *
// //  * Licensed to the Apache Software Foundation (ASF) under one or more
// //  * contributor license agreements.  See the NOTICE file distributed with
// //  * this work for additional information regarding copyright ownership.  The
// //  * ASF licenses this file to you under the Apache License, Version 2.0 (the
// //  * "License"); you may not use this file except in compliance with the
// //  * License.  You may obtain a copy of the License at
// //  *
// //  *   http://www.apache.org/licenses/LICENSE-2.0
// //  *
// //  * Unless required by applicable law or agreed to in writing, software
// //  * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// //  * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
// //  * License for the specific language governing permissions and limitations
// //  * under the License.
// //  *
// //  ****************************************************************************/

// // #include <nuttx/config.h>

// // #include <sys/types.h>
// // #include <sys/ioctl.h>

// // #include <stdio.h>
// // #include <stdlib.h>
// // #include <unistd.h>
// // #include <string.h>
// // #include <errno.h>
// // #include <debug.h>

// // #include <nuttx/fs/fs.h>

// // #include <time.h>
// // #include <fcntl.h>

// // #include <poll.h>
// // #include <sensor/accel.h>
// // #include <sensor/gyro.h>
// // #include <nuttx/sensors/sensor.h>
// // #include <uORB/uORB.h>
// // #include <sched.h>

// // #define REG_LOW_MASK 0xFF00
// // #define REG_HIGH_MASK 0x00FF
// // #define MPU6500_FS_SEL 32.8f
// // #define MPU6500_AFS_SEL 4096.0f
// // #define DEVNAME_SIZE 32

// // struct mpu6500_imu_msg
// // {
// //   uint64_t timestamp;
// //   float acc_x;
// //   float acc_y;
// //   float acc_z;
// //   float temp;
// //   float gyro_x;
// //   float gyro_y;
// //   float gyro_z;
// //   float temperature;
// // };

// // ORB_DECLARE(mpu6500_imu_msg);

// // static bool g_mpu6500_daemon_started;

// // #ifdef CONFIG_DEBUG_UORB

// // static void print_mpu6500_imu_msg(FAR const struct orb_metadata *meta,
// //                                   FAR const void *buffer)
// // {
// //   FAR const struct mpu6500_imu_msg *message = buffer;
// //   const orb_abstime now = orb_absolute_time();

// //   uorbinfo_raw("%s:\ttimestamp: %" PRIu64 " (%" PRIu64 " us ago) acc_X: %.4f acc_Y: %.4f acc_Z:%.4f Temp: %.4f \n "
// //                "gyro_X: %0.4f gyro_Y: %0.4f gyro_Z: %0.4f temp: %0.2f\n",
// //                meta->o_name, message->timestamp, now - message->timestamp,
// //                message->acc_x, message->acc_y, message->acc_z, message->gyro_x, message->gyro_y, message->gyro_z, message->temperature);
// // }
// // #endif

// // /****************************************************************************
// //  * Public Data
// //  ****************************************************************************/

// // ORB_DEFINE(mpu6500_imu_msg, struct mpu6500_imu_msg, print_mpu6500_imu_msg);

// // int mpu6500_daemon(int argc, FAR char *argv[])
// // {
// //   g_mpu6500_daemon_started = true;
// //   int fd_mpu;
// //   int afd_mpu;
// //   int ret;
// //   int instance = 0;

// //   int16_t imu_raw[7];
// //   struct mpu6500_imu_msg imu0;

// //   afd_mpu = orb_advertise_multi_queue_persist(ORB_ID(mpu6500_imu_msg), &imu0,
// //                                           &instance, sizeof(struct mpu6500_imu_msg));
// //   if (afd_mpu < 0)
// //   {
// //     syslog(LOG_ERR, "Orb advertise failed.\n");
// //   }
// // for (;;)
// // {
// //   fd_mpu = open(CONFIG_IMU0_PATH, O_RDONLY);
// //   if (fd_mpu < 0)
// //   {
// //     syslog(LOG_ERR, "[mpu6500] failed to open mpu6500.");
// //     goto error;
// //   }

// //   ret = read(fd_mpu, imu_raw, sizeof(imu_raw));

// //   if (ret != sizeof(imu_raw))
// //   {
// //     syslog(LOG_ERR, "Failed to read IMU data.\n");
// //     goto rd_err;
// //   }

// //   imu0.acc_x = (((imu_raw[0] & REG_HIGH_MASK) << 8) + ((imu_raw[0] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;
// //   imu0.acc_y = (((imu_raw[1] & REG_HIGH_MASK) << 8) + ((imu_raw[1] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;
// //   imu0.acc_z = (((imu_raw[2] & REG_HIGH_MASK) << 8) + ((imu_raw[2] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;

// //   imu0.gyro_x = (((imu_raw[4] & REG_HIGH_MASK) << 8) + ((imu_raw[4] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL;
// //   imu0.gyro_y = (((imu_raw[5] & REG_HIGH_MASK) << 8) + ((imu_raw[5] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL;
// //   imu0.gyro_z = (((imu_raw[6] & REG_HIGH_MASK) << 8) + ((imu_raw[6] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL;
// //   imu0.timestamp = orb_absolute_time();

// //   close(fd_mpu);

// //   if (OK != orb_publish(ORB_ID(mpu6500_imu_msg), afd_mpu, &imu0))
// //   {
// //     syslog(LOG_ERR, "Orb Publish failed\n");
// //   }
// //   usleep(500000);
// // }

// //   ret = orb_unadvertise(afd_mpu);
// //   if (ret < 0)
// //   {
// //     syslog(LOG_ERR, "Orb Unadvertise failed.\n");
// //   }
// // rd_err:
// //   close(fd_mpu);
// // error:
// //   return 0;
// // }

// // int main(int argc, FAR char *argv[])
// // {
// //   int ret;

// //   printf("[mpu6500] Starting task.\n");
// //   if (g_mpu6500_daemon_started)
// //   {
// //     printf("[mpu6500] Task already started.\n");
// //   return EXIT_SUCCESS;
// //   }

// //   ret = task_create("mpu6500_daemon",SCHED_PRIORITY_DEFAULT,
// //                     CONFIG_CUSTOM_APPS_MPU6500_STACKSIZE, mpu6500_daemon,
// //                     NULL);

// //   if (ret < 0)
// //   {
// //     int errcode = errno;
// //     printf("[mpu6500] ERROR: Failed to start mpu6500_dameon: %d\n",
// //            errcode);
// //     return EXIT_FAILURE;
// //   }

// //   printf("[mpu6500] mpu6500_daemon started\n");
// //   return EXIT_SUCCESS;

// // }

// /****************************************************************************
//  * Included Files
//  ****************************************************************************/

// #include <nuttx/config.h>
// #include <stdio.h>
// #include <syslog.h>

// #include <time.h>
// #include <fcntl.h>

// #include <poll.h>
// #include <sensor/accel.h>
// #include <sensor/mag.h>
// #include <nuttx/sensors/sensor.h>

// // #include <uORB/topics/orb_mag_scaled.h>

// // Uncomment below for logging
// // #define LOGGING

// /****************************************************************************
//  * Public Functions
//  ****************************************************************************/

// #ifdef CONFIG_DEBUG_UORB

// static void print_orb_mag_scaled_msg(FAR const struct orb_metadata *meta,
//                                      FAR const void *buffer)
// {
//   FAR const struct orb_mag_scaled_s *message = buffer;
//   const orb_abstime now = orb_absolute_time();

//   uorbinfo_raw("%s:\ttimestamp: %" PRIu64 " (%" PRIu64 " us ago) X: %.4f Y: %.4f Z:%.4f Temp: %.4f",
//                meta->o_name, message->timestamp, now - message->timestamp,
//                message->mag_x, message->mag_y, message->mag_z, message->temperature);
// }
// #endif
// // ORB_DEFINE(orb_mag_scaled, struct orb_mag_scaled_s, print_orb_mag_scaled_msg);

// /**
//  * @brief Subscribe to the orb_mag_scaled topic and retrieve the latest data.
//  *
//  * @param mag_scaled Pointer to the structure where the data will be stored.
//  * @return int 0 on success, -1 on failure.
//  */
// void continuous_subscribe_and_retrieve_data(void)
// {
//   int fd;
//   int ret;

//   struct orb_mag_scaled_s mag_scaled;

//   /* Subscribe to the orb_mag_scaled topic */
//   fd = orb_subscribe(ORB_ID(orb_mag_scaled));
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Failed to subscribe to orb_mag_scaled topic.\n");
//     return;
//   }

//   /* Poll for new data */
//   struct pollfd fds;
//   fds.fd = fd;
//   fds.events = POLLIN;

//   while (1) // Infinite loop
//   {
//     if (poll(&fds, 1, -1) > 0) // Infinite timeout
//     {
//       if (fds.revents & POLLIN)
//       {
//         /* Copy the data from the orb */
//         ret = orb_copy(ORB_ID(orb_mag_scaled), fd, &mag_scaled);
//         if (ret < 0)
//         {
//           syslog(LOG_ERR, "ORB copy error, %d \n", ret);
//           continue;
//         }

//         // Print the received data
//         printf("Timestamp: %" PRIu64 "\n", mag_scaled.timestamp);
//         printf("Mag_X: %.4f Mag_Y: %.4f Mag_Z: %.4f\n", mag_scaled.mag_x, mag_scaled.mag_y, mag_scaled.mag_z);
//         printf("Acc_X: %.4f Acc_Y: %.4f Acc_Z: %.4f\n", mag_scaled.acc_x, mag_scaled.acc_y, mag_scaled.acc_z);
//         printf("Gyro_X: %.4f Gyro_Y: %.4f Gyro_Z: %.4f\n", mag_scaled.gyro_x, mag_scaled.gyro_y, mag_scaled.gyro_z);
//         printf("Temperature: %.2f\n", mag_scaled.temperature);
//       }
//     }
//     else
//     {
//       syslog(LOG_ERR, "Poll error.\n");
//     }
//   }

//   ret = orb_unsubscribe(fd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Orb unsubscribe Failed.\n");
//   }
//   return 0;
// }

// int main(int argc, FAR char *argv[])
// {
//   printf("[mag] Starting continuous data retrieval...\n");
//   continuous_subscribe_and_retrieve_data();

//   return 0;
// }

/****************************************************************************
 * custom_apps/mpu6500/mpu6500_main.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include <time.h>
#include <fcntl.h>

#include <poll.h>
#include <sensor/accel.h>
#include <sensor/gyro.h>
#include <nuttx/sensors/sensor.h>
#include <uORB/uORB.h>
#include <sched.h>

#define REG_LOW_MASK 0xFF00
#define REG_HIGH_MASK 0x00FF
#define MPU6500_FS_SEL 32.8f
#define MPU6500_AFS_SEL 4096.0f
#define DEVNAME_SIZE 32

struct mpu6500_imu_msg
{
  uint64_t timestamp;
  float acc_x;
  float acc_y;
  float acc_z;
  float temp;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float temperature;
};

ORB_DECLARE(mpu6500_imu_msg);

static bool g_mpu6500_daemon_started;

#ifdef CONFIG_DEBUG_UORB

static void print_mpu6500_imu_msg(FAR const struct orb_metadata *meta,
                                  FAR const void *buffer)
{
  FAR const struct mpu6500_imu_msg *message = buffer;
  const orb_abstime now = orb_absolute_time();

  uorbinfo_raw("%s:\ttimestamp: %" PRIu64 " (%" PRIu64 " us ago) acc_X: %.4f acc_Y: %.4f acc_Z:%.4f Temp: %.4f \n "
               "gyro_X: %0.4f gyro_Y: %0.4f gyro_Z: %0.4f temp: %0.2f\n",
               meta->o_name, message->timestamp, now - message->timestamp,
               message->acc_x, message->acc_y, message->acc_z, message->gyro_x, message->gyro_y, message->gyro_z, message->temperature);
}
#endif


/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(mpu6500_imu_msg, struct mpu6500_imu_msg, print_mpu6500_imu_msg);

int mpu6500_daemon(int argc, FAR char *argv[])
{
  g_mpu6500_daemon_started = true;
  int fd;
  int afd;
  int ret;
  int instance = 0;

  int16_t imu_raw[7];
  struct mpu6500_imu_msg imu0;

  afd = orb_advertise_multi_queue_persist(ORB_ID(mpu6500_imu_msg), &imu0,
                                          &instance, sizeof(struct mpu6500_imu_msg));
  if (afd < 0)
  {
    syslog(LOG_ERR, "Orb advertise failed.\n");
  }
for (;;)
{
  fd = open(CONFIG_IMU0_PATH, O_RDONLY);
  if (fd < 0)
  {
    syslog(LOG_ERR, "[mpu6500] failed to open mpu6500.");
    goto error;
  }

  ret = read(fd, imu_raw, sizeof(imu_raw));

  if (ret != sizeof(imu_raw))
  {
    syslog(LOG_ERR, "Failed to read IMU data.\n");
    goto rd_err;
  }

  imu0.acc_x = (((imu_raw[0] & REG_HIGH_MASK) << 8) + ((imu_raw[0] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;
  imu0.acc_y = (((imu_raw[1] & REG_HIGH_MASK) << 8) + ((imu_raw[1] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;
  imu0.acc_z = (((imu_raw[2] & REG_HIGH_MASK) << 8) + ((imu_raw[2] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;

  imu0.gyro_x = (((imu_raw[4] & REG_HIGH_MASK) << 8) + ((imu_raw[4] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL ;
  imu0.gyro_y = (((imu_raw[5] & REG_HIGH_MASK) << 8) + ((imu_raw[5] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL ;
  imu0.gyro_z = (((imu_raw[6] & REG_HIGH_MASK) << 8) + ((imu_raw[6] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL ;
  imu0.timestamp = orb_absolute_time();
  printf("Acc X: %0.02f %0.02f Y:%0.02f Z:%0.02f\n",  imu0.gyro_x, 2048 - imu0.gyro_x, imu0.gyro_y, imu0.gyro_z);
   
  close(fd);

  if (OK != orb_publish(ORB_ID(mpu6500_imu_msg), afd, &imu0))
  {
    syslog(LOG_ERR, "Orb Publish failed\n");
  }
  usleep(500000);
}


  ret = orb_unadvertise(afd);
  if (ret < 0)
  {
    syslog(LOG_ERR, "Orb Unadvertise failed.\n");
  }
rd_err:
  close(fd);
error:
  return 0;
}


int main(int argc, FAR char *argv[])
{
  int ret;

  printf("[mpu6500] Starting task.\n");
  if (g_mpu6500_daemon_started)
  {
    printf("[mpu6500] Task already started.\n");
  return EXIT_SUCCESS;
  }

  ret = task_create("mpu6500_daemon",SCHED_PRIORITY_DEFAULT,
                    CONFIG_CUSTOM_APPS_MPU6500_STACKSIZE, mpu6500_daemon,
                    NULL);

  if (ret < 0)
  {
    int errcode = errno;
    printf("[mpu6500] ERROR: Failed to start mpu6500_dameon: %d\n",
           errcode);
    return EXIT_FAILURE;
  }

  printf("[mpu6500] mpu6500_daemon started\n");
  return EXIT_SUCCESS;
  
  
}