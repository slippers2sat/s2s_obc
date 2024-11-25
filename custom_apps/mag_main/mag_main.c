/****************************************************************************
 * custom_apps/mag_main/custom_hello.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *time
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
#include "mag_main.h"
#include <syslog.h>

#include <time.h>
#include <fcntl.h>

#include <poll.h>
#include <sensor/accel.h>
#include <sensor/gyro.h>
#include <nuttx/sensors/sensor.h>



//TODO uncomment below for logging
// #define LOGGING
//TODO
#define REG_LOW_MASK 0xFF00
#define REG_HIGH_MASK 0x00FF
#define MPU6500_FS_SEL 32.8f
#define MPU6500_AFS_SEL 4096.0f
#define DEVNAME_SIZE 32
// #include
static bool g_mag_daemon_started;
#ifndef CONFIG_MPU_PATH
#define CONFIG_MPU_PATH "/dev/mpu6500"
#endif
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
  int ANT_DEPLOYED;

};

struct sensor_accel imu_acc_data1;
struct sensor_gyro imu_gyro_data1;
struct mpu6500_imu_msg raw_imu1;
void collect_imu_mag();

void read_MPU6500(int fd, struct sensor_accel *acc_data, struct sensor_gyro *gyro_data, struct mpu6500_imu_msg *raw_imu1);


ORB_DECLARE(mpu6500_imu_msg);

static bool g_mpu6500_daemon_started;
/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_UORB

static void print_orb_mag_scaled_msg(FAR const struct orb_metadata *meta,
                                     FAR const void *buffer)
{
  FAR const struct orb_mag_scaled_s *message = buffer;
  const orb_abstime now = orb_absolute_time();

  uorbinfo_raw("%s:\ttimestamp: %" PRIu64 " (%" PRIu64 " us ago) X: %.4f Y: %.4f Z:%.4f Temp: %.4f",
               meta->o_name, message->timestamp, now - message->timestamp,
               message->mag_x, message->mag_y, message->mag_z, message->temperature);
}
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

ORB_DEFINE(orb_mag_scaled, struct orb_mag_scaled_s, print_orb_mag_scaled_msg);

/****************************************************************************
 * custom_hello_main
 ****************************************************************************/

int mag_daemon(int argc, FAR char *argv[])
{

  g_mag_daemon_started = true;
  struct orb_mag_scaled_s mag_scaled;
  int instance = 0;
  bool updated;
  int afd;

  struct sensor_mag mag0;
  struct pollfd fds;
  int fd;
  int ret;
  int i;

  // for mpu
  g_mpu6500_daemon_started = true;
  int fd_mpu;
  int afd_mpu;
  uint16_t imu_raw[7];
  // struct mpu6500_imu_msg imu0;

  /* advertie scaled mag topic */

  afd = orb_advertise_multi_queue_persist(ORB_ID(orb_mag_scaled), &mag_scaled,
                                          &instance, sizeof(struct orb_mag_scaled_s));
  if (afd < 0)
  {
    syslog(LOG_ERR, "Orb advertise failed.\n");
  }

  fd = orb_subscribe_multi(ORB_ID(sensor_mag), 0);

  fds.fd = fd;
  fds.events = POLLIN;
  for (;;)
  {
//     // IMU

//     fd_mpu = open(CONFIG_IMU0_PATH, O_RDONLY);
//     if (fd_mpu < 0)
//     {
//       syslog(LOG_ERR, "[mpu6500] failed to open mpu6500.");
//       goto error;
//     }

//     ret = read(fd_mpu, imu_raw, sizeof(imu_raw));

//     if (ret != sizeof(imu_raw))
//     {
//       syslog(LOG_ERR, "Failed to read IMU data.\n");
//       goto rd_err;
//     }

//     mag_scaled.acc_x = (((imu_raw[0] & REG_HIGH_MASK) << 8) + ((imu_raw[0] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL ;
//     mag_scaled.acc_y = (((imu_raw[1] & REG_HIGH_MASK) << 8) + ((imu_raw[1] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;
//     mag_scaled.acc_z = (((imu_raw[2] & REG_HIGH_MASK) << 8) + ((imu_raw[2] & REG_LOW_MASK) >> 8)) / MPU6500_AFS_SEL;

//     mag_scaled.gyro_x = (((imu_raw[4] & REG_HIGH_MASK) << 8) + ((imu_raw[4] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL - 1996;
//     mag_scaled.gyro_y = (((imu_raw[5] & REG_HIGH_MASK) << 8) + ((imu_raw[5] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL;
//     mag_scaled.gyro_z = (((imu_raw[6] & REG_HIGH_MASK) << 8) + ((imu_raw[6] & REG_LOW_MASK) >> 8)) / MPU6500_FS_SEL;
//     mag_scaled.timestamp = orb_absolute_time();

//     close(fd_mpu);
//     if (poll(&fds, 1, 1000) > 0)
//     {
//       if (fds.revents & POLLIN)
//       {
//         ret = orb_copy_multi(fd, &mag0, sizeof(struct sensor_mag));
//         if (ret < 0)
//         {
//           syslog(LOG_ERR, "ORB copy error, %d \n", ret);
//           return ret;
//         }

//         // syslog(LOG_INFO, "Copied data from orb_object.\n");
//       //TODO uncomment for logging
//       #ifdef LOGGING
//         printf("Timestamp: %lli \t", mag0.timestamp);
//         syslog(LOG_DEBUG, "Temperature: %0.02f \t||", mag0.temperature);
//       #endif
//         printf("X : %0.02f||%0.02f \t", mag0.mag_x, 0.14 * (mag0.mag_x -32768) );
//         printf("Y : %0.02f||%0.02f \t", mag0.mag_y, 0.14 * (mag0.mag_y -32768 ));
//         printf("Z : %0.02f||%0.02f \t\n", mag0.mag_z, 0.14 * (mag0.mag_z -32768) );
//       }
//       mag_scaled.mag_x = mag0.mag_x * 100;
//       mag_scaled.mag_y = mag0.mag_y * 100;
//       mag_scaled.mag_z = mag0.mag_z * 100;
//       mag_scaled.temperature = mag0.temperature - 50;
//       mag_scaled.timestamp = orb_absolute_time();
      
//       printf("Acc X : %0.02f \t", mag_scaled.acc_x);
//         printf("Y : %0.02f \t", mag_scaled.acc_y);
//         printf("Z : %0.02f \t\n", mag_scaled.acc_z);
//         printf("Gyro X : %0.02f \t", mag_scaled.gyro_x);
//         printf("Y : %0.02f \t", mag_scaled.gyro_y);
//         printf("Z : %0.02f \t\n", mag_scaled.gyro_z);
//       // printf("X : %0.02f \t", mag_scaled.mag_x);
//       //   printf("Y : %0.02f \t", mag_scaled.mag_y);
//       //   printf("Z : %0.02f \t\n", mag_scaled.mag_z);

//       if (OK != orb_publish(ORB_ID(orb_mag_scaled), afd, &mag_scaled))
//       {
//         syslog(LOG_ERR, "Orb Publish failed\n");
//       }
//     }

//     if (OK != orb_publish(ORB_ID(orb_mag_scaled), afd, &mag_scaled))
//     {
//       syslog(LOG_ERR, "Orb Publish mag failed\n");
//     }
//     usleep(10000);
//   }

//   ret = orb_unadvertise(afd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Orb Unadvertise mag failed.\n");
//   }

//   ret = orb_unsubscribe(fd);

//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Orb unsubscribe Failed.\n");
  
  collect_imu_mag();
  sleep(3);
  }

// rd_err:
//   close(fd_mpu);
// error:
  return 0;
}

/****************************************************************************
 * mag_main thread
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{
  int ret;

  printf("[mag] Starting task.\n");
  if (g_mag_daemon_started)
  {
    printf("[mag] Task already started.\n");
    return EXIT_SUCCESS;
  }

  ret = task_create("mag_daemon", SCHED_PRIORITY_DEFAULT,
                    CONFIG_CUSTOM_APPS_MAG_MAIN_STACKSIZE, mag_daemon,
                    NULL);

  if (ret < 0)
  {
    int errcode = errno;
    printf("[mag] ERROR: Failed to start mag_dameon: %d\n",
           errcode);
    return EXIT_FAILURE;
  }

  printf("[mag] mag_daemon started\n");
}
void collect_imu_mag()
{
  float acq_period = CONFIG_EXAMPLES_SENSOR_FUSION_SAMPLE_RATE / 1000.0f;
  printf("Sensor Fusion example\n");
  printf("Sample Rate: %.2f Hz\n", 1.0 / acq_period);
  printf("got inside sensor_work");
  int fd, fd_mag;

  int16_t mag_data[4];

  fd = open("/dev/mpu6500", O_RDONLY);
  if (fd < 0)
  {
    printf("Failed to open mpu6500\n");
    return; // This might create an issue
  }

  
  read_MPU6500(fd, &imu_acc_data1, &imu_gyro_data1, &raw_imu1);
  // read_lis3mdl(fd_mag, &raw_imu1, mag_data);

  // printf("Timestamp: %f  Temperature: %f\n"
  //        "Accelerometer X: %f | Y: %f | Z: %f\n"
  //        "Gyroscope X: %f | Y: %f | Z: %f\n"
  //        "Magnetometer X: %f | Y: %f | Z: %f\n",
  //        imu_acc_data1.timestamp, imu_acc_data1.temperature,
  //        imu_acc_data1.x, imu_acc_data1.y, imu_acc_data1.z,
  //        imu_gyro_data1.x, imu_gyro_data1.y, imu_gyro_data1.z
  //       //  ,
  //       //  raw_imu1.mag_x, raw_imu1.mag_y, raw_imu1.mag_z
  //        );
  printf("************************************************\n");

  close(fd);
  close(fd_mag);

 }
void read_MPU6500(int fd, struct sensor_accel *acc_data, struct sensor_gyro *gyro_data, struct mpu6500_imu_msg *raw_imu1)
{
  int16_t raw_data[7];
  memset(raw_imu1, 0, sizeof(struct mpu6500_imu_msg));
  int ret = read(fd, raw_data, sizeof(raw_data));
  if (ret <= 0) //!= sizeof(raw_data))
  {
    printf("Failed to read accelerometer data\n");
  }
  else
  {
    raw_imu1->acc_x = ((raw_data[0] & REG_HIGH_MASK) << 8) + ((raw_data[0] & REG_LOW_MASK) >> 8);
    raw_imu1->acc_y = ((raw_data[1] & REG_HIGH_MASK) << 8) + ((raw_data[1] & REG_LOW_MASK) >> 8);
    raw_imu1->acc_z = ((raw_data[2] & REG_HIGH_MASK) << 8) + ((raw_data[2] & REG_LOW_MASK) >> 8);
    raw_imu1->gyro_x = ((raw_data[4] & REG_HIGH_MASK) << 8) + ((raw_data[4] & REG_LOW_MASK) >> 8);
    raw_imu1->gyro_y = ((raw_data[5] & REG_HIGH_MASK) << 8) + ((raw_data[5] & REG_LOW_MASK) >> 8);
    raw_imu1->gyro_z = ((raw_data[6] & REG_HIGH_MASK) << 8) + ((raw_data[6] & REG_LOW_MASK) >> 8);
  }

  acc_data->x = raw_imu1->acc_x *9.8/ MPU6500_AFS_SEL;
  acc_data->y = raw_imu1->acc_y *9.8/ MPU6500_AFS_SEL;
  acc_data->z = raw_imu1->acc_z *9.8/ MPU6500_AFS_SEL;

  gyro_data->x = raw_imu1->gyro_x / MPU6500_FS_SEL;
  gyro_data->y = raw_imu1->gyro_y / MPU6500_FS_SEL;
  gyro_data->z = raw_imu1->gyro_z / MPU6500_FS_SEL;
  printf("Timestamp: %0.02f  Temperature: %0.02f\n"
         "Accelerometer X: %0.02f | Y: %0.02f | Z: %0.02f\n"
         "Gyroscope X: %0.02f | Y: %0.02f | Z: %0.02f\n",
         
        //  imu_acc_data1.timestamp, imu_acc_data1.temperature,
         acc_data->x, acc_data->y, acc_data->z,
         gyro_data->x, gyro_data->y, gyro_data->z);
}


// /****************************************************************************
//  * custom_apps/mag_main/custom_hello.c
//  *
//  * Licensed to the Apache Software Foundation (ASF) under one or more
//  * contributor license agreements.  See the NOTICE file distributed with
//  * this work for additional information regarding copyright ownership.  The
//  * ASF licenses this file to you under the Apache License, Version 2.0 (the
//  * "License"); you may not use this file except in compliance with the
//  * License.  You may obtain a copy of the License at
//  *
//  *   http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
//  * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
//  * License for the specific language governing permissions and limitations
//  * under the License.
//  *
//  ****************************************************************************/

// /****************************************************************************
//  * Included Files
//  ****************************************************************************/

// #include <nuttx/config.h>
// #include <stdio.h>
// #include "mag_main.h"


// static bool g_mag_daemon_started;
// /****************************************************************************
//  * Public Functions
//  ****************************************************************************/

// #ifdef CONFIG_DEBUG_UORB

// static void print_orb_mag_scaled_msg(FAR const struct orb_metadata *meta,
//                                      FAR const void *buffer)
// {
//   FAR const struct orb_mag_scaled_s *message = buffer;
//   const orb_abstime now = orb_absolute_time();

//   uorbinfo_raw("%s:\ttimestamp: %"PRIu64" (%"PRIu64" us ago) X: %.4f Y: %.4f Z:%.4f Temp: %.4f",
//                 meta->o_name, message->timestamp, now - message->timestamp, 
//                 message->mag_x, message->mag_y, message->mag_z, message->temperature);
// }
// #endif

// /****************************************************************************
//  * Public Data
//  ****************************************************************************/

// ORB_DEFINE(orb_mag_scaled, struct orb_mag_scaled_s, print_orb_mag_scaled_msg);

// /****************************************************************************
//  * custom_hello_main
//  ****************************************************************************/

// int  mag_daemon(int argc, FAR char *argv[])
// {
  
//   g_mag_daemon_started = true;
//   struct orb_mag_scaled_s mag_scaled;
//   int instance =0;
//   bool updated;
//   int afd;

//   struct sensor_mag mag0;
//   struct pollfd fds;
//   int fd;
//   int ret;
//   int i;

//   /* advertie scaled mag topic */

//   afd = orb_advertise_multi_queue_persist(ORB_ID(orb_mag_scaled),&mag_scaled,
//                                            &instance, sizeof(struct orb_mag_scaled_s));
//   if (afd < 0)
//   {
//     syslog(LOG_ERR, "Orb advertise failed.\n");
//   }
  

//   fd = orb_subscribe_multi(ORB_ID(sensor_mag), 0);

//   fds.fd = fd;
//   fds.events = POLLIN;
//   for(;;)
//   {
//     if (poll(&fds, 1, 3000) > 0)
//     {
//       if(fds.revents & POLLIN)
//       {
//         ret = orb_copy_multi(fd, &mag0, sizeof(struct sensor_mag));
//         if (ret < 0)
//         {
//           syslog(LOG_ERR, "ORB copy error, %d \n", ret);
//           return ret;
//         }

//         // syslog(LOG_INFO, "Copied data from orb_object.\n");

//         printf("Timestamp: %lli \t", mag0.timestamp);
//         printf("Temperature: %0.02f \t", mag0.temperature);
//         printf("X : %0.02f \t", mag0.mag_x);
//         printf("Y : %0.02f \t", mag0.mag_y);
//         printf("Z : %0.02f \t\n", mag0.mag_z);
//       }
//       mag_scaled.mag_x = mag0.mag_x * 100;
//       mag_scaled.mag_y = mag0.mag_y * 100;
//       mag_scaled.mag_z = mag0.mag_z * 100;
//       mag_scaled.temperature = mag0.temperature - 50;
//       mag_scaled.timestamp = orb_absolute_time();

//       if(OK != orb_publish(ORB_ID(orb_mag_scaled), afd, &mag_scaled))
//       {
//         syslog(LOG_ERR,"Orb Publish failed\n");
//       }
//     }
//   }
  
//   ret = orb_unadvertise(afd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Orb Unadvertise failed.\n");
//   }

//   ret = orb_unsubscribe(fd);

//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Orb unsubscribe Failed.\n");
//   }
  
//   return 0;
// }



// /****************************************************************************
//  * mag_main thread
//  ****************************************************************************/
// int main(int argc, FAR char *argv[])
// {
//   int ret;

//   printf("[mag] Starting task.\n");
//   if (g_mag_daemon_started)
//   {
//     printf("[mag] Task already started.\n");
//   return EXIT_SUCCESS;
//   }

//   ret = task_create("mag_daemon",SCHED_PRIORITY_DEFAULT,
//                     CONFIG_CUSTOM_APPS_MAG_MAIN_STACKSIZE, mag_daemon,
//                     NULL);

//   if (ret < 0)
//   {
//     int errcode = errno;
//     printf("[mag] ERROR: Failed to start mag_dameon: %d\n",
//            errcode);
//     return EXIT_FAILURE;
//   }

//   printf("[mag] mag_daemon started\n");
  
  
// }