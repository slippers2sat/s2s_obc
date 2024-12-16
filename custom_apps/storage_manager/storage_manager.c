
/****************************************************************************
 * custom_apps/storage_manager/storage_manager_main.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership. The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#include <nuttx/config.h>
// #include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <uORB/uORB.h>
#include <sensor/mag.h>
#include <nuttx/sensors/sensor.h>
#include <poll.h>

#include "common_functions_edited.h"
#include "file_operations.h"
#include <sensor/rgb.h>
#include <sensor/command.h>
#include <sensor/flash_operation.h>

void Setup();
#define MFM_MTD_MAINSTORAGE "/mnt/fs/mfm/mtd_mainstorage"
#define MFM_MTD_MISSION "/mnt/fs/mfm/mtd_mission"
#define SFM_MTD_MAINSTORAGE "/mnt/fs/sfm/mtd_mainstorage"
#define SFM_MTD_MISSION "/mnt/fs/sfm/mtd_mission"

#define RESERVATION_CMD "/reservation_command.txt"

#define NB_LOWERHALFS 1
// #include <sensor/adc.h>
static struct work_s work_storage;
static int8_t count = 0;
static bool g_storage_manager_started;

void store_sat_health_data(satellite_health_s *sat_health_data, char *pathname);
void sort_reservation_command(uint16_t file_size, bool reorder);
int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode);
void read_and_print_mag_data(void)
{
  count++;
  int sub_fd;
  struct reservation_command res;
  int fd_reservation;
  fd_reservation = orb_subscribe(ORB_ID(reservation_command));
  struct orb_mag_scaled_s mag_data;
  satellite_health_s satellite_health;
  bool updated;

  struct pollfd fds2;
  struct sensor_rgb satHealth;
  int fd2, ret;
  fd2 = orb_subscribe_multi(ORB_ID(sensor_rgb), 0);
  fds2.fd = fd2;
  fds2.events = POLLIN;

  sub_fd = orb_subscribe(ORB_ID(orb_mag_scaled));
  // ads = orb_subscibe(ORB_ID(e_ads7953_0));
  if (sub_fd < 0)
  {
    // syslog(LOG_ERR, "Failed to subscribe to orb_mag_scaled topic\n");
    return;
  }
  uint8_t count = 0;
  sleep(30);

  // sort_reservation_command(1, false);

  while (1)
  {
    if (count % 10 == 0)
    {
      sort_reservation_command(1, false);
      flash_operations();
    }
    count++;
    orb_check(fd_reservation, &updated);
    if (updated)
    {
      // updated = false;
      struct file file_ptr;
      orb_copy(ORB_ID(reservation_command), fd_reservation, &res);
      printf("Value of reservation command uorb has been updated");
      printf("The reservation command is %02x %02x %02x\n", res.cmd[0], res.cmd[1], res.cmd[2]);
      if (res.latest_time == 0x00 && res.mcu_id != 0)
      {
        int fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_CREAT | O_WRONLY | O_APPEND);
        ssize_t file_size = file_seek(&file_ptr, 0, SEEK_END);
        ssize_t bytes_written = file_write(&file_ptr, &res, sizeof(res));
        if (bytes_written > 0)
        {
          printf("File size is %d.\nReservation table with data size %d has been updated\n", file_size, bytes_written);
        }
        if (ret = file_syncfs(&file_ptr) < 0)
        {
          // syslog(LOG_DEBUG, "some issue while synfs closing: %d\n",ret);
          file_syncfs(&file_ptr);
        }
        if (file_close(&file_ptr) < 0)
        {
          // syslog(LOG_DEBUG, "some issue while synfs closing\n");

          file_close(&file_ptr);
        }
        // sort_reservation_command(1);
      }
      sleep(1);
    }
    // updated = false;

    if (count % 90 == 0)
    {
      count = 0;
      orb_check(sub_fd, &updated);
      if (updated)
      {
        orb_copy(ORB_ID(orb_mag_scaled), sub_fd, &mag_data);

        // edited
        if (poll(&fds2, 1, 3000) > 0)
        {
          if (fds2.revents & POLLIN)
          {
            ret = orb_copy_multi(fd2, &satHealth, sizeof(struct sensor_rgb));
            // print_satellite_health_data(&satHealth);
            if (ret < 0)
            {
              syslog(LOG_ERR, "ORB copy error, %d \n", ret);
              return ret;
            }
            else
            {
              printf("Satellite Health ORB is getting data %d\n", satellite_health.rsv_cmd);
              // if (count % 12 == 0)
              {
              }
              // printf("%d %d %d\n",satHealth.accl_x, satHealth.accl_y, satHealth.accl_z);
            }

            // printf("Temperature: %0.02f \t", mag0.temperature);
          }
        }
        // edited

#ifdef LOGGING
        // syslog(LOG_INFO, "\n\n\r****-------[Storage maanager ]Mag Data:\n");

        // syslog(LOG_INFO, "  ACCELEROMETER X: %.4f Y: %.4f Z: %.4f\n", mag_data.acc_x, mag_data.acc_y, mag_data.acc_z);
        // syslog(LOG_INFO, "  GYROSCOPE X: %.4f Y: %.4f Z: %.4f\n", mag_data.gyro_x, mag_data.gyro_y, mag_data.gyro_z);

        // syslog(LOG_INFO, "  MAGNETOMETER X: %.4f Y: %.4f Z: %.4f\n", mag_data.mag_x, mag_data.mag_y, mag_data.mag_z);
        // syslog(LOG_INFO, "  Temp:  %.4f\n", mag_data.temperature);
#endif
        satellite_health.accl_x = mag_data.acc_x;
        satellite_health.accl_y = mag_data.acc_y;
        satellite_health.accl_z = mag_data.acc_z;

        satellite_health.gyro_x = mag_data.gyro_x;
        satellite_health.gyro_y = mag_data.gyro_y;
        satellite_health.gyro_z = mag_data.gyro_z;

        satellite_health.mag_x = mag_data.mag_x;
        satellite_health.mag_y = mag_data.mag_y;
        satellite_health.mag_z = mag_data.mag_z;
        if (satellite_health.msn_flag != 0x11 & satellite_health.ant_dep_stat == DEPLOYED)
        {
          store_sat_health_data(&satHealth, MFM_MAIN_STRPATH);
          print_satellite_health_data(&satHealth);
        }
        // store_sat_health_data(&satellite_health, SFM_MAIN_STRPATH);
      }
      maintain_data_consistency();
    }

    sleep(1); // Sleep for 500 ms
  }

  orb_unsubscribe(sub_fd);
}

int storage_manager_daemon(int argc, FAR char *argv[])
{
  g_storage_manager_started = true;
  read_and_print_mag_data();
  return 0;
}

int main(int argc, FAR char *argv[])
{
  int ret;

  printf("[storage_manager] Starting task.\n");
  Setup();
  if (g_storage_manager_started)
  {
    printf("[storage_manager] Task already started.\n");
    return EXIT_SUCCESS;
  }

  // ret = task_create("storage_manager_daemon", SCHED_PRIORITY_DEFAULT, CONFIG_CUSTOM_APPS_STORAGE_MANAGER_STACKSIZE, storage_manager_daemon, NULL);
  ret = task_create("storage_manager_daemon", SCHED_PRIORITY_DEFAULT, 8078, storage_manager_daemon, NULL);

  if (ret < 0)
  {
    int errcode = errno;
    printf("[storage_manager] ERROR: Failed to start storage_manager_daemon: %d\n", errcode);
    return EXIT_FAILURE;
  }

  printf("[storage_manager] storage_manager_daemon started\n");
  return EXIT_SUCCESS;
}

void store_sat_health_data(satellite_health_s *sat_health_data, char *pathname)
{
  struct file file_p;
  int ret;
  // if(strcmp(pathname, SFM_MAIN_STRPATH) == 1){
  //   gpio_write(GPIO_MUX_EN, false);

  //   gpio_write(GPIO_SFM_MODE, false);
  // }
  // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
  if (open_file_flash(&file_p, pathname, file_name_sat_health, O_CREAT | O_RDWR | O_APPEND) >= 0)
  {

    ssize_t bytes_written = file_write(&file_p, sat_health_data, sizeof(satellite_health_s));
    if (bytes_written > 0)
    {
      syslog(LOG_INFO, "Satellite Health data write Successful.\nData Len: %d.\n", bytes_written);
      file_close(&file_p);
    }
    else
    {
      // syslog(LOG_INFO, "Write Failure.\n");
    }
    if (ret = file_syncfs(&file_p) < 0)
    {
      // syslog(LOG_DEBUG, "some issue while synfs closing: %d\n",ret);
      file_syncfs(&file_p);
    }
    if (file_close(&file_p) < 0)
    {
      // syslog(LOG_DEBUG, "some issue while synfs closing\n");

      file_close(&file_p);
    }
  }
  else
  {
    // syslog(LOG_ERR, "Error opening file to write satellite health data..\n");
  }
  file_close(&file_p);

  // if(strcmp(pathname, SFM_MAIN_STRPATH) == 1){

  //   gpio_write(GPIO_SFM_MODE, true);
  // }
}

int open_file_flash(struct file *file_pointer, char *flash_strpath, char *filename, int open_mode)
{

  const char file_name[] = {'\0'};
  // memcpy(file_name, filename, sizeof(filename));
  char path[65];
  sprintf(path, "%s%s", flash_strpath, filename);
  int fd = file_open(file_pointer, path, open_mode);
  if (fd < 0)
  {
    // syslog(LOG_ERR, "Error opening file: %s\n", path);
    return fd;
  }
  else
  {
    // syslog(LOG_INFO, "Opened file: %s ...\n", path);
  }
  return fd;
}

void flash_operations()
{
  struct command command_ops;
  int updated = 0;
  int flash_fd = orb_subscribe_multi(ORB_ID(command), 10);

  // orb_check(flash_fd, &updated);
  // if (updated)
  while (1)
  {
    orb_check(flash_fd, &updated);
    if (updated)
    {
      updated = false;
      orb_copy_multi(flash_fd, &command_ops, sizeof(struct command));
      // printf("----Received mcu id : %d \n",command_ops.mcu_id);
      printf("\nData received :\nTimestamp : %d \nPath: %s \nAddress:%d \nCommand:%d \nNumber of packets:%d\n",
             command_ops.timestamp,
             command_ops.path,
             command_ops.address,
             command_ops.command,
             // command_ops.packet_number,
             command_ops.num_of_packets);
      // send_data_uorb(command_ops.path, &command_ops);
      send_data_uorb(command_ops.path, command_ops.address, command_ops.num_of_packets);
    }
    sleep(1);
  }
  orb_unsubscribe(ORB_ID(command));
}

void Setup()
{
  int fd = 0;
  struct file flp1, flp2, flp3, flp4;
  fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_CREAT);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Could not create file named sat_health... \n");
  }
  file_close(&flp1);

  /*delete this later
   */
  fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_TRUNC);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Could not create file named sat_health... \n");
  }
  file_close(&flp1);
  close(fd);
  return 0;
  /*
   */
  fd = open_file_flash(&flp2, MFM_MAIN_STRPATH, file_name_flag, O_CREAT);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Could not create flags.txt data file ... \n");
  }
  file_close(&flp2);

  fd = open_file_flash(&flp3, MFM_MSN_STRPATH, file_name_cam_msn, O_CREAT);
  if (fd < 0)
  {
    syslog(LOG_ERR, "could not create cam.txt msn file ...\n");
  }
  file_close(&flp3);

  fd = open_file_flash(&flp4, MFM_MSN_STRPATH, file_name_epdm_msn, O_CREAT);
  if (fd < 0)
  {
    syslog(LOG_ERR, "Could not create epdm.txt msn file\n");
  }
  file_close(&flp4);

  syslog(LOG_INFO, "Checking initial flag data...\n");
  check_flag_data();
  print_critical_flag_data(&critic_flags);
}

void get_top_rsv(struct reservation_command *res)
{
  struct file fptr;
  int ret;
  int fd = open_file_flash(&fptr, MFM_MAIN_STRPATH, "/reservation_command.txt", O_RDONLY);
  uint32_t file_size = file_seek(&fptr, 0, SEEK_END);
  if (file_size == 0 || file_size < 6)
  {
    res->mcu_id = 0;
    res->cmd[0] = 0;
    res->cmd[1] = 0;
    res->cmd[2] = 0;
    res->time[0] = 0;
    res->time[1] = 0;
  }
  else
  {
    if (fd >= 0)
    {
      file_seek(&fptr, 0, SEEK_SET);
      if (file_read(&fptr, res, sizeof(struct reservation_command)) >= 0)
      {
        printf("read the data as :\n %02x %02x %02x %02x %02x %02x\n", res->mcu_id, res->cmd[0], res->cmd[1], res->cmd[2], res->cmd[3], res->cmd[4]);
      }
    }
    uint32_t t1 = (uint32_t)res->cmd[3] << 8 | res->cmd[4];
    res->latest_time = t1 * 60;
  }
  file_close(&fptr);
}

// void sort_reservation_command(uint16_t file_size, bool reorder) {
//     struct file file_ptr;
//     uint16_t temp = 0;

//     int fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_RDONLY);
//     struct reservation_command res_temp[16];
//     uint16_t t = 0x00;

//     file_size = file_seek(&file_ptr, 0, SEEK_END);

//     // Read reservation commands
//     for (int i = 0; i < (file_size / 10); i++) {
//         file_seek(&file_ptr, temp, SEEK_SET);
//         temp += sizeof(struct reservation_command);
//         if (file_read(&file_ptr, &res_temp[i], sizeof(struct reservation_command)) > 0) {
//             // printf("Number of res command read is %d and temp is %d\n", i + 1, temp);
//             // printf("_________________________________________\n");
//             // printf(" MCU id : %02x Cmd[0] :%02x , Cmd[1]:%02x ,Cmd[2]:%02x, Cmd[3]:%02x Cmd[4]:%02x Cmd[5]:%d\n",
//             //        res_temp[i].mcu_id, res_temp[i].cmd[0], res_temp[i].cmd[1], res_temp[i].cmd[2], res_temp[i].cmd[3], res_temp[i].cmd[4], t);
//             // printf("_________________________________________\n");
//         }
//     }
//     file_close(&file_ptr);

//     int n = file_size / 10;

//     if (reorder == true) {
//         // Bubble sort, starting from index 1
//         for (int i = 1; i < n - 1; i++) {
//             for (int j = 1; j < n - i; j++) {
//                 uint16_t t1 = (uint16_t)res_temp[j].cmd[3] << 8 | res_temp[j].cmd[4];
//                 uint16_t t2 = (uint16_t)res_temp[j + 1].cmd[3] << 8 | res_temp[j + 1].cmd[4];
//                 if (t1 > t2) {
//                     // Swap res_temp[j] and res_temp[j + 1]
//                     struct reservation_command temp = res_temp[j];
//                     res_temp[j] = res_temp[j + 1];
//                     res_temp[j + 1] = temp;
//                 }
//             }
//         }

//         // Print sorted commands, starting from index 1
//         for (int i = 1; i < n; i++) {
//             printf("Number of res command read is %d and temp is %d\n", i + 1, temp);
//             printf("_________________Sorted________________________\n");
//             printf("MCU id : %02x Cmd[0] :%02x , Cmd[1]:%02x ,Cmd[2]:%02x, Cmd[3]:%02x Time[0]:%02x Time[1]:%d executed:%d\n",
//                    res_temp[i].mcu_id, res_temp[i].cmd[0], res_temp[i].cmd[1], res_temp[i].cmd[2], res_temp[i].cmd[3], res_temp[i].cmd[4], res_temp[i].cmd[5], res_temp[i].executed, t);
//             printf("__________________Sorted_______________________\n");
//         }

//         // Write sorted commands back to the file, starting from index 1
//         fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_WRONLY | O_APPEND);
//         if (fd >= 0) {
//             for (int i = 1; i < n; i++) {
//                 file_write(&file_ptr, &res_temp[i], sizeof(struct reservation_command));
//             }
//         }
//         if (file_close(&file_ptr) < 0) {
//             file_close(&file_ptr);
//         }
//     } else {
//         publish_data(res_temp[1]);
//     }
// }

// // void sort_reservation_command(uint16_t file_size, bool reorder)
// // {
// //   struct file file_ptr;
// //   uint16_t temp = 0;

// //   int fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_RDONLY);
// //   struct reservation_command res_temp[16];
// //   uint16_t t = 0x00;
// //   struct reservation_command sorted;

// //   file_size = file_seek(&file_ptr, 0, SEEK_END);

// //   for (int i = 0; i < (file_size / 10); i++)
// //   {
// //     file_seek(&file_ptr, temp, SEEK_SET);
// //     temp += sizeof(struct reservation_command);
// //     if (file_read(&file_ptr, &res_temp[i], sizeof(struct reservation_command)) > 0)
// //     {
// //       printf("Number of res command read is %d and temp is %d\n", i + 1, temp);
// //       printf("_________________________________________\n");
// //       printf(" MCU id : %02x Cmd[0] :%02x , Cmd[1]:%02x ,Cmd[2]:%02x, Cmd[3]:%02x Cmd[4]:%02x Cmd[5]:%d\n",
// //              res_temp[i].mcu_id, res_temp[i].cmd[0], res_temp[i].cmd[1], res_temp[i].cmd[2], res_temp[i].cmd[3], res_temp[i].cmd[4], t);
// //       printf("_________________________________________\n");
// //     }
// //   }
// //   file_close(&file_ptr);

// //   int n = file_size / 10;

// //   // if (n <= 1 | res_temp[0].mcu_id ==0)
// //   {
// //     // printf("-------------------------------------------------------------------------------\n");
// //     // printf("Number of reservation command is : %d\n", n);

// //     // n = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_TRUNC | O_WRONLY);
// //     // file_close(&file_ptr);
// //     // printf("Number of reservation command is : %d\n", n);

// //     // printf("-------------------------------------------------------------------------------\n");
// //   }
// //   // else
// //   {

// //     if (reorder == true)
// //     {
// //       for (int i = 1; i < n - 1; i++)
// //       {
// //         for (int j = 1; j < n - i - 1; j++)
// //         {
// //           uint16_t t1 = (uint16_t)res_temp[j].cmd[3] << 8 | res_temp[j].cmd[4];
// //           uint16_t t2 = (uint16_t)res_temp[j + 1].cmd[3] << 8 | res_temp[j + 1].cmd[4];
// //           if (t1 > t2)
// //           {
// //             // Swap res_temp[j] and res_temp[j + 1]
// //             struct reservation_command temp = res_temp[j];
// //             res_temp[j] = res_temp[j + 1];
// //             res_temp[j + 1] = temp;
// //           }
// //         }
// //       }
// //       for (int i = 1; i < n - 1; i++)
// //       {
// //         {
// //           printf("Number of res command read is %d and temp is %d\n", i + 1, temp);
// //           printf("_________________Sorted________________________\n");
// //           printf("MCU id : %02x Cmd[0] :%02x , Cmd[1]:%02x ,Cmd[2]:%02x, Cmd[3]:%02x Time[0]:%02x Time[1]:%d executed:%d\n",
// //                  res_temp[i].mcu_id, res_temp[i].cmd[0], res_temp[i].cmd[1], res_temp[i].cmd[2], res_temp[i].cmd[3], res_temp[i].cmd[4], res_temp[i].cmd[5], res_temp[i].executed, t);
// //           printf("__________________Sorted_______________________\n");
// //         }
// //       }
// //       // fd = file_open(&file_ptr, "/mnt/fs/mfm/mtd_mainstorage/reservation_command.txt",O_WRONLY|O_TRUNC|O_APPEND);
// //       fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_WRONLY |O_APPEND);
// //       if (fd >= 0)
// //       {
// //         for (int i = 1; i < n - 1; i++)
// //         {
// //           file_write(&file_ptr, &res_temp[i], sizeof(struct reservation_command));
// //         }
// //       }
// //       if (file_close(&file_ptr) < 0)
// //       {
// //         file_close(&file_ptr);
// //       }
// //       // res_temp[0] = res_temp[1];
// //     }

// //   }
// //   if(reorder != true)
// //     publish_data(res_temp[1]);

// //   // else
// //     // publish_data(res_temp[0]);

// // }
// void publish_data(struct reservation_command res_temp)
// {
//   uint16_t t1 = (uint16_t)res_temp.cmd[3] << 8 | res_temp.cmd[4];
//   res_temp.latest_time = t1;
//   int adc_instance = 1;
//   // sorted = ;
//   int raw_afd = orb_advertise(ORB_ID(reservation_command), &res_temp);
//   if (raw_afd < 0)
//   {
//     syslog(LOG_ERR, "Raw ADC advertise failed. %i\n", raw_afd);
//   }
//   else
//   {
//     if (OK != orb_publish(ORB_ID(reservation_command), raw_afd, &res_temp))
//     {
//       syslog(LOG_ERR, "Orb Publish failed\n");
//     }
//     else
//     {

//       syslog(LOG_DEBUG, "Reservation Command Orb Published \n");
//     }
//   }
//     orb_unadvertise(raw_afd);

// }

void sort_reservation_command(uint16_t file_size, bool reorder)
{
  //     struct file file_ptr;
  //     uint16_t temp = 0;

  //     int fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_RDONLY);
  //     struct reservation_command res_temp[16];
  //     uint16_t t = 0x00;

  //     file_size = file_seek(&file_ptr, 0, SEEK_END);

  //     // Read reservation commands
  //     for (int i = 0; i < (file_size / 10); i++) {
  //         file_seek(&file_ptr, temp, SEEK_SET);
  //         temp += sizeof(struct reservation_command);
  //         if (file_read(&file_ptr, &res_temp[i], sizeof(struct reservation_command)) > 0) {
  //             // printf("Number of res command read is %d and temp is %d\n", i + 1, temp);
  //             // printf("_________________________________________\n");
  //             // printf(" MCU id : %02x Cmd[0] :%02x , Cmd[1]:%02x ,Cmd[2]:%02x, Cmd[3]:%02x Cmd[4]:%02x Cmd[5]:%d\n",
  //             //        res_temp[i].mcu_id, res_temp[i].cmd[0], res_temp[i].cmd[1], res_temp[i].cmd[2], res_temp[i].cmd[3], res_temp[i].cmd[4], t);
  //             // printf("_________________________________________\n");
  //         }
  //     }
  //     file_close(&file_ptr);

  //     int n = file_size / 10;

  //     if (reorder == true) {
  //         // Bubble sort, starting from index 1
  //         for (int i = 1; i < n - 1; i++) {
  //             for (int j = 1; j < n - i; j++) {
  //                 uint16_t t1 = (uint16_t)res_temp[j].cmd[3] << 8 | res_temp[j].cmd[4];
  //                 uint16_t t2 = (uint16_t)res_temp[j + 1].cmd[3] << 8 | res_temp[j + 1].cmd[4];
  //                 if (t1 > t2) {
  //                     // Swap res_temp[j] and res_temp[j + 1]
  //                     struct reservation_command temp = res_temp[j];
  //                     res_temp[j] = res_temp[j + 1];
  //                     res_temp[j + 1] = temp;
  //                 }
  //             }
  //         }

  //         // Print sorted commands, starting from index 1
  //         for (int i = 1; i < n; i++) {
  //             printf("Number of res command read is %d and temp is %d\n", i + 1, temp);
  //             printf("_________________Sorted________________________\n");
  //             printf("MCU id : %02x Cmd[0] :%02x , Cmd[1]:%02x ,Cmd[2]:%02x, Cmd[3]:%02x Time[0]:%02x Time[1]:%d executed:%d\n",
  //                    res_temp[i].mcu_id, res_temp[i].cmd[0], res_temp[i].cmd[1], res_temp[i].cmd[2], res_temp[i].cmd[3], res_temp[i].cmd[4], res_temp[i].cmd[5], res_temp[i].executed, t);
  //             printf("__________________Sorted_______________________\n");
  //         }

  //         // Write sorted commands back to the file, starting from index 1
  //         fd = open_file_flash(&file_ptr, MFM_MAIN_STRPATH, RESERVATION_CMD, O_WRONLY | O_APPEND);
  //         if (fd >= 0) {
  //             for (int i = 1; i < n; i++) {
  //                 file_write(&file_ptr, &res_temp[i], sizeof(struct reservation_command));
  //             }
  //         }
  //         if (file_close(&file_ptr) < 0) {
  //             file_close(&file_ptr);
  //         }
  //     } else {
  //         // publish_data(res_temp[1]);
  //     }
}

void maintain_data_consistency()
{
  struct file mfm_file_pointer, sfm_file_pointer;
  int mfm_fd, sfm_fd;
  uint32_t mfm_read, sfm_read, sfm_seek_pointer_status = 0, mfm_seek_pointer_status = 0;
  uint8_t temp[2000], count = 0;

  char filename[4][30] = {"/flags.txt", "/satHealth.txt", "/satellite_Logs.txt", "/reservation_table.txt"}; // "/cam_nir.txt", "/epdm.txt", "/adcs.txt"};
  for (int i = 0; i < 4; i++)
  {
    mfm_fd = file_open_flash(mfm_file_pointer, MFM_MAIN_STRPATH, filename[i], O_RDWR);
    sfm_fd = file_open_flash(mfm_file_pointer, SFM_MAIN_STRPATH, filename[i], O_RDWR);
    if (mfm_fd >= 0)
    {
      mfm_read = file_seek(&mfm_file_pointer, 0, SEEK_END);
    }

    if (sfm_fd >= 0)
    {
      sfm_read = file_seek(&sfm_file_pointer, 0, SEEK_END);
    }
    if (mfm_fd >= 0 & sfm_fd >= 0)
    {
      if (mfm_read != sfm_read)
      {
        if (mfm_read < sfm_read)
        {
          sfm_seek_pointer_status = mfm_read;
          mfm_seek_pointer_status = sfm_read;
          do
          {
            file_seek(&sfm_file_pointer, sfm_seek_pointer_status, SEEK_SET);
            if (sfm_read - count > 2000)
            {
              count = 2000;
            }
            else
            {
              count = sfm_read - count;
            }
            if (file_read(&sfm_file_pointer, temp, count) >= 0)
            {
              ssize_t written = file_write(&mfm_file_pointer, temp, count);
              printf("Data of size %d has been writtern\n", written);
            }
            sfm_seek_pointer_status += count;

          } while (sfm_seek_pointer_status <= sfm_read);
        }
        if (mfm_read > sfm_read)
        {
          sfm_seek_pointer_status = mfm_read;
          mfm_seek_pointer_status = sfm_read;
          do
          {
            file_seek(&mfm_file_pointer, mfm_seek_pointer_status, SEEK_SET);
            if (sfm_read - count > 2000)
            {
              count = 2000;
            }
            else
            {
              count = sfm_read - count;
            }
            if (file_read(&mfm_file_pointer, temp, count) >= 0)
            {
              ssize_t written = file_write(&sfm_file_pointer, temp, count);
              printf("Data of size %d has been writtern\n", written);
            }
            mfm_seek_pointer_status += count;
          } while (mfm_seek_pointer_status <= mfm_read);
        }
      }
    }
    file_close(&mfm_file_pointer);
    file_close(&sfm_file_pointer);
  }
}

void send_data_uorb(char path[200], uint32_t address, uint16_t num_of_packets)
{
  int adc_instance = 10;

  struct flash_operation flash;
  int raw_afd = orb_advertise_multi_queue_persist(ORB_ID(flash_operation), &flash,
                                                  &adc_instance, sizeof(struct flash_operation));
  int count = 0,pkt=0;
  // while (1)
  {
    flash.timestamp = (uint64_t)time(NULL);
    if (raw_afd < 0)
    {
      syslog(LOG_ERR, "Raw ADC advertise failed. %i\n", raw_afd);
    }
    else
    {

      {
        printf("\nData sent :\nTimestamp : %d \nPacket Type:%d \nPacket Number:%d \n",
               flash.timestamp,
               flash.packet_type,
               flash.packet_number);
        struct file fp;
        int fd = file_open(&fp, path, O_RDONLY);
        uint8_t data[80] = {'\0'};
        if (fd >= 0)
        {
          count = address;
          do
          {
            printf("file has been opened with name %s\n", path);
            printf("\n----------------------------\n");
            pkt++;
            //TODO:read data at once and send it accordingly instead

            file_seek(&fp, count, SEEK_SET);
            // file_read(&fp, &flash.data, sizeof(data));
            file_read(&fp, data, sizeof(data));
            // flash.data = data;
            for(int i=0;i<80;i++){
              printf("%02x ", data[i]);
              flash.data[i] = data[i];
            }
            flash.packet_type=0x0a;
            flash.packet_number= pkt;
            printf("cc: %d num_of_pkt:%d   Packet number:%d\n",count,num_of_packets, flash.packet_number);
            printf("\n----------------------------\n");
            count += 80;

            if (OK != orb_publish(ORB_ID(flash_operation), raw_afd, &flash))
            {
              syslog(LOG_ERR, "Orb Publish failed\n");
            }
            else
            {
              syslog(LOG_ERR, "Orb data published\n");
            }
            // if(num_of_packets == pkt) break;
            num_of_packets -= 1;

            sleep(1);
          } while (num_of_packets > 0);
        }
        file_close(&fp);
      }
    }
  }
  // orb_unadvertise(raw_afd);
}
