// /****************************************************************************
//  * apps/custom_apps/cubus_app/common_functions.c
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

#include "common_functions.h"
// #include <mqueue.h>

// CRITICAL_FLAGS critic_flags;

// /****************************************************************************
//  * Name: adc_main
//  ****************************************************************************/

// /*
// Write to a queue
// */

// void writer_mq_edited(satellite_health_s *str)
// {
//   struct mq_attr attr;
//   mqd_t mqd;
//   char buffer[1024];
//   // Initialize attributes
//   attr.mq_flags = 0;
//   attr.mq_maxmsg = 10;
//   attr.mq_msgsize = 8192;
//   attr.mq_curmsgs = 0;

//   mqd = mq_open("/gpio", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR, NULL);
//   if (mqd == (mqd_t)-1)
//   {
//     perror("mq_open");
//     exit(1);
//   }

//   // Dynamically allocated array of strings
//   // char *str[] = {"A", "posix", "message", "queue", "example", "kjsalkfjsdaf","exit"};
//   // char *str ;
//   // str = arg1;
//   else
//   {
//     printf("size of %d/ %d is %f", strlen(str), sizeof(str), sizeof(str[0]));
//     int str_count = sizeof(str) / sizeof(str[0]);

//     printf("Writing messages to the POSIX message queue\n\n");
//     printf("Number of strings: %d\n", str_count);
//     // printf("MSG is %s\n",arg1);
//     // for (int i = 0; i < str_count; i++)
//     {
//       sprintf(buffer, "%0.02f %0.02f %0.02f \0", str->accl_x, str->accl_y, str->accl_z);
//       // Write to the POSIX message queue

//       if (mq_send(mqd, buffer, strlen(buffer), 0) == -1) // +1 to include the null terminator
//       // if (mq_send(mqd, str[i], strlen(str[i]) + 1, 0) == -1)
//       { // +1 to include the null terminator

//         perror("mq_send");
//         exit(1);
//       }
//       // printf("Data sent: %s\n", str[i]);
//     }

//     if (mq_close(mqd) == -1)
//     {
//       perror("mq_close");
//       exit(1);
//     }
//   }
// }

// void reader_mq_edited()
// {
//   struct mq_attr attr;
//   mqd_t mqd;

//   // Initialize attributes
//   attr.mq_flags = 0;
//   attr.mq_maxmsg = 10;
//   attr.mq_msgsize = 8192;
//   attr.mq_curmsgs = 0;

//   mqd_t mq;
//   char buffer[1024 + 1];
//   // satellite_health_s buffer;
//   ssize_t bytes_read;

//   // Open the message queue
//   mq = mq_open("/gpio", O_RDONLY);
//   if (mq == (mqd_t)-1)
//   {
//     // perror("mq_open");
//     exit(1);
//   }
//   else
//   {
//     // printf("Waiting for messages...\n");
//     usleep(10000);

//     // while(1)
//     {
//       // Receive the message
//       bytes_read = mq_receive(mq, buffer, sizeof(buffer), NULL);
//       if (bytes_read == -1)
//       {
//         perror("mq_receive");
//         exit(1);
//       }
//       // buffer[bytes_read] = '\0';  // Null-terminate the string

//       printf("*******\n*************Size of mqueue msg :%d , Received: %s\n*******\n", bytes_read, buffer);
//       for (ssize_t i = 0; i < bytes_read; i++)
//       {
//         if (buffer[i] == '\0')
//         {
//           printf("\\0"); // Print null characters as "\0"
//         }
//         else
//         {
//           putchar(buffer[i]);
//         }
//       }
//       printf("\n*******\n");
//       sleep(2);
//       // Exit if the received message is "exit"
//     }

//     // Cleanup
//     if (mq_close(mq) == -1)
//     {
//       perror("mq_close");
//       exit(1);
//     }

//     if (mq_unlink("/gpio") == -1)
//     {
//       perror("mq_unlink");
//       exit(1);
//     }
//   }
// }

void mission_data(char *filename, uint8_t *data, uint16_t size1)
{
  struct file file_p;
  // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
  if (open_file_flash(&file_p, "/mnt/fs/mfm/mtd_mainstorage", filename, O_CREAT | O_RDWR | O_APPEND) >= 0)
  {
    ssize_t bytes_written = file_write(&file_p, data, size1);
    // writer_mq_edited(sat_health_data);
    if (bytes_written > 0)
    {
      syslog(LOG_INFO, "Satellite Health data write Successful.\nData Len: %d.\n", bytes_written);
      file_close(&file_p);
    }
    else
    {
      syslog(LOG_INFO, "Write Failure.\n");
    }
    file_syncfs(&file_p);
    file_close(&file_p);
  }
  else
  {
    syslog(LOG_ERR, "Error opening file to write satellite health data..\n");
  }
  file_close(&file_p);
  // TODO delter this later
}

// void retrieve_latest_sat_health_data(satellite_health_s *sat_health_buf)
// {
//   printf("**************************\n***********************************************************\n Reading MFM data\n");
//   struct stat st;
//   struct file fptr;
//   int fd = 0;

//   fd = open_file_flash(&fptr, SFM_MAIN_STRPATH, file_name_sat_health, O_RDONLY);
//   if (fd >= 0)
//   {
//     int size_file = file_seek(&fptr, 0, SEEK_END);
//     int off = file_seek(&fptr, size_file - 112, SEEK_SET);

//     // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
//     printf("Size of file : %d \n Offset: %d \n  ", size_file, off);
//     ssize_t bytes_read = file_read(&fptr, sat_health_buf, sizeof(satellite_health_s));
//     if (bytes_read > 0)
//     {
//       syslog(LOG_INFO, "Flash Read Successful.\nData Len: %d.\n", bytes_read);
//       file_close(&fptr);
//       print_satellite_health_data(sat_health_buf);
//     }
//     else
//     {
//       syslog(LOG_INFO, "Read Failure.\n");
//     }
//     file_syncfs(&fptr);
//     file_close(&fptr);
//   }
//   else
//   {
//     syslog(LOG_ERR, "Error opening file to read satellite health data..\n");
//   }
//   file_close(&fptr);

//   printf("**************************\n***********************************************************\n Reading MFM data completed\n");
// }

// void retrieve_sat_health_data(satellite_health_s sat_health_buf[], int times)
// {
//   struct stat st;
//   struct file fptr;
//   int fd = 0;
//   fd = open_file_flash(&fptr, MFM_MAIN_STRPATH, file_name_sat_health, O_RDONLY);
//   if (fd >= 0)
//   {
//     int size_file = file_seek(&fptr, 0, SEEK_END);
//     int off = file_seek(&fptr, size_file - 112 * times, SEEK_SET);
//     printf("Size of file : %d \n Offset: %d \n  ", size_file, off);
//     for (int i = 0; i < times; i++)
//     {
//       // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
//       ssize_t bytes_read = file_read(&fptr, &sat_health_buf[i], sizeof(satellite_health_s));
//       if (bytes_read > 0)
//       {
//         syslog(LOG_INFO, "Flash Read Successful.\nData Len: %d.\n", bytes_read);
//         file_close(&fptr);
//         print_satellite_health_data(&sat_health_buf[i]);
//       }
//       else
//       {
//         syslog(LOG_INFO, "Read Failure.\n");
//       }
//     }
//     file_syncfs(&fptr);
//     file_close(&fptr);
//   }
//   else
//   {
//     syslog(LOG_ERR, "Error opening file to read satellite health data..\n");
//   }
//   file_close(&fptr);
// }

pthread_mutex_t flash_mutex = PTHREAD_MUTEX_INITIALIZER;
int store_flag_data(int fd, CRITICAL_FLAGS *flag_data)
{
  struct file fp;
  int bwr;
  // int fd;

  // pthread_mutex_lock(&flash_mutex); // Lock the mutex

  // fd = open("/dev/intflash", O_RDWR);
  // if (fd >= 0)
  // {
  //     // Internal flash file opened successfully
  //     up_progmem_eraseblock(22);
  //     up_progmem_write(FLAG_DATA_INT_ADDR, flag_data, sizeof(CRITICAL_FLAGS));

  //     // if (close(fd) < 0)
  //     // {
  //     //     syslog(LOG_ERR, "Error closing internal flash file descriptor.\n");
  //     // }
  // }
  // else
  // {
  //     syslog(LOG_ERR, "Error opening internal flash to store new flag data ... \n");
  // }

  int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
  if (fd1 >= 0)
  {
    file_truncate(&fp, sizeof(CRITICAL_FLAGS)); // Clearing out any previous data
    bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
    if (bwr == 0 || bwr != sizeof(CRITICAL_FLAGS))
    {
      syslog(LOG_ERR, "Error in writing flag data to MFM\n Will try once again without verifying...\n");

      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      bwr = file_write(&fp, flag_data, sizeof(CRITICAL_FLAGS));
      syslog(LOG_INFO, "Size of flag data written to MFM on second attempt: %d", bwr);
    }

    if (file_close(&fp) < 0)
    {
      syslog(LOG_ERR, "Error closing main flash file descriptor.\n");
    }
  }
  else
  {
    syslog(LOG_ERR, "Unable to open %s%s for writing critical flash data\n", MFM_MAIN_STRPATH, file_name_flag);
  }
  syslog(LOG_DEBUG, "\n-----Storing data to flash-----\n");
  print_critical_flag_data(&critic_flags);

  pthread_mutex_unlock(&flash_mutex); // Unlock the mutex

  return 0;
}

int check_flag_data()
{
  CRITICAL_FLAGS rd_flags_int = {0};
  // CRITICAL_FLAGS rd_flags_mfm = {255, 255, 255, 255, 255, 255}; // = {0xff};
  ssize_t read_size_mfm = 0;
  struct file fp;
  int fd;

  int fd1 = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_RDWR);
  // open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
  if (fd1 >= 0)
  {
    read_size_mfm = file_read(&fp, &rd_flags_int, sizeof(CRITICAL_FLAGS));
    printf("got read size : %d \n", read_size_mfm);
  }
  if(file_close(&fp) >= 0){
    syslog(LOG_DEBUG, "File closed success\n");
  }
  if (rd_flags_int.ANT_DEP_STAT == DEPLOYED && rd_flags_int.UL_STATE == UL_RX)
  {
  }
  else
  {
    syslog(LOG_INFO, "No valid data found in internal or main flash.\nInitializing all flags to default...\n");
    memset(&critic_flags, 0, sizeof(CRITICAL_FLAGS));
    critic_flags.ANT_DEP_STAT = UNDEPLOYED;
    critic_flags.KILL_SWITCH_STAT = KILL_SW_OFF;
    critic_flags.OPER_MODE = NRML_MODE;
    critic_flags.RSV_FLAG = RSV_NOT_RUNNING;
    critic_flags.UL_STATE = UL_NOT_RX;
    read_size_mfm = file_write(&fp, &critic_flags, sizeof(CRITICAL_FLAGS));
    printf("got read size : %d \n", read_size_mfm);
    write_to_mfm("/mnt/fs/mfm/mtd_mainstorage/flags.txt", &critic_flags);

  }
  

  // pthread_mutex_unlock(&flash_mutex); // Unlock the mutex
  // store_flag_data(fd, &critic_flags);

  print_critical_flag_data(&critic_flags);
  return 0;
}

void write_to_mfm(char *path, CRITICAL_FLAGS *flag_data)
{
  int fd;
  struct file fptr;
  int8_t writeBytes;
  fd = file_open(&fptr, path, O_CREAT | O_RDWR);
  if (fd > 0)
  {
    writeBytes = file_write(&fptr, flag_data, sizeof(CRITICAL_FLAGS));
    if (writeBytes > 0)
    {
      syslog(LOG_SYSLOG, "Flag pointer updated successfully\n");
    }
    else
    {
      writeBytes = file_write(&fptr, flag_data, sizeof(CRITICAL_FLAGS));
      if (writeBytes > 0)
      {
        syslog(LOG_SYSLOG, "Flag pointer updated successfully\n");
      }
    }
  }

  file_close(&fptr);
  close(fd);
}
/*

// /*
//  */

// void Setup()
// {
//   int fd = 0;
//   struct file flp1, flp2, flp3, flp4;
//   fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_CREAT);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Could not create file named sat_health... \n");
//   }
//   file_close(&flp1);

//   /*delete this later
//   */
//      fd = open_file_flash(&flp1, MFM_MAIN_STRPATH, file_name_sat_health, O_TRUNC);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Could not create file named sat_health... \n");
//   }
//   file_close(&flp1);
//   close(fd);
//  return 0;
// /*
//   */
//   fd = open_file_flash(&flp2, MFM_MAIN_STRPATH, file_name_flag, O_CREAT);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Could not create flags.txt data file ... \n");
//   }
//   file_close(&flp2);

//   fd = open_file_flash(&flp3, MFM_MSN_STRPATH, file_name_cam_msn, O_CREAT);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "could not create cam.txt msn file ...\n");
//   }
//   file_close(&flp3);

//   fd = open_file_flash(&flp4, MFM_MSN_STRPATH, file_name_epdm_msn, O_CREAT);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Could not create epdm.txt msn file\n");
//   }
//   file_close(&flp4);

//   syslog(LOG_INFO, "Checking initial flag data...\n");
//   check_flag_data();
//   print_critical_flag_data(&critic_flags);
// }

// /*
// //  */
// void print_satellite_health_data(satellite_health_s *sat_health)
// {
//   printf(" *******************************************\r\n");
//   printf(" |   X axis acceleration    \t %f \t|\r\n", sat_health->accl_x);
//   printf(" |   Y axis acceleration    \t %f \t|\r\n", sat_health->accl_y);
//   printf(" |   Z axis acceleration    \t %f \t|\r\n", sat_health->accl_z);

//   printf(" |   X axis Gyro data       \t %f \t|\r\n", sat_health->gyro_x);
//   printf(" |   Y axis Gyro data       \t %f \t|\r\n", sat_health->gyro_y);
//   printf(" |   Z axis gyro data       \t %f \t|\r\n", sat_health->gyro_z);

//   printf(" |   X axis magnetic field  \t %f \t|\r\n", sat_health->mag_x);
//   printf(" |   Y axis magnetic field  \t %f \t|\r\n", sat_health->mag_y);
//   printf(" |   Z axis magnetic field  \t %f \t|\r\n", sat_health->mag_z);

//   printf(" |   Solar Panel 1 Voltage: \t %d \t|\r\n", sat_health->sol_p1_v);
//   printf(" |   Solar Panel 2 Voltage: \t %d \t|\r\n", sat_health->sol_p2_v);
//   printf(" |   Solar Panel 3 Voltage: \t %d \t|\r\n", sat_health->sol_p3_v);
//   printf(" |   Solar Panel 4 Voltage: \t %d \t|\r\n", sat_health->sol_p4_v);
//   printf(" |   Solar Panel 5 Voltage: \t %d \t|\r\n", sat_health->sol_p5_v);
//   printf(" |   Solar Panel T Voltage: \t %d \t|\r\n", sat_health->sol_t_v);
//   printf(" |--------------------------------------|\r\n");
//   printf(" |   Solar Panel 1 Current: \t %d \t|\r\n", sat_health->sol_p1_c);
//   printf(" |   Solar Panel 2 Current: \t %d \t|\r\n", sat_health->sol_p2_c);
//   printf(" |   Solar Panel 3 Current: \t %d \t|\r\n", sat_health->sol_p3_c);
//   printf(" |   Solar Panel 4 Current: \t %d \t|\r\n", sat_health->sol_p4_c);
//   printf(" |   Solar Panel 5 Current: \t %d \t|\r\n", sat_health->sol_p5_c);
//   printf(" |   Solar Panel T Current: \t %d \t|\r\n", sat_health->sol_t_c);
//   printf(" |--------------------------------------|\r\n");
//   printf(" |   Unreg Line Current:    \t %d \t|\r\n", sat_health->unreg_c);
//   printf(" |   Main 3v3 Current:      \t %d \t|\r\n", sat_health->v3_main_c);
//   printf(" |   COM 3v3 Current:       \t %d \t|\r\n", sat_health->v3_com_c);
//   printf(" |   5 Volts line Current:  \t %d \t|\r\n", sat_health->v5_c);
//   printf(" |   3v3 2 line Current:    \t %d \t|\r\n", sat_health->v3_2_c);
//   printf(" |--------------------------------------|\r\n");
//   printf(" |   Raw Current:           \t %d \t|\r\n", sat_health->raw_c);
//   printf(" |   Raw Voltage:           \t %d \t|\r\n", sat_health->raw_v);
//   printf(" |--------------------------------------|\r\n");
//   printf(" |   Battery Total Voltage: \t %d \t|\r\n", sat_health->batt_volt);
//   printf(" |   Battery Total Current: \t %d \t|\r\n", sat_health->batt_c);
//   printf(" |   Battery Temperature:   \t %d \t|\r\n", sat_health->temp_batt);
//   printf(" *********************************************\r\n");
// }

// /*
//  */
void print_critical_flag_data(CRITICAL_FLAGS *flags)
{
  CRITICAL_FLAGS rd_flags_int = {0xff};

  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  { // internal flash file opened successfully
    syslog(LOG_INFO, "Printing Internal flash flag data.\n");
    up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
    print_critical_flag_data(&rd_flags_int);
  }

  printf(" ********************************************\r\n");
  printf(" |   Antenna Deployment Status \t %d \t|\r\n", flags->ANT_DEP_STAT);
  printf(" |   Kill Switch Status        \t %d \t|\r\n", flags->KILL_SWITCH_STAT);
  printf(" |   Operation Mode            \t %d \t|\r\n", flags->OPER_MODE);
  printf(" |   Reservation Table Flag    \t %d \t|\r\n", flags->RSV_FLAG);
  printf(" |   Command uplink status     \t %d \t|\r\n", flags->UL_STATE);
  printf(" |   Reset counter     \t %d \t|\r\n", flags->RST_COUNT);

  printf(" ********************************************\r\n");
}

// /*
//  */
// int gpio_write1(uint32_t pin, uint8_t mode)
// {

//   gpio_config_s gpio_numval;
//   int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
//     close(fd);
//     return -1;
//   }
//   gpio_numval.gpio_num = pin;
//   gpio_numval.gpio_val = mode;
//   if (gpio_numval.gpio_val > 1 || gpio_numval.gpio_num < 0)
//   {
//     syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
//     return -2;
//   }
//   int ret = write(fd, (const void *)&gpio_numval, sizeof(gpio_config_s));
//   close(fd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Unable to write to gpio pin...\n");
//   }
//   return ret;
// }

// int gpio_read(uint32_t pin)
// {
//   gpio_config_s gpio_read;
//   int fd = open(ETX_LED_DRIVER_PATH, O_WRONLY);
//   if (fd < 0)
//   {
//     syslog(LOG_ERR, "Error opening %s for GPIO WRITE...", ETX_LED_DRIVER_PATH);
//     close(fd);
//     return -1;
//   }
//   gpio_read.gpio_num = pin;
//   if (gpio_read.gpio_num < 0)
//   {
//     syslog(LOG_ERR, "Undefined GPIO pin or set mode selected...\n");
//     return -2;
//   }
//   int ret = read(fd, (const void *)&gpio_read, sizeof(gpio_config_s));
//   close(fd);
//   if (ret < 0)
//   {
//     syslog(LOG_ERR, "Unable to write to gpio pin...\n");
//     return -3;
//   }
//   return gpio_read.gpio_val;
// }