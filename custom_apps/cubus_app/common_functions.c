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
pthread_mutex_t flash_mutex = PTHREAD_MUTEX_INITIALIZER;

void mission_data(char *filename, uint8_t *data, uint16_t size1)
{
  struct file file_p;
  // TODO: discuss and figure out if we need to set limit to size of file and truncate contents once the file size limit is reached ...
  if (open_file_flash(&file_p, "/mnt/fs/mfm", filename, O_CREAT | O_RDWR | O_APPEND) >= 0)
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


int clear_int_flag(){
  CRITICAL_FLAGS c={'\0'};
  struct file fp;
  int fd;
  pthread_mutex_lock(&flash_mutex); // Lock the mutex

  fd = open("/dev/intflash", O_WRONLY);
  up_progmem_eraseblock(22);
  up_progmem_write(FLAG_DATA_INT_ADDR, c, sizeof(CRITICAL_FLAGS));

  close(fd);
  // fd = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_TRUNC);
  // file_close(&fp);
  pthread_mutex_unlock(&flash_mutex); // Lock the mutex

}

int store_flag_data( CRITICAL_FLAGS *flag_data)
{
  struct file fp;
  int bwr;
  printf("\n**************SToring flag data*********\n");
  print_critical_flag_data(flag_data);
  // int fd;

  pthread_mutex_lock(&flash_mutex); // Lock the mutex

  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  {
      // Internal flash file opened successfully
      up_progmem_eraseblock(22);
      up_progmem_write(FLAG_DATA_INT_ADDR, flag_data, sizeof(CRITICAL_FLAGS));

      // if (close(fd) < 0)
      // {
      //     syslog(LOG_ERR, "Error closing internal flash file descriptor.\n");
      // }
  }
  else
  {
      syslog(LOG_ERR, "Error opening internal flash to store new flag data ... \n");
  }
  close(fd);
  
  int fd1 = open_file_flash(&fp, MFM_MAIN_STRPATH, file_name_flag, O_RDWR);
  if (fd1 >= 0)
  {
    file_truncate(&fp, sizeof(CRITICAL_FLAGS)); // Clearing out any previous data
    bwr = file_write(&fp, &flag_data, sizeof(CRITICAL_FLAGS));
    if (bwr == 0 || bwr != sizeof(CRITICAL_FLAGS))
    {
      syslog(LOG_ERR, "Error in writing flag data to MFM\n Will try once again without verifying...\n");

      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      bwr = file_write(&fp, &flag_data, sizeof(CRITICAL_FLAGS));
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
  // print_critical_flag_data(&critic_flags);

  pthread_mutex_unlock(&flash_mutex); // Unlock the mutex
  return 0;
}

int check_flag_data()
{

  CRITICAL_FLAGS rd_flags_int;

  CRITICAL_FLAGS rd_flags_mfm = {0xff};
  uint8_t mfm_have_data = 0;
  ssize_t read_size_mfm = 0;
  struct file fp;

  // irqstate_t flags;
  // flags = enter_critical_section();
  int fd = open("/dev/intflash", O_RDWR);
  if (fd >= 0)
  { // internal flash file opened successfully
    syslog(LOG_INFO, "Printing Internal flash flag data.\n");
    up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
    print_critical_flag_data(&rd_flags_int);
  }
  else
  {
    syslog(LOG_ERR, "Error opening internal flash atempt 1......\n ");
  }

  close(fd);
  rd_flags_int.RST_COUNT+=1;
  // store_flag_data(0,&rd_flags_int);
  // store_critical_flags(&rd_flags_int);
  // TODO: discuss and decide whether we want to append the flags data or if we want flags data to be stored only one on the same folders ...
  // INFO: for now, system will overwrite the previous flag data if it needs to update it, it can be changed by changing file open mode and will need file seek to set cursor/pointer to read data

  int fd1 = file_open(&fp, "/mnt/fs/mfm/mtd_mainstorage/flags.txt", O_RDWR);
  if (fd1 >= 0)
  {
    read_size_mfm = file_read(&fp, &rd_flags_mfm, sizeof(CRITICAL_FLAGS));
  }

  // TODO: add code for shared flash memory as well.. for now only main flash and internal flash data are read....
  if (rd_flags_int.ANT_DEP_STAT == 0xff)
  { // no data available in internal flash
    if (read_size_mfm == 0 || read_size_mfm != sizeof(CRITICAL_FLAGS))
    { // no data in mfm or data is corrupted
      syslog(LOG_INFO, "Data not available in internal flash....\n Data corrupted or not available in main flash\n Read size from mfm %d \n", read_size_mfm);

      syslog(LOG_INFO, "Printing Internal flash flag data.\n");
      // print_critical_flag_data(&rd_flags_int);

      syslog(LOG_INFO, "Initializing all the flags to default...\n");
      critic_flags.ANT_DEP_STAT = UNDEPLOYED;
      critic_flags.KILL_SWITCH_STAT = KILL_SW_OFF;
      critic_flags.OPER_MODE = NRML_MODE; // later on we check for battery voltage, now set default normal mode
      critic_flags.RSV_FLAG = RSV_NOT_RUNNING;
      critic_flags.UL_STATE = UL_NOT_RX;

      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      file_write(&fp, &critic_flags, sizeof(CRITICAL_FLAGS));
      file_close(&fp);

      fd = open("/dev/intflash", O_RDWR); // open internal flash to write
      if (fd >= 0)
      {
        up_progmem_eraseblock(22);
        up_progmem_write(0x081C0000, &critic_flags, sizeof(CRITICAL_FLAGS));
      }
      else
      {
        syslog(LOG_ERR, "Unable to open internal flash to write flag data ... \n");
      }
      close(fd); // closing internal flash just after writing
    }
    else
    { // data available in main flash and is equal to the expected buffer size
      syslog(LOG_INFO, "Flag data available in Main Flash Memory only.. Data size: %d\n", read_size_mfm);
      file_close(&fp);

      syslog(LOG_INFO, "Printing Internal flash flag data.\n");
      // print_critical_flag_data(&rd_flags_int);

      syslog(LOG_INFO, "copying from main flash to internal flash ...\n");
      critic_flags.ANT_DEP_STAT = rd_flags_mfm.ANT_DEP_STAT;
      critic_flags.KILL_SWITCH_STAT = rd_flags_mfm.KILL_SWITCH_STAT;
      critic_flags.OPER_MODE = rd_flags_mfm.OPER_MODE;
      critic_flags.RSV_FLAG = rd_flags_mfm.RSV_FLAG;
      critic_flags.UL_STATE = rd_flags_mfm.UL_STATE;

      // fd = open("/dev/intflash", O_RDWR);
      // if (fd >= 0)
      // {
      //   up_progmem_eraseblock(22);
      //   up_progmem_write(0x081C0000, &critic_flags, sizeof(CRITICAL_FLAGS));
      // }
      // else
      // {
      //   syslog(LOG_ERR, "Unable to open internal flash to write flags data\n");
      // }
      // close(fd);
      store_flag_data(&critic_flags);
    }
  }
  else
  { // data available in internal flash memory
    if (read_size_mfm == 0 || read_size_mfm != sizeof(CRITICAL_FLAGS))
    { // data not available in main flash
      syslog(LOG_INFO, "Data available in internal flash memory only...\n Copying from internal flash to main flash\n");
      critic_flags.ANT_DEP_STAT = rd_flags_int.ANT_DEP_STAT;
      critic_flags.KILL_SWITCH_STAT = rd_flags_int.KILL_SWITCH_STAT;
      critic_flags.OPER_MODE = rd_flags_int.OPER_MODE;
      critic_flags.RSV_FLAG = rd_flags_int.RSV_FLAG;
      critic_flags.UL_STATE = rd_flags_int.UL_STATE;

      syslog(LOG_INFO, "Printing Internal flash flag data.\n");
      // print_critical_flag_data(&rd_flags_int);
      // close(fd);  //closing internal flash
      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      file_write(&fp, &critic_flags, sizeof(CRITICAL_FLAGS));
      file_close(&fp);
    }
    else
    { // data available in both flash memories...
      // TODO: add sfm as well and check for data discrepancy from all three flash memories and go for data from majority ones
      // INFO: for now, data from internal flash is trusted... data from mfm is read just for debugging purposes (without three copies checking for data discrepancy makes no sense)
      syslog(LOG_INFO, "data available in both main flash and internal flash memories\n ");

      syslog(LOG_INFO, "Printing Main flash flag data....\n");
      // print_critical_flag_data(&rd_flags_mfm);

      syslog(LOG_INFO, "Printing Internal flash flag data.\n");
      print_critical_flag_data(&rd_flags_int);

      int data_check = 0;
      critic_flags.ANT_DEP_STAT = rd_flags_int.ANT_DEP_STAT;
      critic_flags.KILL_SWITCH_STAT = rd_flags_int.KILL_SWITCH_STAT;
      critic_flags.OPER_MODE = rd_flags_int.OPER_MODE;
      critic_flags.RSV_FLAG = rd_flags_int.RSV_FLAG;
      critic_flags.UL_STATE = rd_flags_int.UL_STATE;
      // close(fd);
      file_truncate(&fp, sizeof(CRITICAL_FLAGS));
      file_write(&fp, &critic_flags, sizeof(CRITICAL_FLAGS));
      file_close(&fp);
    }
  }
  file_close(&fp);
  syslog(LOG_INFO, "Flags data check and write complete.... \n");
  // leave_critical_section(flags);
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

void print_critical_flag_data(CRITICAL_FLAGS *flags)
{
  CRITICAL_FLAGS rd_flags_int = {0xff};

  int fd = open("/dev/intflash", O_RDONLY);
  if (fd >= 0)
  { // internal flash file opened successfully
    syslog(LOG_INFO, "Printing Internal flash flag data.\n");
    up_progmem_read(FLAG_DATA_INT_ADDR, &rd_flags_int, sizeof(rd_flags_int));
    // print_critical_flag_data(&rd_flags_int);
  }

  printf(" ********************************************\r\n");
  printf(" |   Antenna Deployment Status \t %d \t|\r\n", flags->ANT_DEP_STAT);
  printf(" |   Kill Switch Status        \t %d \t|\r\n", flags->KILL_SWITCH_STAT);
  printf(" |   Operation Mode            \t %d \t|\r\n", flags->OPER_MODE);
  printf(" |   Reservation Table Flag    \t %d \t|\r\n", flags->RSV_FLAG);
  printf(" |   Command uplink status     \t %d \t|\r\n", flags->UL_STATE);
  printf(" |   Reset counter     \t %d \t|\r\n", flags->RST_COUNT);

  printf(" ********************************************\r\n");
  close(fd);
}

struct KILL_SW{
 time_t timestamps[3];
    int count;
};
void add_command_timestamp(struct KILL_SW *kill_sw, time_t timestamp) {
    if (kill_sw->count < 3) {
        kill_sw->timestamps[kill_sw->count++] = timestamp;
    } else {
        for (int i = 1; i < 3; ++i) {
            kill_sw->timestamps[i - 1] = kill_sw->timestamps[i];
        }
        kill_sw->timestamps[3 - 1] = timestamp;
    }
}

int check_command_times(struct KILL_SW *kill_sw) {
    if (kill_sw->count < 3) {
        return 0;
    }
    
    time_t first_time = kill_sw->timestamps[0];
    time_t last_time = kill_sw->timestamps[3 - 1];
    
    return (difftime(last_time, first_time) <= 600);
}

int receive_command(struct KILL_SW *kill_sw) {
    time_t current_time = time(NULL);
    add_command_timestamp(kill_sw, current_time);

    if (check_command_times(kill_sw)) {
        // save_data();
        // kill_sw.
        // Reset kill_sw after saving data
        kill_sw->count = 0;
        return 0;
    }
    return 1;
}
