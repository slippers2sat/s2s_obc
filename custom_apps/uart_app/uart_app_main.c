/****************************************************************************
 * apps/examples/uart_app/uart_app_main.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "uart_app_main.h"


/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Name: main
 ****************************************************************************/

void cmd_com();
void cmd_epdm();
void cmd_adcs();
void cmd_cam();

int main(int argc, FAR char *argv[])
{
    printf("uart main is working");
    int ret;
    // ret = task_create("UART COM",10,2048,cmd_com,NULL);
    // ret = task_create("UART EPDM",10,2048,cmd_epdm,NULL);
    // ret = task_create("UART ADCS",10,2048,cmd_adcs,NULL);
    // ret = task_create("UART CAM",10,2048,cmd_cam,NULL);
    while(1){
        cmd_epdm();
        usleep(10000);
    }


}

void cmd_com(){
    printf("COM Receiving mode in command\n");
    
}

void cmd_epdm(){
    printf("EPDM Receiving mode in command\n");
    int fd,ret,rxBuff[5];
    int data = 0x23;
    fd = open("/dev/ttyS4",O_RDWR);
    write(fd, data, sizeof(data));
    
    while(1){
        ret = read(fd,rxBuff, sizeof(rxBuff));
        if(ret > 0){
            printf("got data from epdm: %02x \n %s\n",rxBuff,rxBuff);
            write(fd, data, sizeof(data));
        }
        usleep(10000);
    }
}

void cmd_adcs(){
    printf("ADCS Receiving mode in command\n");
}

void cmd_cam(){
    printf("CAM Receiving mode in command\n");
}