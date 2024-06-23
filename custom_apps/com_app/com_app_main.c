
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define COM_UART "/dev/ttyS1"
enum MSNS {
COM,
EPDM,
ADCS,
CAM
} MSN_CHOICE;



void send_data_uart(char *dev_path, int *data){
    int fd,  elapsed =0, required =1000000;
    char *data1;
    int count =0;
    fd = open(dev_path, O_RDWR);
    if(fd < 0){
        //syslog(LOG_ERROR, "Unable to open the given uart dev path : %s ret : %d",dev_path, fd);
    }
    else{
        fd = write(fd, data, sizeof(data));
        if(fd > 0){
            while(elapsed < required){
                receive_data_uart(dev_path, data1);
                if(data[0] == data1[0] && data[1] == data1[1] && data[2] == data1[2]  && data[3] == data1[3] && data[4] == data1[4] && data[5] == data1[5])
                {
                  printf("\n***Acknowledgement received for %s*******\n",dev_path);
                  break;   
                }
                else

                usleep(1000);
                required += 1000;
            }
            //syslog(LOG_//syslog,"Data written successfully");
        }
    }
    close(fd);
}   

void receive_data_uart(char *dev_path, uint8_t *data){
    int fd;
    fd = open(dev_path, O_RDWR);
    if(fd < 0){
        //syslog(LOG_ERROR, "Unable to open the given uart dev path : %s ret : %d",dev_path, fd);
    }
    else{

            fd = read(fd, data, sizeof(data));
            if(fd > 0){
                return;
            }
                //syslog(LOG_//syslog,"Data written successfully");

    }
    close(fd);
}   

void send_data_uart_char(char *dev_path, uint8_t *data){
    int fd;
    fd = open(dev_path, O_RDWR);
    if(fd < 0){
        //syslog(LOG_ERROR, "Unable to open the given uart dev path : %s ret : %d",dev_path, fd);
    }
    else{
        fd = write(fd, data, sizeof(data));
        if(fd > 0){
            //syslog(LOG_//syslog,"Data written successfully");
        }
    }
    close(fd);
}   

/*
Handshake command is to be provided 6bytes with a header 0x53 and footer 0x7e
*/
void handshake(int choice, uint8_t ack[7]){
    switch(choice){
        case COM: printf("COM uart");
                send_data_uart(COM_UART, ack);
                break;
        case EPDM: printf("COM epdm");
                send_data_uart("/dev/ttyS3", ack);
                break;
        case ADCS:printf("ADCS uart"); 
                send_data_uart("/dev/ttyS3", ack); 
                break;
        case CAM: printf("CAM uart");
                send_data_uart("/dev/ttyS3", ack);
                break;
        default: send_data_uart_char("/dev/ttyS4", "\nCaught on default\n");
                break;
    };
    
}

// void send_data

/*
Beacon type 1 is of 84 including header+packet_type+ data + information/data + footer
*/
void beacon_type_1(){
uint8_t data[85];//={0x53,0x01,0x54};
data[0] = 0x53;
data[1]= 0x01;
data[2] =0x54;
for(int i=3;i<83;i++){
    data[i]= i;
}
data[83] = 0x73;
data[84] =  "\0";
write(COM_UART,data, sizeoof(data));
}

/*
Beacon type 2 is of 84 including header+packet_type+ data + information/data + footer
*/
void beacon_type_2(){
uint8_t data[85];//={0x53,0x01,0x54};
data[0] = 0x53;
data[1]= 0x02;
data[2] =0x54;
for(int i=3;i<83;i++){
    data[i]= i;
}
data[83] = 0x73;
data[84] =  "\0";
write(COM_UART,data, sizeoof(data));
}


/*
Receive telecommand from GS for 90 seconds
*/
void receive_telecommand_rx(){
    uint8_t data[255];
    receive_data_uart(COM_UART,data);
}
/*
COM will be in digipeater mode till a digipeating message is received and digipeated
*/
void digipeater_mode(){
    uint8_t *data;
    receive_data_uart(COM_UART,data);
}

/****************************************************************************
 * custom_hello_thread
 ****************************************************************************/
int main(int argc, FAR char *argv[])
{   uint8_t ack[7]={0x53, 0x01, 0x02, 0x03, 0x04, 0x7e};
    if (argc>1){
        if(!strcmp(argv[1],"com"))
        {
            handshake(COM,ack);
        }
        else if(!strcmp(argv[1],"epdm"))
        {
            handshake(EPDM,ack);
        }
        else if(!strcmp(argv[1],"cam"))
        {
            handshake(CAM,ack);
        }
        else if(!strcmp(argv[1],"adcs"))
        {
            handshake(ADCS,ack);
        }
        else
        {
            handshake(77,ack);
        }

    }
    else {
        handshake(COM, ack);

        handshake(EPDM, ack);

        handshake(CAM, ack);

        handshake(ADCS, ack);

        handshake(77,ack);
    }

    return 0;
}