
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

#define REG_LOW_MASK 0xFF00
#define REG_HIGH_MASK 0x00FF
#define MPU6050_FS_SEL 32.8f
#define MPU6050_AFS_SEL 4096.0f

struct sensor_accel {
    float x;
    float y;
    float z;
    float temperature;
    float timestamp;
};

struct sensor_gyro {
    float x;
    float y;
    float z;
};

struct mpu6500_imu_msg {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};



/****************************************************************************
 * Public Functions
 ****************************************************************************/

void read_mpu6500(int fd, struct sensor_accel *acc_data, struct sensor_gyro *gyro_data, struct mpu6500_imu_msg *raw_imu);

int main(int argc, FAR char *argv[])
{
    struct sensor_accel imu_acc_data;
    struct sensor_gyro imu_gyro_data;
    struct mpu6500_imu_msg raw_imu;

    int fd = open("/dev/mpu6500", O_RDONLY);
    if (fd < 0)
    {
        printf("Failed to open mpu6500\n");
        return -1;
    }

    while(1){
      read_mpu6500(fd, &imu_acc_data, &imu_gyro_data, &raw_imu);

    printf("Timestamp: %f  Temperature: %f\n"
           "Accelerometer X: %f | Y: %f | Z: %f\n"
           "Gyroscope X: %f | Y: %f | Z: %f\n",
           imu_acc_data.timestamp, imu_acc_data.temperature,
           imu_acc_data.x, imu_acc_data.y, imu_acc_data.z,
           imu_gyro_data.x, imu_gyro_data.y, imu_gyro_data.z);
    sleep(2);
    }
    close(fd);
    return 0;
}

void read_mpu6500(int fd, struct sensor_accel *acc_data, struct sensor_gyro *gyro_data, struct mpu6500_imu_msg *raw_imu)
{
    int16_t raw_data[7];
    memset(raw_imu, 0, sizeof(struct mpu6500_imu_msg));
    int ret = read(fd, raw_data, sizeof(raw_data));
    if (ret <= 0)
    {
        printf("Failed to read accelerometer data\n");
    }
    else
    {
        raw_imu->acc_x = ((raw_data[0] & REG_HIGH_MASK) << 8) + ((raw_data[0] & REG_LOW_MASK) >> 8);
        raw_imu->acc_y = ((raw_data[1] & REG_HIGH_MASK) << 8) + ((raw_data[1] & REG_LOW_MASK) >> 8);
        raw_imu->acc_z = ((raw_data[2] & REG_HIGH_MASK) << 8) + ((raw_data[2] & REG_LOW_MASK) >> 8);
        raw_imu->gyro_x = ((raw_data[4] & REG_HIGH_MASK) << 8) + ((raw_data[4] & REG_LOW_MASK) >> 8);
        raw_imu->gyro_y = ((raw_data[5] & REG_HIGH_MASK) << 8) + ((raw_data[5] & REG_LOW_MASK) >> 8);
        raw_imu->gyro_z = ((raw_data[6] & REG_HIGH_MASK) << 8) + ((raw_data[6] & REG_LOW_MASK) >> 8);
    }

    acc_data->x = raw_imu->acc_x * 9.8 / MPU6050_AFS_SEL;
    acc_data->y = raw_imu->acc_y * 9.8 / MPU6050_AFS_SEL;
    acc_data->z = raw_imu->acc_z * 9.8 / MPU6050_AFS_SEL;

    gyro_data->x = raw_imu->gyro_x / MPU6050_FS_SEL;
    gyro_data->y = raw_imu->gyro_y / MPU6050_FS_SEL;
    gyro_data->z = raw_imu->gyro_z / MPU6050_FS_SEL;
}
