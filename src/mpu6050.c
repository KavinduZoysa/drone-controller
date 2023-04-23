#include "mpu6050.h"
#include "usart.h"

void mpu6050Init() {
    tw_init(TW_FREQ_400K, true);

    uint8_t data[2] = {PWR_MGMT, 0};
	tw_master_transmit(MPU6050_ADDR_AD0_LOW, data, sizeof(data), false);

    data[0] = GYRO_CONFIG;
    data[1] = 24;
	tw_master_transmit(MPU6050_ADDR_AD0_LOW, data, sizeof(data), false);

    data[0] = ACC_CONFIG;
    data[1] = 16;
	tw_master_transmit(MPU6050_ADDR_AD0_LOW, data, sizeof(data), false);
}

void readmpu6050(axis3i16_t* gyro, axis3i16_t* accel) {
	uint8_t data[14];
	
	data[0] = ACCEL_START;
    tw_master_transmit(MPU6050_ADDR_AD0_LOW, data, 1, true);
	
    tw_master_receive(MPU6050_ADDR_AD0_LOW, data, sizeof(data));
	
	accel->x = ((((int16_t) data[0]) << 8) | data[1]);
	accel->y = ((((int16_t) data[2]) << 8) | data[3]);
	accel->z = ((((int16_t) data[4]) << 8) | data[5]);
	gyro->x = ((((int16_t) data[8]) << 8) | data[9]);
	gyro->y = ((((int16_t) data[10]) << 8) | data[11]);
	gyro->z = ((((int16_t) data[12]) << 8) | data[13]);
}
