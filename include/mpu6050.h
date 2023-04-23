#include "i2c.h"
#include "types.h"

#define MPU6050_ADDR_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDR_AD0_HIGH    0x69 // address pin high (VCC)

#define PWR_MGMT        0x6B
#define GYRO_CONFIG     0x1B
#define ACC_CONFIG      0x1C

#define GYRO_START      0x43
#define ACCEL_START     0x3B

#define MPU6050_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)
#define MPU6050_G_PER_LSB_8      (float)((2 * 8) / 65536.0)

void mpu6050Init();
void readmpu6050(axis3i16_t* gyro, axis3i16_t* accel);