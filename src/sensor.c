#include "sensor.h"
#include "imu.h"

static axis3i16_t gyro;
static axis3i16_t accel;
static axis3i16_t accelLPF;
// static axis3i16_t accelLPFAligned;
static axis3i32_t accelStoredFilterValues;
static uint8_t imuAccLpfAttFactor;

static bias gyroBias;

static void biasInit(bias* b);

void sensorInit() {
    mpu6050Init();
    biasInit(&gyroBias);
}

static void biasInit(bias* b) {
    b->isBufferFilled = false;
    b->bufHead = b->buffer;
}

void sensorRead(sensor_t* sensor) { 
    readmpu6050(&gyro, &accel);

    imuAddBiasValue(&gyroBias, &gyro);
    if (!gyroBias.isBiasValueFound) {
        imuFindBiasValue(&gyroBias);
    }

    sensor->gyro.x = (gyro.x - gyroBias.bias.x) * MPU6050_DEG_PER_LSB_2000;
    sensor->gyro.y = (gyro.y - gyroBias.bias.y) * MPU6050_DEG_PER_LSB_2000;
    sensor->gyro.z = (gyro.z - gyroBias.bias.z) * MPU6050_DEG_PER_LSB_2000;

    imuAccIIRLPFilter(&accel, &accelLPF, &accelStoredFilterValues, (int32_t) imuAccLpfAttFactor);
    // imuAccAlignToGravity(&accelLPF, &accelLPFAligned);

    // TODO: Calculate bias and reduce it
    sensor->accel.x = (accelLPF.x) * MPU6050_G_PER_LSB_8;
    sensor->accel.y = (accelLPF.y) * MPU6050_G_PER_LSB_8;
    sensor->accel.z = (accelLPF.z) * MPU6050_G_PER_LSB_8;
}
