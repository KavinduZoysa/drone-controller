#include "filter.h"
#include "sensor.h"
#include <stdio.h>

static axis3i16_t gyro;
static axis3i16_t accel;
static axis3i16_t accelLPF;
static axis3i16_t accelLPFAligned;
static axis3i32_t accelStoredFilterValues;
static uint8_t imuAccLpfAttFactor;

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

static void imuAccIIRLPFilter(axis3i16_t* in, axis3i16_t* out, axis3i32_t* storedValues, int32_t attenuation);
static void imuAccAlignToGravity(axis3i16_t* in, axis3i16_t* out);

void sensorInit() {
    mpu6050Init();
}

void sensorRead(sensor_t* sensor) { 
    readmpu6050(&gyro, &accel);

    sensor->gyro.x = gyro.x * MPU6050_DEG_PER_LSB_2000;
    sensor->gyro.y = gyro.y * MPU6050_DEG_PER_LSB_2000;
    sensor->gyro.z = gyro.z * MPU6050_DEG_PER_LSB_2000;

    imuAccIIRLPFilter(&accel, &accelLPF, &accelStoredFilterValues, (int32_t)imuAccLpfAttFactor);
    // imuAccAlignToGravity(&accelLPF, &accelLPFAligned);

    sensor->gyro.x = (accelLPFAligned.x) * MPU6050_G_PER_LSB_8;
    sensor->gyro.y = (accelLPFAligned.y) * MPU6050_G_PER_LSB_8;
    sensor->gyro.z = (accelLPFAligned.z) * MPU6050_G_PER_LSB_8;
}

static void imuAccIIRLPFilter(axis3i16_t* in, axis3i16_t* out, axis3i32_t* storedValues, int32_t attenuation) {
    out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
    out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
    out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}

static void imuAccAlignToGravity(axis3i16_t* in, axis3i16_t* out) {
    axis3i16_t rx;
    axis3i16_t ry;

    // Rotate around x-axis
    rx.x = in->x;
    rx.y = in->y * cosRoll - in->z * sinRoll;
    rx.z = in->y * sinRoll + in->z * cosRoll;

    // Rotate around y-axis
    ry.x = rx.x * cosPitch - rx.z * sinPitch;
    ry.y = rx.y;
    ry.z = -rx.x * sinPitch + rx.z * cosPitch;

    out->x = ry.x;
    out->y = ry.y;
    out->z = ry.z;
    // TODO: The code for initializing cosPitch, cosRoll etc, should be imported
}