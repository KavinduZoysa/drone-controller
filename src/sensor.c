#include "filter.h"
#include "sensor.h"

static axis3i16_t gyro;
static axis3i16_t accel;
static axis3i16_t accelLPF;
static axis3i16_t accelLPFAligned;
static axis3i32_t accelStoredFilterValues;
static uint8_t imuAccLpfAttFactor;

static bias gyroBias;

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

static void imuAccIIRLPFilter(axis3i16_t* in, axis3i16_t* out, axis3i32_t* storedValues, int32_t attenuation);
static void imuAccAlignToGravity(axis3i16_t* in, axis3i16_t* out);
static void biasInit(bias* b);
static bool imuFindBiasValue(bias* b);
static void imuAddBiasValue(bias* b, axis3i16_t* dVal);
static void imuCalculateVarianceAndMean(bias* bias, axis3i32_t* varOut, axis3i32_t* meanOut);

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

// TODO: Move imu* functions to new file
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

static void imuAddBiasValue(bias* b, axis3i16_t* dVal) {
    b->bufHead->x = dVal->x;
    b->bufHead->y = dVal->y;
    b->bufHead->z = dVal->z;
    b->bufHead++;

    if (b->bufHead >= &b->buffer[IMU_NBR_OF_BIAS_SAMPLES]) {
        b->bufHead = b->buffer;
        b->isBufferFilled = true;
    }
}

static bool imuFindBiasValue(bias* b) {
    bool foundBias = false;

    if (b->isBufferFilled) {
        axis3i32_t variance;
        axis3i32_t mean;

        imuCalculateVarianceAndMean(b, &variance, &mean);

        if (variance.x < GYRO_VARIANCE_THRESHOLD_X && variance.y < GYRO_VARIANCE_THRESHOLD_Y && variance.z < GYRO_VARIANCE_THRESHOLD_Z) {
            // In actual code this block executes at every 1ms
            // TODO: Consider about that
            b->bias.x = mean.x;
            b->bias.y = mean.y;
            b->bias.z = mean.z;
            foundBias = true;
            b->isBiasValueFound = true;
        }
    }
    return foundBias;
}

static void imuCalculateVarianceAndMean(bias* bias, axis3i32_t* varOut, axis3i32_t* meanOut) {
    uint32_t i;
    int32_t sum[GYRO_NBR_OF_AXES] = {0};
    int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
    }

    varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
    varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
    varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

    meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
    meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
    meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;
}
