#include <stdbool.h>

#include "types.h"
#include "filter.h"

#define IMU_NBR_OF_BIAS_SAMPLES  128

#define GYRO_VARIANCE_BASE        4000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)
#define GYRO_NBR_OF_AXES          3

typedef struct {
    axis3i16_t   bias;
    bool       isBiasValueFound;
    bool       isBufferFilled;
    axis3i16_t*  bufHead;
    axis3i16_t   buffer[IMU_NBR_OF_BIAS_SAMPLES];
} bias;

void imuAccIIRLPFilter(axis3i16_t* in, axis3i16_t* out, axis3i32_t* storedValues, int32_t attenuation);
// static void imuAccAlignToGravity(axis3i16_t* in, axis3i16_t* out);
void imuAddBiasValue(bias* b, axis3i16_t* dVal);
bool imuFindBiasValue(bias* b);
void imuCalculateVarianceAndMean(bias* bias, axis3i32_t* varOut, axis3i32_t* meanOut);
