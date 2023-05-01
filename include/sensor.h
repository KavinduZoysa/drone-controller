#include "mpu6050.h"

#define IMU_NBR_OF_BIAS_SAMPLES  128

#define GYRO_VARIANCE_BASE        4000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)
#define GYRO_NBR_OF_AXES            3

typedef struct {
  axis3i16_t   bias;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  axis3i16_t*  bufHead;
  axis3i16_t   buffer[IMU_NBR_OF_BIAS_SAMPLES];
} bias;

void sensorInit();
void sensorRead(sensor_t* sensor);
