#include <math.h>
#include <Wire.h>

#define MPU_ADDR 0b1101000
#define SAMPLES 2000
#define ACCEL_SENSITIVITY 16384.0

struct Gyro {
    long x;
    long y;
    long z;
};

struct Accel {
    float x;
    float y;
    float z;
};

struct Angle {
    float x;
    float y;
    float z;
};

class MPU6050 {
    private:
        Gyro calli = {0, 0, 0};
        Angle angle = {0, 0, 0};
    public:
        void begin();
        void calibrateGyro();
        Gyro readRawGyro();
        Accel readAccel();
        Angle calculateAngle();
};
