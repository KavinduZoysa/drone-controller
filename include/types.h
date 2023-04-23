/* State of the quadcoptor at a given time*/
typedef struct state_c {
    float roll;
    float piitch;
    float yaw;
} state_t;

typedef struct axis3_s {
    float x;
    float y;
    float z;
} axis3_t;

typedef struct axis3i16_s {
    int16_t x;
    int16_t y;
    int16_t z;
} axis3i16_t;

typedef struct axis3i32_s {
    int32_t x;
    int32_t y;
    int32_t z;
} axis3i32_t;

/* Raw sensor output */
typedef struct sensor_s {
    axis3_t gyro;
    axis3_t accel;
} sensor_t;
