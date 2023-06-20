#include <stdint.h>

#ifndef TYPES_H
#define TYPES_H

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

typedef struct attitude_s {
    float roll;
    float pitch;
    float yaw;
} attitude_t;

typedef struct state_s {
    attitude_t attitude;
} state_t;

typedef struct control_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    float thrust;
} control_t;

typedef enum mode_e {
  modeDisable = 0,
  modeAbs,
  modeVelocity
} stab_mode_t;

typedef struct setpoint_s {
    attitude_t attitude;
    attitude_t attitudeRate;

    struct {
        stab_mode_t x;
        stab_mode_t y;
        stab_mode_t z;
        stab_mode_t roll;
        stab_mode_t pitch;
        stab_mode_t yaw;
    } mode;
} setpoint_t;
#endif
