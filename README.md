## Open Source Mini Quadcoptor Projects
1. [Crazyflie-2-1](https://www.bitcraze.io/products/crazyflie-2-1/)
2. [Paparazzi UAV](https://wiki.paparazziuav.org/wiki/Main_Page)

### Crazyflie-2.X
This [document](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/) explains the source code functionalities of source code. 

Need to find the place where sensor data is being read and processed.

In tag `2017.06`, it happens as shown below:

                        stabalizer.c, stabilizerTask()
                            |
                            |
                            V
                        estimator.c, stateEstimator()
                            |
                            |
        ____________________|____________________
       |                                         |
       |                                         |
       V                                         V
    estimatorKalman.c                       estimatorComplementary.c
                                                 |
                                                 |__ sensors_cf1.c sensorAcquire()
                                                 |__ sensors_cf2.c sensorAcquire()
                                                 |__ sensors_bosch.c sensorAcquire()

2.1 uses `STM32F405 main application MCU (Cortex-M4, 168MHz, 192kb SRAM, 1Mb flash)` microcontroller.

### Crazyflie-1.0
This [document](https://wiki.bitcraze.io/projects:crazyflie:index) contains the information about the hardware and software of the Crazyflie-1.0.

Need to checkout tag `2017.06` in https://github.com/bitcraze/crazyflie-firmware

The MCU is `STM32F103CB @ 72 MHz (128kb flash, 20kb RAM)`

### Code summary of tag `2017.06` in Crazyflie frameware

The main function is `stabilizerTask` in `stabilizer.c`

```c
/** Attitude in euler angle form */
typedef struct attitude_s {
    uint32_t timestamp;  // Timestamp when the data was computed
    float roll;
    float pitch;
    float yaw;
} attitude_t;

/* Orientation as a quaternion */
typedef struct quaternion_s {
    uint32_t timestamp;
    union {
        struct {
            float q0;
            float q1;
            float q2;
            float q3;
        };
        struct {
            float x;
            float y;
            float z;
            float w;
        };
    };
} quaternion_t;

/* x,y,z vector */
struct vec3_s {
    uint32_t timestamp; // Timestamp when the data was computed
    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

typedef struct state_s {
    attitude_t attitude;
    quaternion_t attitudeQuaternion;
    point_t position;
    velocity_t velocity;
    acc_t acc;
} state_t;

typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

typedef struct sensorData_s {
    Axis3f acc;
    Axis3f gyro;
    Axis3f mag;
    baro_t baro;
    zDistance_t zrange;
    point_t position;
#ifdef LOG_SEC_IMU
    Axis3f accSec;
    Axis3f gyroSec;
#endif
} sensorData_t;

typedef struct control_s {
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    float thrust;
} control_t;
```

The `stabilizerTask` function cantains following function calls.

```c
void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
```
This functionality can be directly used to the implementation.
This function calls `estimatorComplementary`, and it does,
1. `sensorsAcquire` updates sensor data (gyro and acc)
2. calculated `roll`, `pitch`, `yaw` using complementory filter.

Even if `control` is passed, it is not used.

```c
void stateController(control_t *control, setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
```

### Paparazzi UAV Lisa/S
Lisa/S framework is the smallest quadcopter framework available in paparazzi UAV. This [document](https://wiki.paparazziuav.org/wiki/Lisa/S/Tutorial/Nano_Quadcopter) provides how paparazzi UAV can be used to install flight controller in Lisa/S nano quadcopter.

It uses `72MHz 32bit ARM Cortex M3 MCU with 16KB RAM and 512KB Flash` microcontroller.

## Notes
1. Both projects use high performance microcontrollers (`ARM Cortex MX`) than ATmega328P.
2. Since Arduino uno is used as the programmer, the avrdude command should be `avrdude -pm328p -c arduino -P /dev/ttyACM0`. the port can be find using `ls -l /dev/tty*`