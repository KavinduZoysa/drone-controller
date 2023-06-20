#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "sensor.h"
#include "usart.h"
#include "sensfusion6.h"
#include "controllerPID.h"

// TODO: Check F_CPU is set to 16MHz

#define REFRESH_RATE_HZ         200

#define PRESCALER               1024
// TODO: Replace the const with equation
#define REFRESH_RATE_IN_TICKS   65458 // (uint16_t) (65536 - ((F_CPU / (REFRESH_RATE_HZ * PRESCALER)))) // ((int) (1/REFRESH_RATE_HZ)/(1/(F_CPU/PRESCALER))) // CPU Frequency is 16MHz
#define ATTITUDE_UPDATE_DT    (float)(1.0f / REFRESH_RATE_HZ)

static void interruptInit();
static void PWMInit();

volatile uint8_t loopFlag = 0;

ISR(TIMER1_OVF_vect) {
    TCNT1 = REFRESH_RATE_IN_TICKS;
    loopFlag = 1;
}

int main(void) {
    sensorInit();
    stateControllerInit(ATTITUDE_UPDATE_DT);
    // Use for debug purposes
    USARTInit();
    interruptInit();
    PWMInit();

    sensor_t sensor;
    state_t state;
    setpoint_t setpoint = {.attitude = {.roll = 0.0, .pitch = 0.0, .yaw = 0.0}, .attitudeRate = {.roll = 0.0, .pitch = 0.0, .yaw = 0.0}};
    control_t control;
    uint8_t i = 0;
    while (1) {
        if (loopFlag == 1) {
            // TODO: Move this to a high frequency loop
            sensorRead(&sensor); // In crazyflie, sensor read happens at 1000Hz. May be we have to introduce another timer to achieve this.

            // complementory filter
            // In crazyflie, complementory filter update happens at 250Hz
            sensfusion6UpdateQ(sensor.gyro.x, sensor.gyro.y, sensor.gyro.z, sensor.accel.x, sensor.accel.y, sensor.accel.z, 1.0/REFRESH_RATE_HZ);
            sensfusion6GetEulerRPY(&state.attitude.roll, &state.attitude.pitch, &state.attitude.yaw);

            stateController(&control, &setpoint, &sensor, &state);

            if (i == 200) {
                // print
                printf("gyro.x : %f\n", sensor.gyro.x);
                printf("gyro.y : %f\n", sensor.gyro.y);
                printf("gyro.z : %f\n", sensor.gyro.z);
                printf("accel.x : %f\n", sensor.accel.x);
                printf("accel.y : %f\n", sensor.accel.y);
                printf("accel.z : %f\n", sensor.accel.z);
                printf("*****************************************\n");
                printf("roll  : %f\n", state.attitude.roll);
                printf("pitch : %f\n", state.attitude.pitch);
                printf("yaw   : %f\n", state.attitude.yaw);
                i = 0;
            }
            i = i + 1;
            loopFlag = 0;
        }
    }

    // while (1)
    // {
    //     for (int i = 0; i < 155; i++) {
    //         OCR0A = i;
    //         _delay_ms(10);
    //         printf("yaw   : %d\n", i);
    //     }
    // }

    return 0;    
}

void interruptInit() {
    sei();

    TCCR1B = 0x05;
    TCNT1 = REFRESH_RATE_IN_TICKS;  

    TIMSK1 |= (1 << TOIE1);
}

void PWMInit() {
    DDRD |= (1 << PD6);
    TCCR0A |= (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
    TCCR0B |= (1 << CS00);
}
