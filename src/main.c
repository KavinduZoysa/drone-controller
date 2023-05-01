#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "sensor.h"
#include "usart.h"

#define REFRESH_RATE_HZ         200

#define PRESCALER               1024
// TODO: Replace the const with equation
#define REFRESH_RATE_IN_TICKS   65458 // (uint16_t) (65536 - ((F_CPU / (REFRESH_RATE_HZ * PRESCALER)))) // ((int) (1/REFRESH_RATE_HZ)/(1/(F_CPU/PRESCALER)))

static void interruptInit();

volatile uint8_t loopFlag = 0;

ISR(TIMER1_OVF_vect) {
    TCNT1 = REFRESH_RATE_IN_TICKS;
    loopFlag = 1;
}

int main(void) {
    sensorInit();
    // Use for debug purposes
    USARTInit();
    interruptInit();

    sensor_t sensor;
    uint8_t i = 0;
    while (1) {
        if (loopFlag == 1) {
            // TODO: Move this to a high frequency loop
            sensorRead(&sensor); // In crazyflie sensor read happens at full rate. May be we have to introduce another timer to achieve this.

            // complementory filter
            if (i == 200) {
                // print
                printf("gyro.x : %f\n", sensor.gyro.x);
                printf("gyro.y : %f\n", sensor.gyro.y);
                printf("gyro.z : %f\n", sensor.gyro.z);
                printf("accel.x : %f\n", sensor.accel.x);
                printf("accel.y : %f\n", sensor.accel.y);
                printf("accel.z : %f\n", sensor.accel.z);
                i = 0;
            }
            i = i + 1;
            loopFlag = 0;
        }
    }

    return 0;    
}

void interruptInit() {
    sei();

    TCCR1B = 0x05;
    TCNT1 = REFRESH_RATE_IN_TICKS;  

    TIMSK1 |= (1 << TOIE1);
}
