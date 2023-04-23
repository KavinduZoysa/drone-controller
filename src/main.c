#include <util/delay.h>

#include "sensor.h"
#include "usart.h"

#define REFRESH_RATE 200 // Hz

int main(void) {
    sensorInit();
    // Use for debug purposes
    USARTInit();
    sensor_t sensor;
    
    while (1) {
        sensorRead(&sensor);
        // print sensor
        printf("*****************************************************************\n");
    }

    return 0;    
}