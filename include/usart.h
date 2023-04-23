#include <avr/io.h>
#include <stdio.h>

// #define F_CPU 16000000UL
#define BAUD 9600
#define UBRR_VAL (16000000UL/16/BAUD-1)

void USARTInit(void);
int USARTTransmitChar(char data, FILE *stream);
void printFloat(float f);