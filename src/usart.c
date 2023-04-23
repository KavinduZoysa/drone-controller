#include "usart.h"

static FILE customStdout = FDEV_SETUP_STREAM(USARTTransmitChar, NULL, _FDEV_SETUP_WRITE);

void USARTInit(void) {
    // set baud rate
    UBRR0H = (UBRR_VAL >> 8);
    UBRR0L = UBRR_VAL;

    // enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // set frame format: 8 data bits, no parity, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    stdout = &customStdout;
}

int USARTTransmitChar(char data, FILE *stream) {
    // wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));

    // put data into buffer, sends the data
    UDR0 = data;

    return 0;
}
