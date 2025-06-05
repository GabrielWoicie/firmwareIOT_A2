#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>


void uart_init(void);
void uart_write(const uint8_t *data, size_t size);
int uart_read(uint8_t *buffer, size_t size);

#endif // UART_H
