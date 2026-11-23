#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "link_driver.h"
#include "main.h"
#include "stm32mp157dxx_cm4.h"
#include "stm32mp1xx_hal_uart.h"

typedef struct {
    USART_TypeDef*  instance;
    uint32_t        baud;
    uint32_t        word_length;
    uint32_t        parity;
    uint32_t        stop_bits;
} uart_cfg_t;

typedef struct {
    link_driver_t       base; // Must be first. This struct is a "child class" of link_driver_t. This being first allows us to cast a pointer to the "parent" struct.
    uart_cfg_t          cfg;
    UART_HandleTypeDef  huart;
} uart_driver_t;

// Binds driver. Does not touch hardware until d->base->ops->init().
void uart_driver_create(uart_driver_t* ud, const uart_cfg_t* cfg);

// Call from generated UART IRQ handler.
void uart_driver_irq(USART_TypeDef* instance);

#endif /* UART_DRIVER_H */