#include "driver/uart.h"
#include "stdio.h"
#include <stdint.h>





#pragma once

#define readBuf 512
#define writeBuf 512

#define DMX_UART_BUF_SIZE      512
#define DMX_UART_BAUD_RATE     250000


#define DMX_UART_PORT_NUM      UART_NUM_0
#define DMX_UART_PORT_NUM1     UART_NUM_2
#define DMX_UART_TX_PIN        GPIO_NUM_4
#define DMX_UART_RX_PIN        GPIO_NUM_18


#define BYTESTOBESENT 512

#define RX_BUF_SIZE 512
#define USB_QUEUE_DEPTH 8
#define DATA_QUEUE_DEPTH 4


void dmx_Init(void);
void dmx_Write(uint8_t* datas);
int dmx_Read(uint8_t *data);
void dmx_Task(void *arg);
