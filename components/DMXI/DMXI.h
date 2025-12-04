#include "driver/uart.h"
#include "stdio.h"
#include <stdint.h>





#pragma once

#define readBuf 512
#define writeBuf 512

#define DMX_UART_BUF_SIZE      512
#define DMX_UART_BAUD_RATE     250000


#define DMX_UART_PORT_NUM      UART_NUM_0
#define DMX_UART_PORT_NUM1     UART_NUM_1
#define DMX_UART_TX_PIN        GPIO_NUM_18 // GPIO pin voor DMX TX 5 
#define DMX_UART_RX_PIN        GPIO_NUM_5  // 18


#define BYTESTOBESENT 512

#define RX_BUF_SIZE 512
#define USB_QUEUE_DEPTH 8
#define DATA_QUEUE_DEPTH 4

#define Jtag_RX_BUFSIZE 512

typedef struct {
    uint8_t buf[Jtag_RX_BUFSIZE + 1];     // Data buffer
    size_t buf_len;                                     // Number of bytes received
                                           // Index of CDC device interface
} usb_rx_message_t;                                     // USB RX message structure


typedef struct {
    uint8_t dBuf[Jtag_RX_BUFSIZE + 1];     // Data buffer
    size_t dBuf_len;                                     // Number of bytes received
                                      // Index of CDC device interface
} data_cap_message_t;                                    // DMX TX message structure


extern QueueHandle_t usb_queue;
extern QueueHandle_t dataCap_queue;


void dmx_Init(void);
void dmx_Write(uint8_t* data);
void dmx_Read(void *arg);
void dmx_Task(void *arg);

void tUSB_DMXI_parser(void *arg);
void tUSB_DMXI_sender(void *arg);
