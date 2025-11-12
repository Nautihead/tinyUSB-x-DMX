

#ifndef TUSB_DMXI_H
#define TUSB_DMXI_H

#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "DMXI.h"

#define USB_QUEUE_DEPTH 5       //hoeveel verschillende messages mogen er in de queue staan
#define DATA_QUEUE_DEPTH 5


#define CONFIG_TINYUSB_CDC_RX_BUFSIZE 512



typedef struct {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];     // Data buffer
    size_t buf_len;                                     // Number of bytes received
    uint8_t itf;                                        // Index of CDC device interface
} usb_rx_message_t;                                     // USB RX message structure


typedef struct {
    uint8_t dBuf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];     // Data buffer
    size_t dBuf_len;                                     // Number of bytes received
    uint8_t dItf;                                        // Index of CDC device interface
} data_cap_message_t;                                    // DMX TX message structure


extern QueueHandle_t usb_queue;
extern QueueHandle_t dataCap_queue;

static const char *TAG = "example";




void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event);
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event);

void usb_init(void);
void tUSB_DMXI_sender(void *arg);
void tUSB_DMXI_parser(void *arg);

#endif