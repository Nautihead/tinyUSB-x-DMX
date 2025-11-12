/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */



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
#include "tUSB_DMXI.h"


QueueHandle_t usb_queue = NULL;
QueueHandle_t dataCap_queue = NULL;


uint8_t STARTBYTES[3] = {0x00, 0xFF, 0x00} ;// DMX start bytes
uint8_t STOPBYTES[3] = {0xFF, 0x00, 0x00}; // DMX stop bytes

static uint8_t rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];




/**
 * @brief CDC device RX callback
 *
 * CDC device signals, that new data were received
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */


void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {

        usb_rx_message_t tx_msg = {
            .buf_len = rx_size,
            .itf = itf,
        };

        memcpy(tx_msg.buf, rx_buf, rx_size);
        xQueueSend(usb_queue, &tx_msg, 0);
    } else {
        ESP_LOGE(TAG, "Read Error");
    }
}



/**
 * @brief CDC device line change callback
 *
 * CDC device signals, that the DTR, RTS states changed
 *
 * @param[in] itf   CDC device index
 * @param[in] event CDC event type
 */
void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

void usb_init(void)
{                                               //usb_init is afkomstig van de tusb
    
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = NULL,
        .hs_configuration_descriptor = NULL,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = NULL,
#endif // TUD_OPT_HIGH_SPEED
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
   
   


    ESP_LOGI(TAG, "USB initialization DONE");
}


void USB_Read(void *arg)
{
    assert(usb_queue);
    usb_rx_message_t msg;

    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = NULL,
        .hs_configuration_descriptor = NULL,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = NULL,
#endif // TUD_OPT_HIGH_SPEED
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
   
   


    ESP_LOGI(TAG, "USB initialization DONE");
   
}



void tUSB_DMXI_parser(void *arg) {
    usb_rx_message_t msg_in;
    data_cap_message_t msg_out;
    ESP_LOGI(TAG, "Entered parser task");
  
    while (1) {
        if (xQueueReceive(usb_queue, &msg_in, portMAX_DELAY)) {

             if (msg_in.buf_len) {
            ESP_LOGI(TAG, "data ontvangen");
            ESP_LOG_BUFFER_HEXDUMP(TAG, msg_in.buf, msg_in.buf_len, ESP_LOG_INFO);

            bool start_found = true;
          

            for (int i = 0; i < 3; i++) {
                if (msg_in.buf[i] != STARTBYTES[i]) {
                    start_found = false;

                    break;
                }
            }
            for (int i = 3; i <= 514; i++) {
                msg_out.dBuf[i] = msg_in.buf[i];
                ESP_LOGI(TAG, "received data byte %d", i);
            }

            

            if (start_found) {
                ESP_LOGI(TAG, "DMX start sequence detected");
                msg_out.dBuf_len = msg_in.buf_len;
                msg_out.dItf = msg_in.itf;
                xQueueSend(dataCap_queue, &msg_out, 0);
                
            }
        }
            
                
        }
        ESP_LOGI(TAG, "parser running");
        vTaskDelay(500);
    }
}



void tUSB_DMXI_sender(void *arg) {

    data_cap_message_t msg_in;
     
    ESP_LOGI(TAG, "Entered sender task");

    while(1)
    { if (xQueueReceive(dataCap_queue, &msg_in, portMAX_DELAY)) {

            // Stuur data via DMX
            dmx_Write(msg_in.dBuf);

            ESP_LOGI(TAG, "Data sent via DMX, length: %d", msg_in.dBuf_len);

            for (int i = 0; i < msg_in.dBuf_len; i++) {                 //  Debugging
                ESP_LOGI(TAG, "Sent byte %d: %d", i, msg_in.dBuf[i]);
            }
        }

    
    ESP_LOGI(TAG, "sender running");
    vTaskDelay(500);}

      

}
