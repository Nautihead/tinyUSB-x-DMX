#ifndef WIFI_DMXI_H_
#define WIFI_DMXI_H_
#include "esp_err.h"
#include "string.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"

#define WIFI_DMXI_BYTES 515

extern QueueHandle_t wifiGet_queue;

typedef struct {
    uint8_t buf[WIFI_DMXI_BYTES];     // Data buffer
    size_t buf_len;                  // Number of bytes received
                       
} wifi_rx_message_t; 

static esp_err_t root_get_handler(httpd_req_t *req);
static httpd_handle_t start_webserver(void);

// void tcp_server_init(void);
void tcp_server_task(void *pvParameters);
void wifi_init_softap(void);
void WIFI_DMXI_Run(void);
void wifi_init_softap(void);


#endif