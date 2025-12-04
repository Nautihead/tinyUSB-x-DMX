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
#include "DMXI.h"
#include "esp_wifi.h"
 #include "WIFI_DMXI.h"








    
void app_main(void)
{

    WIFI_DMXI_Run();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
   

    dmx_Init();
    vTaskDelay(10 / portTICK_PERIOD_MS);

    

    usb_queue = xQueueCreate(5, sizeof(usb_rx_message_t));
    dataCap_queue = xQueueCreate(5, sizeof(data_cap_message_t));
   // wifiGet_queue = xQueueCreate(5, sizeof(wifi_rx_message_t));

   
   // assert(wifiGet_queue);
    assert(usb_queue);
    assert(dataCap_queue);
  
  //  xTaskCreate(tcp_server_task, "server", 8192, NULL, 5, NULL);
     xTaskCreate(dmx_Read, "READ", 8192, NULL, 5, NULL);
    xTaskCreate(tUSB_DMXI_parser, "tUSB_DMXI_parser", 8192, NULL, 5, NULL);
    xTaskCreate(tUSB_DMXI_sender, "tUSB_DMXI_sender", 8192, NULL, 1, NULL);

    // start parser en usb tasks
    
    
   

}