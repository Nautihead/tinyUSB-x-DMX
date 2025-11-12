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






    
void app_main(void)
{
   
    usb_init();
    vTaskDelay(1000 / portTICK_PERIOD_MS); // wacht even tot USB is opgestart
    dmx_Init();

    usb_queue = xQueueCreate(5, sizeof(usb_rx_message_t));
    dataCap_queue = xQueueCreate(5, sizeof(data_cap_message_t));

    assert(usb_queue);
    assert(dataCap_queue);
  

    xTaskCreate(tUSB_DMXI_parser, "tUSB_DMXI_parser", 8192, NULL, 5, NULL);
    xTaskCreate(tUSB_DMXI_sender, "tUSB_DMXI_sender", 8192, NULL, 1, NULL);

    // start parser en usb tasks
    
    
   

}