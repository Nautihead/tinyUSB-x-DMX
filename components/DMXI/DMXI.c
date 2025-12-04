#include "DMXI.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <string.h>
#include "rom/ets_sys.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"

static const char *TAG = "DMXI";
#define BUF_SIZE (1024)
#define STACK_SIZE (4096)

QueueHandle_t usb_queue = NULL;
QueueHandle_t dataCap_queue = NULL;

uint8_t STARTBYTES[3] = {0x00, 0xFF, 0x00} ;// DMX start bytes
uint8_t STOPBYTES[3] = {0xFF, 0x00, 0x00}; // DMX stop bytes


void dmx_Init(void)
{
    // Configure UART parameters
         uart_config_t conf = {  
        .baud_rate = DMX_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_2,   // vaak nodig voor DMX
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,

        
    
    };

    
    
    gpio_reset_pin(DMX_UART_TX_PIN);
    gpio_set_direction(DMX_UART_TX_PIN, GPIO_MODE_OUTPUT);
    
  

    // Install UART driver
    uart_driver_install(DMX_UART_PORT_NUM1, DMX_UART_BUF_SIZE * 2, 513, 0, NULL, 0);
    uart_param_config(DMX_UART_PORT_NUM1, &conf);
    uart_set_pin(DMX_UART_PORT_NUM1, DMX_UART_TX_PIN, DMX_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
   
}

void dmx_Write(uint8_t* data)

{

   

    uart_set_line_inverse(UART_NUM_1, UART_SIGNAL_TXD_INV); // optioneel, voor logische inversie
    ets_delay_us(100); // low-level timing voor DMX-break
    uart_set_line_inverse(UART_NUM_1, 0);
    ets_delay_us(10);





    // Send DMX data
    uart_write_bytes(DMX_UART_PORT_NUM1, (const char *)data, 512);
}


void dmx_Read(void *arg) // moet nog bijgewerkt worden voor de interface
{
     usb_rx_message_t sendData;

    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE("usb_serial_jtag echo", "no memory for data");
        return;
    }

    while (1) {

        int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);

        // Write data back to the USB SERIAL JTAG

        if (len) {
            usb_serial_jtag_write_bytes((const char *) data, len, 20 / portTICK_PERIOD_MS);
            sendData.buf_len = len;
            memcpy(sendData.buf, data, len);

            xQueueSend(usb_queue, &sendData, 0);
            ESP_LOGI("usb_serial_jtag echo", "Read %d bytes", len);
        }
    }
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
            printf("Data : %d", msg_in.buf);

            bool start_found = true;
          

            for (int i = 0; i < 3; i++) {
                if (msg_in.buf[i] != STARTBYTES[i]) {
                    start_found = false;

                    break;
                }
            }
            for (int i = 3; i <= 514; i++) {
                msg_out.dBuf[i-2] = msg_in.buf[i];               // omdat de eerste bytes start bytes zijn
                ESP_LOGI(TAG, "received data byte %d", i);
            }

            

            if (start_found) {
                ESP_LOGI(TAG, "DMX start sequence detected");
                msg_out.dBuf_len = msg_in.buf_len;
               
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
    vTaskDelay(500);}}




/*
void dmx_Task(void *arg) // dit is een test functie, deze staat in de task in de main file
{
    uint8_t msg[512];
    while (1) {
        if (dmx_Read()>0)
        {
            dmx_Write(msg);
        }
    }
   vTaskDelete(NULL);
}
*/

void dmx_Test(void *arg) // dit is een test functie, deze staat in de task in de main file
{
    uint8_t msg[512];
    memset(msg,0,512);
    msg[3]=255;
    msg[4]=0;
    msg[5]= 0;
    msg[6]=0;
    msg[7]= 250;
    msg[8]=0;
    
   
   

    dmx_Init();

    while (1) {
        msg[3]= 255;
        msg[4]= 0;
        msg[5]= 0;

        dmx_Write(msg);

        vTaskDelay(100);

          msg[3]= 0;
           msg[4]= 255;
           msg[5]= 0;
        
        dmx_Write(msg);

        vTaskDelay(100);

        
          msg[3]= 0;
           msg[4]= 0;
           msg[5]= 255;
        
        dmx_Write(msg);

        vTaskDelay(100);


      
    }
  
}