#include "DMXI.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <string.h>
#include "rom/ets_sys.h"


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


int dmx_Read(uint8_t *data) // moet nog bijgewerkt worden voor de interface
{
    //

    return uart_read_bytes(DMX_UART_PORT_NUM1, data, DMX_UART_BUF_SIZE, 100 / portTICK_PERIOD_MS);
   

    
}


void dmx_Task(void *arg) // dit is een test functie, deze staat in de task in de main file
{
    uint8_t msg[512];
    while (1) {
        if (dmx_Read(msg)>0)
        {
            dmx_Write(msg);
        }
    }
   vTaskDelete(NULL);
}


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