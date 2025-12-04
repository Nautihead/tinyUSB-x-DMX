#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "WIFI_DMXI.h"

QueueHandle_t wifiGet_queue = NULL;
static const char *TAG = "AP_WEBSERVER";

void tcp_server_task(void *pvParameters)
{
    int listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);          //fd van socket def. (ipv4, tcp, ip protocol)

    server_addr.sin_family = AF_INET;                              //ipv4
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);              //alle beschikbare interfaces
    server_addr.sin_port = htons(5000);                          //poort 5000

    bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)); // binden van socket aan poort en adres

    listen(listen_sock, 1);                                                //luisteren naar inkomende verbindingen op poort 5000, max 1 in wachtrij

    while (1) {
        client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &addr_len);
        printf("Client verbonden\n");
        char buffer[128];
        int len = recv(client_sock, buffer, sizeof(buffer)-1, 0);
        if (len > 0) {
               
                ESP_LOG_BUFFER_HEXDUMP(TAG, buffer, sizeof(buffer)-1 , ESP_LOG_INFO);
            buffer[len] = 0;                                        //                                <= hier buffer met ipc 
            send(client_sock, "OK", 2, 0);
        }

        close(client_sock);
    }
}




// ----------- HTTP HANDLER (serve simple page) ----------
static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char html[] =
        "<!DOCTYPE html>"
        "<html>"
        "<head><title>l</title></head>"
        "<body>"
        "<h1> DENISA IS POOPOOO PIPI</h1>"
        "<p> NEVER gonna gu<p>"
        "<p>fuck you</p>"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));

    // hier DMXI_SEND functie default kleuren sturen

    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_root = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &uri_root);
    }

    return server;
}

// ----------- WIFI ACCESS POINT SETUP ----------
void wifi_init_softap(void)
{
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_wifi_set_mode(WIFI_MODE_AP);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "DENISA IS A BEAN",
            .ssid_len = strlen("DENISA IS A BEAN"),
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen("12345678") == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;  // open AP
    }

    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "Access Point gestart! SSID: ESP32_AP  PASS: 12345678");
}

// ----------- MAIN ----------
void WIFI_DMXI_Run(void)
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_softap();
    start_webserver();
}