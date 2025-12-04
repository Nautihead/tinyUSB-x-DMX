idf_component_register(SRCS "WIFI_DMXI.c"
                       INCLUDE_DIRS .
                       REQUIRES esp_wifi esp_event nvs_flash esp_system esp_http_server )