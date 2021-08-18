#include "esp_event.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define CONFIG_ESP_WIFI_SSID "esp32-isotp-ble-bridge"
#define CONFIG_ESP_WIFI_PASSWORD "password123"
#define CONFIG_ESP_WIFI_CHANNEL 1
#define CONFIG_ESP_MAX_STA_CONN 4

void wifi_server_setup();
