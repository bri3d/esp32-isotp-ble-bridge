#include "esp_event.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define CONFIG_ESP_WIFI_AP_SSID "esp32-isotp-ble-bridge"
#define CONFIG_ESP_WIFI_AP_PASSWORD "password123"
#define CONFIG_ESP_WIFI_AP_CHANNEL 1
#define CONFIG_ESP_WIFI_AP_MAX_STA_CONN 4

#define CONFIG_ESP_WIFI_STATION_SSID "ssid"
#define CONFIG_ESP_WIFI_STATION_PASSWORD "password"
#define CONFIG_ESP_WIFI_STATION_MAXIMUM_RETRY 10

void wifi_ap_server_setup();
void wifi_station_server_setup();
