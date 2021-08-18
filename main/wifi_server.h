#include "esp_event.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define CONFIG_ESP_WIFI_SSID "ssid"
#define CONFIG_ESP_WIFI_PASSWORD "password123"
#define CONFIG_ESP_WIFI_CHANNEL 1
#define CONFIG_ESP_MAX_STA_CONN 4

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_server_setup();
