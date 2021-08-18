#include <string.h>
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "wifi_server.h"

const char *WIFI_SERVER_TAG = "wifi_server";

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(WIFI_SERVER_TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(WIFI_SERVER_TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
    }
}

void wifi_server_setup()
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_ap();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      NULL));
  wifi_config_t wifi_config = {
      .ap = {
          .ssid = CONFIG_ESP_WIFI_SSID,
          .ssid_len = strlen(CONFIG_ESP_WIFI_SSID),
          .channel = CONFIG_ESP_WIFI_CHANNEL,
          .password = CONFIG_ESP_WIFI_PASSWORD,
          .max_connection = CONFIG_ESP_MAX_STA_CONN,
          .authmode = WIFI_AUTH_WPA_WPA2_PSK
      },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_LOGI(WIFI_SERVER_TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d", CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD, CONFIG_ESP_WIFI_CHANNEL);
}
