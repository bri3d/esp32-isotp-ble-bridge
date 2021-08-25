idf_component_register(
  SRCS
  "main.c"
  "isotp.c"
  "ble_server.c"
  "wifi_server.c"
  "web_server.c"
  "endian_helpers.c"
  "ws2812_control.c"
  "ble_callbacks.c"
  "isotp_callbacks.c"
  "twai.c"
  "isotp_links.c"
  "isotp_tasks.c"
  "isotp_link_containers.c"
  "periodic_messages.c"
  INCLUDE_DIRS "."
)
