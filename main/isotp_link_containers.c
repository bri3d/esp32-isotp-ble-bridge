#include <stdint.h>
#include "esp_log.h"
#include "isotp_link_containers.h"

int find_isotp_link_container_index_by_send_arbitration_id(uint32_t arbitration_id) {
  for (int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i) {
    IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
    IsoTpLink *link_ptr = &isotp_link_container->link;
    // wait out for uninitialized
    if (link_ptr->initialized == false) {
      continue;
    }
    // flipped?
    if (link_ptr->send_arbitration_id == arbitration_id) {
      return i;
    }
  }
  return -1;
}

int find_isotp_link_container_index_by_receive_arbitration_id(uint32_t arbitration_id) {
  for (int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i) {
    IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
    IsoTpLink *link_ptr = &isotp_link_container->link;
    // wait out for uninitialized
    if (link_ptr->initialized == false) {
      continue;
    }
    // flipped?
    if (link_ptr->receive_arbitration_id == arbitration_id) {
      return i;
    }
  }
  return -1;
}
