#include <stdint.h>
#include "isotp_link_containers.h"

int find_isotp_link_container_index(uint32_t rx_id) {
  for (int i = 0; i < NUM_ISOTP_LINK_CONTAINERS; ++i) {
    IsoTpLinkContainer *isotp_link_container = &isotp_link_containers[i];
    IsoTpLink *link_ptr = &isotp_link_container->link;
    if (link_ptr->receive_arbitration_id == rx_id) {
      return i;
    }
  }
  return -1;
}
