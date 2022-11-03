// c++ deps
#include <mutex>
#include <queue>
// arduino deps
#include <Arduino.h>
// esp32 deps
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
// git@github.com:arkhipenko/TaskScheduler.git
#include <TaskScheduler.h>
// git@github.com:brandonros/isotp-c.git
#include <isotp.h>
#include <can.h>

// protocol
#define PROTOCOL_HEADER_SIZE 8

// BLE
#define SERVICE_UUID "0000abf0-0000-1000-8000-00805f9b34fb"
#define DATA_NOTIFY_CHARACTERISTIC_UUID "0000abf2-0000-1000-8000-00805f9b34fb"
#define COMMAND_WRITE_CHARACTERISTIC_UUID "0000abf3-0000-1000-8000-00805f9b34fb"
#define DEVICE_NAME "BLE_TO_ISOTP"

// ISOTP
#define ISOTP_BUFSIZE 4096
#define ISO_TP_DEFAULT_ST_MIN_US 500
#define ISO_TP_DEFAULT_RESPONSE_TIMEOUT_US 100000
#define ISO_TP_DEFAULT_BLOCK_SIZE 0x20

int tx_isotp_on_ble_rx(uint16_t request_arbitration_id, uint16_t reply_arbitration_id, uint8_t *msg, uint16_t msg_length);
void tx_ble_on_isotp_rx(uint16_t rx_id, uint16_t tx_id, uint8_t *buffer, uint16_t len);

// math
void write_uint32_be(uint32_t value, uint8_t *output) {
  output[3] = value & 0xFF;
  output[2] = (value >> 8) & 0xFF;
  output[1] = (value >> 16) & 0xFF;
  output[0] = (value >> 24) & 0xFF;
}

uint32_t read_uint32_be(const uint8_t *data) {
  return data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);
}

uint16_t read_uint16_be(const uint8_t *data) {
  return data[1] | (data[0] << 8);
}

// CAN
uint8_t can_rx_buf[8];

// ISOTP user functions
void isotp_user_debug(const char* format, ...) {
  // TODO: log?
}

int isotp_user_send_can(uint32_t arbitration_id, const uint8_t* data, uint8_t size) {
  int ret_val = can_send(arbitration_id, data, size);
  if (ret_val != ESP_OK) {
    Serial.printf("isotp_user_send_can: can_send ret_val = %08x\n", ret_val);
    // we need to stop -> start the TWAI driver or else we'll have 0 success flashing after a timeout has occurred
    can_reset();
    return ISOTP_RET_ERROR;
  }
  return ISOTP_RET_OK;
}

uint32_t isotp_user_get_us() {
  return micros();
}

// ISOTP link containers
typedef struct {
  bool initialized;
  uint16_t request_arbitration_id;
  uint16_t reply_arbitration_id;
  IsoTpLink isotp_link;
  uint8_t isotp_rx_buffer[ISOTP_BUFSIZE];
  uint8_t isotp_tx_buffer[ISOTP_BUFSIZE];
  uint8_t isotp_payload_buffer[ISOTP_BUFSIZE];
} IsoTpLinkContainer;
IsoTpLinkContainer link_containers[4];

IsoTpLinkContainer* find_link_container_by_request_arbitration_id(uint16_t request_arbitration_id) {
  for (int i = 0; i < 4; ++i) {
    if (link_containers[i].initialized == false) {
      continue;
    }
    if (link_containers[i].request_arbitration_id == request_arbitration_id) {
      return &link_containers[i];
    }
  }
  return NULL;
}

IsoTpLinkContainer* find_link_container_by_reply_arbitration_id(uint16_t reply_arbitration_id) {
  for (int i = 0; i < 4; ++i) {
    if (link_containers[i].initialized == false) {
      continue;
    }
    if (link_containers[i].reply_arbitration_id == reply_arbitration_id) {
      return &link_containers[i];
    }
  }
  return NULL;
}

// BLE
enum ble_states {
  WAITING_FOR_CLIENT,
  HAVE_CLIENT,
  CLIENT_NOTIFIED
};

enum ble_command_ids {
  UPLOAD_ISOTP_CHUNK = 0x02,
  FLUSH_ISOTP_PAYLOAD = 0x03,
  START_PERIODIC_MESSAGE = 0x04,
  STOP_PERIODIC_MESSAGE = 0x05,
  CONFIGURE_ISOTP_LINK = 0x06
};

BLEServer *pServer;
BLEService *pService;
BLECharacteristic *pDataNotifyCharacteristic;
BLECharacteristic *pCommandWriteCharacteristic;

static uint8_t ble_tx_command_buf[ISOTP_BUFSIZE + PROTOCOL_HEADER_SIZE] = {0};
static uint8_t ble_rx_command_buf[ISOTP_BUFSIZE + PROTOCOL_HEADER_SIZE] = {0};

ble_states ble_state = WAITING_FOR_CLIENT;

std::mutex ble_command_mtx;

void process_ble_command(uint8_t *data, size_t data_length) {
  ble_command_ids ble_command_id = (ble_command_ids)data[0];
  if (ble_command_id == CONFIGURE_ISOTP_LINK) {
    // parse
    size_t pointer = 1;
    uint32_t link_index = read_uint32_be(data + pointer);
    pointer += 4;
    uint32_t request_arbitration_id = read_uint32_be(data + pointer);
    pointer += 4;
    uint32_t reply_arbitration_id = read_uint32_be(data + pointer);
    pointer += 4;
    uint32_t name_len = read_uint32_be(data + pointer);
    pointer += 4;
    char *name = (char*)(data + pointer);
    // log
    Serial.printf("CONFIGURE_ISOTP_LINK: link_index = %02x request_arbitration_id = %08x reply_arbitration_id = %08x name_len = %08x name = %s\n", link_index, request_arbitration_id, reply_arbitration_id, name_len, name);
    // configure
    isotp_init_link(&link_containers[link_index].isotp_link, request_arbitration_id, link_containers[link_index].isotp_tx_buffer, ISOTP_BUFSIZE, link_containers[link_index].isotp_rx_buffer, ISOTP_BUFSIZE);
    link_containers[link_index].initialized = true;
    link_containers[link_index].request_arbitration_id = request_arbitration_id;
    link_containers[link_index].reply_arbitration_id = reply_arbitration_id;
    // TODO: send command success?
  } else if (ble_command_id == UPLOAD_ISOTP_CHUNK) {
    // parse
    uint32_t chunk_offset = read_uint16_be(data + 1);
    uint32_t chunk_length = read_uint16_be(data + 3);
    uint8_t *bytes = data + 5;
    // log
    Serial.printf("UPLOAD_ISOTP_CHUNK chunk_offset = %04x chunk_length = %04x\n", chunk_offset, chunk_length);
    // copy
    memcpy(ble_rx_command_buf + chunk_offset, bytes, chunk_length);
    // TODO: send command success?
  } else if (ble_command_id == FLUSH_ISOTP_PAYLOAD) {
    // parse
    uint16_t payload_length = read_uint16_be(data + 1);
    uint32_t request_arbitration_id = read_uint32_be(ble_rx_command_buf);
    uint32_t reply_arbitration_id = read_uint32_be(ble_rx_command_buf + 4);
    uint16_t msg_length = payload_length - 8;
    uint8_t *msg = ble_rx_command_buf + 8;
    // log
    Serial.printf("FLUSH_ISOTP_PAYLOAD payload_length = %04x request_arbitration_id = %08x reply_arbitration_id = %08x msg_length = %04x\n", payload_length, request_arbitration_id, reply_arbitration_id, msg_length);
    // dispatch
    tx_isotp_on_ble_rx(request_arbitration_id, reply_arbitration_id, msg, msg_length);
    // TODO: send command success?
  } else if (ble_command_id == START_PERIODIC_MESSAGE) {
    // TODO
  } else if (ble_command_id == STOP_PERIODIC_MESSAGE) {
    // TODO
  } else {
    Serial.printf("unknown command ID: %02x\n", ble_command_id);
  }
}

// BLE callbacks
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("onConnect");
    if (ble_state == WAITING_FOR_CLIENT) {
      ble_state = HAVE_CLIENT;
    }
  };

  void onDisconnect(BLEServer* pServer) {
    Serial.println("onDisconnect");
    ble_state = WAITING_FOR_CLIENT;
      // restart?
    ESP.restart();
  }
};

class CommandWriteCharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
    // lock
    ble_command_mtx.lock();
    // process
    uint8_t *data = pCharacteristic->getData();
    size_t data_length = pCharacteristic->getLength();
    process_ble_command(data, data_length);
    // unlock
    ble_command_mtx.unlock();
  }
};

// isotp + ble
int tx_isotp_on_ble_rx(uint16_t request_arbitration_id, uint16_t reply_arbitration_id, uint8_t *msg, uint16_t msg_length) {
  Serial.printf("tx_isotp_on_ble_rx: sending to request_arbitration_id = %04x reply_arbitration_id = %04x msg_length = %04x...\n", request_arbitration_id, reply_arbitration_id, msg_length);
  IsoTpLinkContainer *link_container = find_link_container_by_request_arbitration_id(request_arbitration_id);
  // check if link is currently sending?
  for (;;) {
    if (link_container->isotp_link.send_status != ISOTP_SEND_STATUS_INPROGRESS) {
      break;
    }
    delay(1);
  }
  // start sending
  int ret_val = isotp_send_with_id(&link_container->isotp_link, request_arbitration_id, msg, msg_length);
  if (ret_val != ISOTP_RET_OK) {
    Serial.printf("isotp_send_with_id: ret_val = %08x\n", ret_val);
  }
  // wait for sending all frames to finish?
  for (;;) {
    if (link_container->isotp_link.send_status != ISOTP_SEND_STATUS_INPROGRESS) {
      break;
    }
    delay(1);
  }
  // check result
  return link_container->isotp_link.send_protocol_result;
}

void tx_ble_on_isotp_rx(uint16_t rx_id, uint16_t tx_id, uint8_t *buffer, uint16_t len) {
  Serial.printf("tx_ble_on_isotp_rx rx_id = %04x tx_id = %04x len = %04x\n", rx_id, tx_id, len);
  // short circuit if no client to send to?
  if (ble_state == WAITING_FOR_CLIENT) {
    return;
  }
  // build message
  write_uint32_be(tx_id, ble_tx_command_buf); // flipped?
  write_uint32_be(rx_id, ble_tx_command_buf + 4); // flipped?
  memcpy(ble_tx_command_buf + 8, buffer, len);
  // set value + notify
  pDataNotifyCharacteristic->setValue(ble_tx_command_buf, len + 8);
  pDataNotifyCharacteristic->notify();
  // sleep to prevent bluetooth congestion?
  delay(10);
}

// task callbacks
void can_rx_task_callback() {
  // read from CAN
  uint16_t arbitration_id = 0;
  size_t size = 0;
  int can_recv_ret_val = can_recv(&arbitration_id, can_rx_buf, &size);
  if (can_recv_ret_val == ESP_OK) {
    // short circuit on low level traffic
    // TODO: get actual CAN filters working?
    if (arbitration_id < 0x500) {
      return;
    }
    // find link container
    IsoTpLinkContainer *link_container = find_link_container_by_reply_arbitration_id(arbitration_id);
    if (link_container) {
      isotp_on_can_message(&link_container->isotp_link, can_rx_buf, size);
    }
  } else if (can_recv_ret_val == ESP_ERR_TIMEOUT) {
    // expected timeout trying to read (no data available?)
  } else {
    Serial.printf("can_recv_ret_val = %08x\n", can_recv_ret_val);
  }
}

void isotp_poll_task_callback() {
  for (int i = 0; i < 4; ++i) {
    // skip uninitialized links
    if (link_containers[i].initialized == false) {
      continue;
    }
    // poll
    isotp_poll(&link_containers[i].isotp_link);
  }
}

void isotp_receive_task_callback() {
  for (int i = 0; i < 4; ++i) {
    // skip uninitialized links
    if (link_containers[i].initialized == false) {
      continue;
    }
    // receive
    uint16_t out_size = 0;
    int isotp_receive_ret_val = isotp_receive(&link_containers[i].isotp_link, link_containers[i].isotp_payload_buffer, ISOTP_BUFSIZE, &out_size);
    if (isotp_receive_ret_val == ISOTP_RET_OK) {
      tx_ble_on_isotp_rx(link_containers[i].request_arbitration_id, link_containers[i].reply_arbitration_id, link_containers[i].isotp_payload_buffer, out_size);
    } else if (isotp_receive_ret_val == ISOTP_RET_NO_DATA) {
       // expected timeout trying to read (no data available?)
    } else {
      Serial.printf("isotp_receive_ret_val = %08x\n", isotp_receive_ret_val);
    }
  }
}

// scheduler + tasks
Scheduler ts;
Task can_rx_task(TASK_IMMEDIATE, TASK_FOREVER, &can_rx_task_callback, &ts, true);
Task isotp_poll_task(TASK_IMMEDIATE, TASK_FOREVER, &isotp_poll_task_callback, &ts, true);
Task isotp_receive_task(TASK_IMMEDIATE, TASK_FOREVER, &isotp_receive_task_callback, &ts, true);

extern "C" void app_main() {
  initArduino();
  // serial
  Serial.begin(115200);
  // CAN
  pinMode(GPIO_NUM_21, OUTPUT);
  digitalWrite(GPIO_NUM_21, LOW); 
  can_init();
  // ISOTP
  for (int i = 0; i < 4; ++i) {
    link_containers[i].initialized = false;
    memset(link_containers[i].isotp_payload_buffer, 0, ISOTP_BUFSIZE);
    memset(link_containers[i].isotp_rx_buffer, 0, ISOTP_BUFSIZE);
    memset(link_containers[i].isotp_tx_buffer, 0, ISOTP_BUFSIZE);
  }
  // BLE
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  // BLE data notify
  pDataNotifyCharacteristic = pService->createCharacteristic(
    DATA_NOTIFY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pDataNotifyCharacteristic->addDescriptor(new BLE2902());
  // BLE command write
  pCommandWriteCharacteristic = pService->createCharacteristic(
    COMMAND_WRITE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE_NR // TODO: PROPERTY_WRITE instead of PROPERTY_WRITE_NR?
  );
  pCommandWriteCharacteristic->setCallbacks(new CommandWriteCharacteristicCallbacks());
  // BLE start
  pService->start();
  // BLE advertising
  pServer->getAdvertising()->addServiceUUID(pService->getUUID());
  pServer->getAdvertising()->start();
  // tasks
  for (;;) {
    ts.execute();
  }
}
