#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * DEFINES
 ****************************************************************************************
*/

//BLE header ID
#define BLE_HEADER_ID					0xF1
#define BLE_PARTIAL_ID					0xF2

//BLE command flags
#define BLE_COMMAND_FLAG_PER_ENABLE		1
#define BLE_COMMAND_FLAG_PER_CLEAR		2
#define BLE_COMMAND_FLAG_PER_ADD		4
#define BLE_COMMAND_FLAG_SPLIT_PK		8
#define BLE_COMMAND_FLAG_LED_COLOR		16

//BLE send queue size
#define SEND_QUEUE_SIZE					64

//BLE max congestion
#define BLE_CONGESTION_MAX				5000

#define spp_sprintf(s,...)         		sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           		(512)
#define SPP_CMD_MAX_LEN            		(20)
#define SPP_STATUS_MAX_LEN         		(20)
#define SPP_DATA_BUFF_MAX_LEN      		(2*1024)
///Attributes State Machine
enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_CHAR,
    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NOTIFY_CHAR,
    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,

    SPP_IDX_SPP_COMMAND_CHAR,
    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_CHAR,
    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,
    SPP_IDX_NB,
};

typedef struct send_message{
	int32_t msg_length;
	uint8_t* buffer;
	uint16_t rxID;
	uint16_t txID;
} send_message_t;

// Header we expect to receive on BLE packets
typedef struct ble_header {
	uint8_t		hdID;
	uint8_t		cmdFlags;
	uint16_t	rxID;
	uint16_t	txID;
	uint16_t	cmdSize;
} ble_header_t;

typedef struct 
{
	void (*data_received)			(const void* src, size_t size);		/* Data received callback - a full frame has been constructed from the client. Buf is not guaranteed to live and should be copied. */
    void (*notifications_subscribed)();
    void (*notifications_unsubscribed)();
} ble_server_callbacks;

void ble_server_setup(ble_server_callbacks callbacks);
void ble_server_shutdown();
void ble_send(uint32_t txID, uint32_t rxID, const void* src, size_t size);
bool ble_connected();
uint16_t ble_queue_spaces();
