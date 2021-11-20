#ifndef CONSTANTS_H
#define CONSTANTS_H

/* --------------------- Definitions and static variables ------------------ */

//Settings
#define BRG_SETTING_ISOTP_STMIN			1
#define BRG_SETTING_LED_COLOR			2
#define BRG_SETTING_PERSIST_DELAY		3
#define BRG_SETTING_PERSIST_Q_DELAY		4
#define BRG_SETTING_BLE_SEND_DELAY		5
#define BRG_SETTING_BLE_MULTI_DELAY		6
#define BRG_SETTING_PASSWORD			7
#define BRG_SETTING_GAP					8

#define RX_TASK_PRIO 			 		3 // Ensure we drain the RX queue as quickly as we reasonably can to prevent overflow and ensure the message pump has fresh data.
#define TX_TASK_PRIO 			 		3 // Ensure we TX messages as quickly as we reasonably can to meet ISO15765-2 timing constraints
#define ISOTP_TSK_PRIO 					2 // Run the message pump at a higher priority than the main queue/dequeue task when messages are available
#define MAIN_TSK_PRIO 					1 // Run the main task at the same priority as the BLE queue/dequeue tasks to help in delivery ordering.
#define PERSIST_TSK_PRIO 				0
#define SLEEP_TSK_PRIO 					0
#define UART_TSK_PRIO 					0

#define ADC_GPIO_NUM					35
#define ADC_CHANNEL_NUM					ADC1_CHANNEL_7
#define TX_GPIO_NUM 					5 // For A0
#define RX_GPIO_NUM 					4 // For A0
#define SILENT_GPIO_NUM 				21 // For A0
#define LED_ENABLE_GPIO_NUM 			13 // For A0
#define LED_GPIO_NUM 					2 // For A0
#define GPIO_OUTPUT_PIN_SEL(X)  		((1ULL<<X))

#define ISOTP_BUFSIZE 					4096

#define TIMEOUT_SHORT					50
#define TIMEOUT_NORMAL					100
#define TIMEOUT_LONG					1000
#define TIMEOUT_CAN						1

#define TIMEOUT_FIRSTBOOT				30

#define TIMEOUT_UARTCONNECTION			120


#define SLEEP_TIME						10

#define US_TO_S							1000000


#define MAX_PASSWORD_LENGTH				64

#define PASSWORD_KEY					"Password"

#define PASSWORD_DEFAULT			   	"BLE2"

#define BLE_GAP_KEY		  				"GAP"


#define UART_TXD 						UART_PIN_NO_CHANGE

#define UART_RXD 						UART_PIN_NO_CHANGE
#define UART_RTS 						UART_PIN_NO_CHANGE
#define UART_CTS 						UART_PIN_NO_CHANGE
#define UART_PORT_NUM      				UART_NUM_0
#define UART_BAUD_RATE     				250000
#define UART_BUFFER_SIZE				8192
#define UART_ECHO						0

#endif