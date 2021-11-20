#define RX_TASK_PRIO 3         // Ensure we drain the RX queue as quickly as we reasonably can to prevent overflow and ensure the message pump has fresh data.
#define TX_TASK_PRIO 3         // Ensure we TX messages as quickly as we reasonably can to meet ISO15765-2 timing constraints
#define ISOTP_TSK_PRIO 2       // Run the message pump at a higher priority than the main queue/dequeue task when messages are available
#define SOCKET_TASK_PRIO 1      // Run the socket task at a low priority.
#define MAIN_TSK_PRIO 1        // Run the main task at the same priority as the BLE queue/dequeue tasks to help in delivery ordering.
