#include "esp_http_server.h"

static const char *index_html = R"EOF(
<!doctype html>
<html>
  <head>
    <title>esp32-isotp-ble-bridge</title>
  </head>
  <body>
    <script type="text/javascript">
      const uint32ToBytes = (value) => {
        return [
          value & 0xFF,
          (value >> 8) & 0xFF,
          (value >> 16) & 0xFF,
          (value >> 24) & 0xFF
        ]
      }
      const sendIsoTpRequest = (socket, arbitrationId, frame) => {
        const arbitrationIdBytes = uint32ToBytes(arbitrationId)
        const payload = [].concat(arbitrationIdBytes, frame)
        socket.send(Uint8Array.from(payload))
      }
      const run = async () => {
        const socket = new WebSocket(`ws://${window.location.host}/ws`)
        await new Promise(resolve => socket.addEventListener('open', resolve))
        socket.addEventListener('message', (event) => {
          console.log(event)
        })
        socket.addEventListener('close', (event) => {
          console.log(event)
        })
        setInterval(() => {
          const requestArbitrationId = 0x7E0 // ECU
          const pdu = [0x3E, 0x00] // tester present
          sendIsoTpRequest(socket, requestArbitrationId, pdu)
        }, 1000)
      }
      run()
    </script>
  </body>
</html>
)EOF";

httpd_handle_t web_server_setup();
