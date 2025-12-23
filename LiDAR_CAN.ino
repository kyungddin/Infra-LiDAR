#include <mcp_can.h>
#include <SPI.h>
#include <SoftwareSerial.h>

// Hardware Pin Definitions
#define SPI_CS_PIN  10
#define RX_PIN      5
#define TX_PIN      6

MCP_CAN CAN0(SPI_CS_PIN);
SoftwareSerial mySerial(RX_PIN, TX_PIN);

void setup() {
  Serial.begin(115200);   // Debug Port
  mySerial.begin(9600);   // Motor Controller Port

  Serial.println("=== CAN Relay Node Started ===");

  // Initialize CAN Bus (500kbps @ 16MHz)
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("CAN Init Success");
  } else {
    Serial.println("CAN Init Failed");
    while (1);
  }
  CAN0.setMode(MCP_NORMAL);
}

void loop() {
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    // Filter ID: 0x123 (Control Command)
    if (rxId == 0x123 && len == 4) {
      // Relay Data: Bypass CAN payload to UART directly
      // Protocol: [Header 0xFE 0xFF] [Vel_H] [Vel_L] [Steer_H] [Steer_L]
      
      mySerial.write(0xFE);
      mySerial.write(0xFF);
      
      mySerial.write(rxBuf[0]); // Velocity High
      mySerial.write(rxBuf[1]); // Velocity Low
      mySerial.write(rxBuf[2]); // Steer High
      mySerial.write(rxBuf[3]); // Steer Low
      
      // Serial.println("Relayed Command"); // Uncomment for debugging
    }
  }
}