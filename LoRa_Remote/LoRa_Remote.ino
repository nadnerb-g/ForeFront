/*
  LoRa Remote Node
    Remote LoRa code that transmits GPS coordinates periodically
    to the base station. 
*/

#include <SPI.h>
#include <RH_RF95.h>

// Feather 32u4 specific LoRa pins.
#define RFM95_CS   8    // Chip Select pin.
#define RFM95_RST  4    // Reset pin.
#define RFM95_INT  7    // Interrupt pin.

// LoRa transmission frequency.
#define RF95_FREQ  915.0

#define BUFFER_LENGTH 20

// Global variable.
char mesg[BUFFER_LENGTH];
int pkt_id = 0;

// Radio driver instance.
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  // Set up pin directions and initially set.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Start the serial connection at 115200 bps.
  Serial.begin(115200);
  while(!Serial) {
    delay(1);
  }
  delay(100);

  Serial.println("Starting Remote Node.");

  // Reset the chip.
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize LoRa chip.
  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    //Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");

    // setup() can't really fail, so just hang.
    while (1);
  }
  Serial.println("LoRa radio successfully initialized.");

  // Set LoRa chip frequency.
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Failed to set LoRa chip frequency.");

    // setup() can't really fail, so just hang.
    while(true);
  }
  Serial.println("Frequency successfully set.");

  // Set the transmission power.
  rf95.setTxPower(23, false);
}

void loop() {
  // Wait 1 second between transmissions.
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Transmitting...");

  memset(mesg, 0, sizeof(mesg));
  itoa(pkt_id, mesg, 10);
  rf95.send((uint8_t *)mesg, BUFFER_LENGTH);

  Serial.println("Waiting for packet to complete sending.");
  rf95.waitPacketSent();
  Serial.print("Packet sent. ");
  Serial.println(mesg);

  digitalWrite(LED_BUILTIN, LOW);
  pkt_id++;
}
