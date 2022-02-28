/**
  LoRa Base Node
    Base LoRa code that receives GPS coordinates periodically
    from multiple remote LoRa nodes. 
**/

#include <SPI.h>
#include <RH_RF95.h>

// Feather 32u4 specific LoRa pins.
#define RFM95_CS   8    // Chip Select pin.
#define RFM95_RST  4    // Reset pin.
#define RFM95_INT  7    // Interrupt pin.

// LoRa transmission frequency.
#define RF95_FREQ  915.0

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

  Serial.println("Starting Base Node.");

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
    while (true);
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
  // Wait .25 seconds between checking for new messages.
  delay(250);
  Serial.println("Listening...");

  // Check if a message is available in the chip buffer.
  if (rf95.available()) {
    uint8_t mesg[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(mesg);

    if (rf95.recv(mesg, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);

      RH_RF95::printBuffer("RECEIVED: ", mesg, len);
      Serial.print("Message: ");
      Serial.println(mesg[0]);

      // Print out the Receiver Signal Strength Indicator (RSSI).
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    }

    digitalWrite(LED_BUILTIN, LOW);
  }
}
