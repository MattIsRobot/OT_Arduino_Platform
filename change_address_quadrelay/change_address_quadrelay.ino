#include <Wire.h>
#include <SparkFun_Qwiic_Relay.h>

// Original and new addresses
#define ORIGINAL_RELAY2_ADDR 0x6D
#define NEW_RELAY2_ADDR      0x6C  // Choose a unique address between 0x07 and 0x78

// Initialize Quad Relay 2 with its original address
Qwiic_Relay quadRelay2(ORIGINAL_RELAY2_ADDR);

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(100000); // Standard I2C speed

  Serial.println("Attempting to change Quad Relay 2's I2C address...");

  // Attempt to communicate with Quad Relay 2
  if (quadRelay2.begin()) {
    Serial.print("Quad Relay 2 found at address 0x");
    if (ORIGINAL_RELAY2_ADDR < 16) Serial.print("0");
    Serial.print(ORIGINAL_RELAY2_ADDR, HEX);
    Serial.println(".");

    // Attempt to change the address
    bool success = quadRelay2.changeAddress(NEW_RELAY2_ADDR);

    if (success) {
      Serial.print("Successfully changed Quad Relay 2's address to 0x");
      if (NEW_RELAY2_ADDR < 16) Serial.print("0");
      Serial.print(NEW_RELAY2_ADDR, HEX);
      Serial.println(".");
    } else {
      Serial.println("Failed to change Quad Relay 2's address.");
    }
  } else {
    Serial.println("Quad Relay 2 not detected at address 0x6C.");
    Serial.println("Ensure it is properly connected and powered.");
  }

  Serial.println("Address change process complete.");
}

void loop() {
  // No actions needed in loop
}
