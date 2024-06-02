#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);

int lastButtonState = HIGH;

int sendBtnPin = 6;

void onButtonPress(int buttonPin, void (*callback)()) {
  // Read the button state
  int buttonState = digitalRead(buttonPin);

  // Check if the button state has changed
  if (buttonState != lastButtonState) {
    // Check if the button is now pressed and it wasn't pressed before
    if (buttonState == LOW) {
      // Call the callback function
      (*callback)();
    }
  }

  // Update the last button state
  lastButtonState = buttonState;

  delay(50);
}

void sendMessage(uint32_t can_id, char key, uint32_t value) {
  struct can_frame canMsg;
  canMsg.can_id  = can_id;
  
  // Determine the number of bytes needed to represent the value
  uint8_t can_dlc = 1; // Start with 1 byte for the key
  if (value <= 0xFF) {
    can_dlc += 1;
  } else if (value <= 0xFFFF) {
    can_dlc += 2;
  } else if (value <= 0xFFFFFF) {
    can_dlc += 3;
  } else {
    can_dlc += 4;
  }
  canMsg.can_dlc = can_dlc;

  // Set the key in the first byte of data
  canMsg.data[0] = key;

  // Set the value in the following bytes, starting with the least significant byte
  for (uint8_t i = 1; i < can_dlc; i++) {
    canMsg.data[i] = (value >> (8 * (i - 1))) & 0xFF;
  }

  // Send the message
  mcp2515.sendMessage(&canMsg);
  Serial.print("Message sent with CAN ID: ");
  Serial.print(can_id, HEX);
  Serial.print(", Key: ");
  Serial.print(key);
  Serial.print(", Value: ");
  Serial.println(value);
}

void setup() {
  pinMode(sendBtnPin, INPUT_PULLUP);

  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_100KBPS);
  mcp2515.setNormalMode();

  Serial.println("CAN Transceiver Initialized");
}

void loop() {
  struct can_frame canMsg;
  
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("Received CAN message with ID: ");
    Serial.print(canMsg.can_id, HEX);
    Serial.print(" Data: ");

    for (int i = 0; i < canMsg.can_dlc; i++) {
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
  }

  onButtonPress(sendBtnPin, [](){ sendMessage(0x0F6, 'A', 12345); });
}
