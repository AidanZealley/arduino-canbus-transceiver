#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);

uint16_t can_id = 0x123;
uint8_t steering_module = 0x12;
uint8_t count_key = 0x01;

const uint32_t can_id_mask = 0x7FF;    // Mask for 11-bit CAN ID
const uint32_t can_id_filter = 0x123;  // Filter for specific CAN ID
const uint8_t module_id_mask = 0xFF;   // Mask for 8-bit module ID
const uint8_t module_id_filter = 0x12; // Filter for specific module ID

int count = 0;

int lastButtonState = HIGH;
unsigned long lastBroadcastTime = 0;
const unsigned long broadcastInterval = 1000;

int sendBtnPin = 6;

void incrementCount() {
  count++;
}

void setup() {
  configureCANFilters();

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
    uint16_t receivedCanId;
    uint8_t receivedModuleId;
    uint8_t receivedKey;
    uint32_t receivedValue;
    
    readCANMessage(&canMsg, receivedCanId, receivedModuleId, receivedKey, receivedValue);

    Serial.print("Received CAN message with ID: ");
    Serial.print(receivedCanId, HEX);
    Serial.print(", Module ID: 0x");
    Serial.print(receivedModuleId, HEX);
    Serial.print(", Key: 0x");
    Serial.print(receivedKey, HEX);
    Serial.print(", Value: ");
    Serial.println(receivedValue);
  }

  if (millis() - lastBroadcastTime >= broadcastInterval) {
    lastBroadcastTime = millis();
    sendCANMessage(can_id, steering_module, count_key, count);
  }

  onButtonPress(sendBtnPin, [](){
    incrementCount();
    sendCANMessage(can_id, steering_module, count_key, count);
  });
}

void configureCANFilters() {
  mcp2515.setFilterMask(MCP2515::MASK0, true, can_id_mask); // true indicates this is for standard IDs
  mcp2515.setFilter(MCP2515::RXF0, true, can_id_filter); // Apply filter for specific CAN ID
  mcp2515.setFilterMask(MCP2515::MASK1, true, module_id_mask); // true indicates this is for standard IDs
  mcp2515.setFilter(MCP2515::RXF2, true, module_id_filter); // Apply filter for specific module ID
}

void readCANMessage(struct can_frame *message, uint16_t &can_id, uint8_t &target_module, uint8_t &key, uint32_t &value) {
    can_id = message->can_id;
    target_module = message->data[0];
    key = message->data[1];
    value = ((uint32_t)message->data[2] << 24) |
            ((uint32_t)message->data[3] << 16) |
            ((uint32_t)message->data[4] << 8) |
            (uint32_t)message->data[5];
}

void sendCANMessage(uint32_t can_id, uint8_t module_id, uint8_t key, uint32_t value) {
    struct can_frame canMsg;
    canMsg.can_id = can_id;

    // Set the module_id and key in the first two bytes of data
    canMsg.data[0] = module_id;
    canMsg.data[1] = key;

    // Set the value in the following bytes, starting with the most significant byte
    canMsg.data[2] = (value >> 24) & 0xFF;
    canMsg.data[3] = (value >> 16) & 0xFF;
    canMsg.data[4] = (value >> 8) & 0xFF;
    canMsg.data[5] = value & 0xFF;

    // Data length code (6 bytes)
    canMsg.can_dlc = 6;

    // Send the message
    MCP2515::ERROR result = mcp2515.sendMessage(&canMsg);

    if (result == MCP2515::ERROR_OK) {
        Serial.print("Sent CAN message with CAN ID: ");
        Serial.print(can_id, HEX);
        Serial.print(", Module ID: 0x");
        Serial.print(module_id, HEX);
        Serial.print(", Key: 0x");
        Serial.print(key, HEX);
        Serial.print(", Value: ");
        Serial.println(value);
    } else {
        Serial.print("Error sending message: ");
        Serial.println(result);
    }
}

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
