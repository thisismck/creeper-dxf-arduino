#include <WiFi.h>
#include <esp_now.h>

// Replace with the receiver's MAC address
uint8_t broadcastAddress[] = {0xE4, 0x65, 0xB8, 0xD8, 0x7A, 0xE0};
// Structure for sending motor data
typedef struct MotorData {
  int forward;   // Forward speed (0-255)
  int backward;  // Backward speed (0-255)
  int turn;      // -1 = left, 0 = straight, 1 = right
} MotorData;

// Create a MotorData instance
MotorData motorData = {0, 0, 0};

// Define button pins
const int forwardButton = 15;  // Forward button GPIO15
const int backwardButton = 14; // Backward button GPIO14
const int leftButton = 13;     // Left button GPIO13
const int rightButton = 12;    // Right button GPIO12

// ESP-NOW peer information
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Configure button pins as input
  pinMode(forwardButton, INPUT_PULLUP);
  pinMode(backwardButton, INPUT_PULLUP);
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
}

void loop() {
  // Read button states
  bool isForwardPressed = !digitalRead(forwardButton);
  bool isBackwardPressed = !digitalRead(backwardButton);
  bool isLeftPressed = !digitalRead(leftButton);
  bool isRightPressed = !digitalRead(rightButton);

  // Set motor data based on button states
  motorData.forward = isForwardPressed ? 255 : 0;  // Max forward speed
  motorData.backward = isBackwardPressed ? 255 : 0; // Max backward speed
  motorData.turn = isLeftPressed ? -1 : (isRightPressed ? 1 : 0); // Left, right, or straight

  // Send motor data via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&motorData, sizeof(motorData));

  // Log the result
  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error sending data");
  }

  // Add a small delay to reduce frequency of sending
  delay(250);
}
