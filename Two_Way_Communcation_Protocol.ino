#include <esp_now.h>
#include <WiFi.h>

// Replace with your peer's MAC Address
// uint8_t broadcastAddress[] = {0x5C, 0x01, 0x3B, 0x74, 0x46, 0x34}; // MAC Address of RED wires
uint8_t broadcastAddress[] = {0x5C, 0x01, 0x3B, 0x74, 0x74, 0x1C}; // MAC Address of Black wires

// Structure to send and receive messages
typedef struct struct_message {
  char message[32];
} struct_message;

struct_message outgoingMessage;
struct_message incomingMessage;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Format callback when data is received
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
  memcpy(&incomingMessage, data, sizeof(incomingMessage));
  Serial.print("Message received: ");
  Serial.println(incomingMessage.message);
}

void setup() {
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // Prepare the message to be sent
  strcpy(outgoingMessage.message, "bye");

  // Send the message
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));

  if (result == ESP_OK) {
    Serial.println("Sent message: bye");
  } else {
    Serial.println("Error sending the message");
  }

  delay(10000); // Send every 10 seconds
}
