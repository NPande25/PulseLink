#include "WiFi.h"  // Include the WiFi library

void setup() {
  Serial.begin(115200);              // Start the Serial communication
  delay(1000);                       // Wait for a moment to stabilize
  WiFi.mode(WIFI_STA);              // Set WiFi mode to Station
  WiFi.begin();                     // Initialize WiFi (no need to connect)
}

void loop() {
  // Get and print the MAC address every 2 seconds
  String macAddress = WiFi.macAddress();
  Serial.print("MAC Address: ");
  Serial.println(macAddress);
  delay(2000);  // Wait for 2 seconds
}
