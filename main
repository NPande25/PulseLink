// PulseLink ESP32 Code: BLE + BME280 + Compass + 8x8 Matrix with Multi-Device Support
// Uses (x, y) relative positioning from a shared origin device (e.g., pulseA is 0,0)
// Each device advertises its location and elevation. Others receive this and display directional
// feedback using a rotating LED matrix, adjusting for compass heading and pressure difference.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>       // Barometric pressure sensor
#include <LedControl.h>            // For controlling the MAX7219 LED matrix
#include <NimBLEDevice.h>          // BLE library for ESP32
#include <QMC5883LCompass.h>       // Magnetometer (compass)

// --- Sensor Initialization ---
Adafruit_BME280 bme;
QMC5883LCompass compass;
LedControl lc = LedControl(11, 13, 10, 1);  // (DIN, CLK, CS, numDevices)

// --- BLE Variables ---
const char* myID = "pulseA";  // Identifier for this device; pulseA is considered the origin (0,0)
BLEAdvertising* pAdvertising;

// --- Self Tracking ---
float myX = 0.0, myY = 0.0;          // Device's current position
float myPressure = 0.0;             // Measured pressure for elevation
int myHeading = 0;                  // Compass azimuth

// --- Friend Tracking Structure ---
struct FriendData {
  float x, y;       // Friend's position
  float pressure;   // Friend's elevation estimate
  std::string id;   // Device name/ID
};
std::vector<FriendData> friends;    // Stores friends' data

// --- Timing Variables ---
int blinkState = 0;
unsigned long lastBlink = 0;
const int scanInterval = 500;       // BLE scan every 500ms
unsigned long lastScanTime = 0;

// --- Helper: Degrees to Radians ---
float degToRad(float d) { return d * 3.1415926 / 180.0; }

// --- LED Matrix Initialization ---
void setupDisplay() {
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
}

// --- Map Friend Locations to LED Grid ---
void setDisplayPatternMulti(float myHeading, float myPressure) {
  lc.clearDisplay(0);
  unsigned long now = millis();

  for (const auto& f : friends) {
    float dx = f.x - myX;
    float dy = f.y - myY;
    float distance = sqrt(dx * dx + dy * dy);  // Euclidean distance

    float angleToFriend = atan2(dy, dx) * 180.0 / 3.1415926;  // Convert to degrees
    float angleOffset = fmod((angleToFriend - myHeading + 360), 360);  // Relative direction
    float angleRad = degToRad(angleOffset);  // Convert to radians for trig

    float pressureDiff = myPressure - f.pressure;  // Relative elevation

    // Determine radial ring based on distance
    int radius = 0;
    if (distance <= 4) radius = 1;
    else if (distance <= 10) radius = 2;
    else if (distance <= 20) radius = 3;
    else if (distance <= 50) radius = 4;
    else continue;  // Too far, ignore

    // Calculate matrix coordinates
    int centerX = 3, centerY = 3;
    int x = centerX + round(radius * cos(angleRad));
    int y = centerY - round(radius * sin(angleRad));  // Y-axis is inverted

    if (x < 0 || x > 7 || y < 0 || y > 7) continue;  // Skip invalid pixels

    // Blinking logic based on elevation difference
    bool shouldBlink = abs(pressureDiff) >= 0.5;  // Blinking = different floor
    int blinkSpeed = pressureDiff > 0 ? 1000 : 250;  // Above = slow, below = fast
    if (!shouldBlink || (now - lastBlink > blinkSpeed)) {
      blinkState = !blinkState;
      lastBlink = now;
    }

    if (!shouldBlink || blinkState) {
      lc.setLed(0, y, x, true);  // Turn on pixel
    }
  }
}

// --- BLE Scan Callback: Process incoming friend data ---
class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice->haveName() && advertisedDevice->haveManufacturerData()) {
      std::string name = advertisedDevice->getName();
      if (name.find("pulse") != std::string::npos && name != myID) {
        std::string data = advertisedDevice->getManufacturerData();
        float values[3];  // x, y, pressure
        memcpy(values, data.data(), sizeof(values));

        FriendData fd;
        fd.x = values[0];
        fd.y = values[1];
        fd.pressure = values[2];
        fd.id = name;

        // Update friend if already exists
        bool exists = false;
        for (auto& f : friends) {
          if (f.id == fd.id) {
            f = fd;
            exists = true;
            break;
          }
        }
        if (!exists) friends.push_back(fd);  // Otherwise, add new
      }
    }
  }
};

// --- BLE Advertising Setup ---
void setupBLE() {
  NimBLEDevice::init(myID);
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  pAdvertising->start();
}

// --- Send self (x, y, pressure) as BLE manufacturer data ---
void updateAdvertisement(float x, float y, float pressure) {
  float payload[3] = {x, y, pressure};
  std::string out((char*)payload, sizeof(payload));
  pAdvertising->setManufacturerData(out);
  pAdvertising->start();
}

// --- Initialization ---
void setup() {
  Serial.begin(115200);
  setupDisplay();
  Wire.begin();

  // Initialize sensors
  if (!bme.begin(0x76)) {
    Serial.println("Could not find BME280 sensor!");
    while (1);
  }
  compass.init();
  compass.setCalibration(-263, 608, -369, 388, -584, 149);  // Adjust if needed

  // Set fixed position for test/demo
  if (strcmp(myID, "pulseA") == 0) {
    myX = 0.0;
    myY = 0.0;
  } else {
    myX = 5.0;
    myY = 3.0;
  }

  // Setup BLE scanning and advertising
  setupBLE();
  NimBLEDevice::getScan()->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  NimBLEDevice::getScan()->setActiveScan(true);
  NimBLEDevice::getScan()->start(0, nullptr, false);
}

// --- Main Loop ---
void loop() {
  myPressure = bme.readPressure();  // Measure elevation
  compass.read();
  myHeading = compass.getAzimuth();  // Get compass direction
  updateAdvertisement(myX, myY, myPressure);  // Broadcast current data

  // BLE scan every scanInterval ms
  if (millis() - lastScanTime > scanInterval) {
    NimBLEDevice::getScan()->clearResults();
    NimBLEDevice::getScan()->start(0, nullptr, false);
    lastScanTime = millis();
  }

  // Update the matrix based on received data
  setDisplayPatternMulti(myHeading, myPressure);
  delay(50);  // Small delay for stability
}
