#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <NimBLEDevice.h>
#include <QMC5883LCompass.h>

// --- Sensor Setup ---
Adafruit_BME280 bme;
QMC5883LCompass compass;

#define DATA_PIN  12
#define CLK_PIN   11
#define CS_PIN    10
#define MAX_DEVICES 1
MD_MAX72XX lc = MD_MAX72XX(MD_MAX72XX::FC16_HW, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// --- BLE Variables ---
const char* myID = "pulseA";  // Change per device
BLEAdvertising* pAdvertising;

// --- Self Tracking ---
float myX = 0.0, myY = 0.0;
float myPressure = 0.0;
int myHeading = 0;

// --- Friends ---
struct FriendData {
  float x, y;
  float pressure;
  std::string id;
};
std::vector<FriendData> friends;

// --- Timing ---
int blinkState = 0;
unsigned long lastBlink = 0;
const int scanInterval = 500;
unsigned long lastScanTime = 0;
const int aloneBlinkSpeed = 700;

// --- Helpers ---
float degToRad(float d) {
  return d * 3.1415926 / 180.0;
}

void setupDisplay() {
  lc.begin();
  lc.control(MD_MAX72XX::INTENSITY, 8);
  lc.clear();
}

// --- Visualize Friends ---
void setDisplayPatternMulti(float myHeading, float myPressure) {
  lc.clear();
  unsigned long now = millis();

  if (friends.empty()) {
    if (now - lastBlink > aloneBlinkSpeed) {
      blinkState = !blinkState;
      lastBlink = now;
    }
    if (blinkState) {
      lc.setPoint(3, 3, true);
      lc.setPoint(3, 4, true);
      lc.setPoint(4, 3, true);
      lc.setPoint(4, 4, true);
    }
    Serial.println("🔕 No friends detected. Showing center blink.");
    return;
  }

  for (const auto& f : friends) {
    float dx = f.x - myX;
    float dy = f.y - myY;
    float distance = sqrt(dx * dx + dy * dy);

    float angleToFriend = atan2(dy, dx) * 180.0 / 3.1415926;
    float angleOffset = fmod((angleToFriend - myHeading + 360), 360);
    float angleRad = degToRad(angleOffset);

    float pressureDiff = myPressure - f.pressure;

    int radius = 0;
    if (distance <= 5) radius = 1;
    else if (distance <= 10) radius = 2;
    else if (distance <= 20) radius = 3;
    else if (distance <= 50) radius = 4;
    else continue;

    int centerX = 3, centerY = 3;
    int x = centerX + round(radius * cos(angleRad));
    int y = centerY - round(radius * sin(angleRad));

    if (x < 0 || x > 7 || y < 0 || y > 7) continue;

    bool shouldBlink = abs(pressureDiff) >= 0.5;
    int blinkSpeed = pressureDiff > 0 ? 1000 : 250;

    if (!shouldBlink || (now - lastBlink > blinkSpeed)) {
      blinkState = !blinkState;
      lastBlink = now;
    }

    if (!shouldBlink || blinkState) {
      lc.setPoint(y, x, true);
    }

    Serial.print("📍 Friend ");
    Serial.print(f.id.c_str());
    Serial.print(" at (");
    Serial.print(f.x);
    Serial.print(", ");
    Serial.print(f.y);
    Serial.print("), distance: ");
    Serial.print(distance);
    Serial.print(", pressureDiff: ");
    Serial.println(pressureDiff);
  }
}

// --- BLE Scan Callback ---
class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    if (advertisedDevice->haveName() && advertisedDevice->haveManufacturerData()) {
      std::string name = advertisedDevice->getName();
      if (name.find("pulse") == std::string::npos || name == myID) return;

      std::string data = advertisedDevice->getManufacturerData();
      if (data.length() < 4 + sizeof(float) * 3) return;

      if (data.substr(0, 4) != "PLNK") return;

      float values[3];
      memcpy(values, data.data() + 4, sizeof(values));

      FriendData fd;
      fd.x = values[0];
      fd.y = values[1];
      fd.pressure = values[2];
      fd.id = name;

      bool exists = false;
      for (auto& f : friends) {
        if (f.id == fd.id) {
          f = fd;
          exists = true;
          break;
        }
      }
      if (!exists) friends.push_back(fd);

      Serial.print("✅ Received from ");
      Serial.print(fd.id.c_str());
      Serial.print(": x=");
      Serial.print(fd.x);
      Serial.print(", y=");
      Serial.print(fd.y);
      Serial.print(", pressure=");
      Serial.println(fd.pressure);
    }
  }
};

// --- BLE Setup ---
void setupBLE() {
  NimBLEDevice::init(myID);
  NimBLEServer* pServer = NimBLEDevice::createServer();
  pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

// --- Broadcast Self Data ---
void updateAdvertisement(float x, float y, float pressure) {
  const char* key = "PLNK";
  float payload[3] = {x, y, pressure};
  std::string out(key, 4);
  out.append((char*)payload, sizeof(payload));
  pAdvertising->setManufacturerData(out);
  pAdvertising->start();

  Serial.print("📡 Advertising: x=");
  Serial.print(x);
  Serial.print(", y=");
  Serial.print(y);
  Serial.print(", pressure=");
  Serial.println(pressure);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Serial.println("✅ PulseLink booted successfully!");
  setupDisplay();
  Wire.begin();

  if (!bme.begin(0x76)) {
    Serial.println("❌ Could not find BME280 sensor!");
    while (1);
  }

  compass.init();
  compass.setCalibration(-263, 608, -369, 388, -584, 149);

  if (strcmp(myID, "pulseA") == 0) {
    myX = 0.0;
    myY = 0.0;
  } else {
    myX = 5.0;
    myY = 3.0;
  }

  setupBLE();
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new MyScanCallbacks());
  pScan->setActiveScan(true);
  pScan->start(scanInterval / 1000);

  Serial.println("✅ Setup complete. Starting scan...");
}

// --- Loop ---
void loop() {
  myPressure = bme.readPressure();
  compass.read();
  myHeading = compass.getAzimuth();

  Serial.print("🧭 Heading: ");
  Serial.print(myHeading);
  Serial.print("°, Pressure: ");
  Serial.println(myPressure);

  updateAdvertisement(myX, myY, myPressure);

  if (millis() - lastScanTime > scanInterval) {
    NimBLEDevice::getScan()->clearResults();
    NimBLEDevice::getScan()->start(scanInterval / 1000);
    lastScanTime = millis();
  }

  setDisplayPatternMulti(myHeading, myPressure);
  delay(50);
}
