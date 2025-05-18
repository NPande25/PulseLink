# PulseLink
**COSC 067/267 Final Project – Spring 2025**  
**Contributors:** Mason Childers, Kristian Feed, Michael Maddison, Nikhil Pande

---

## 🔍 Overview

**PulseLink** is a wearable device designed to help individuals locate friends in crowded environments such as festivals, fraternity houses, and parties. The system displays directional and proximity-based feedback using an 8×8 LED matrix, enabling users to navigate toward friends even across multiple floors.

---

## 🧠 How It Works

Each PulseLink device is powered by an ESP32 microcontroller with the following integrated components:

- **BME280 barometric sensor** to estimate relative elevation  
- **QMC5883L compass sensor** to detect the device’s current facing direction (azimuth)  
- **8×8 LED Matrix (MAX7219)** to visualize the locations of nearby friends  
- **Bluetooth Low Energy (BLE)** for inter-device communication

Devices continuously broadcast their `(x, y)` position and air pressure over BLE. Each device:

- Receives friend data (position, elevation)
- Calculates distance and direction based on `(x, y)` and its own compass heading
- Displays relative location as a dot on the LED matrix:
  - Center = user
  - Outer rings = increasing distance
  - Dot **blinks fast** if the friend is **below**, **blinks slow** if **above**, and **is solid** if on the **same floor**

---

## 📦 Hardware Components

- ESP32 Dev Board  
- 8×8 MAX7219 LED Matrix  
- BME280 Sensor (I2C)  
- QMC5883L Compass (I2C)  
- USB Power or Battery  
- Jumper wires + breadboard or wearable housing

---

## 📐 Coordinate System

- One device (e.g., `pulseA`) is fixed at origin `(0, 0)`
- Other devices are positioned relative to this origin (e.g., `(5, 3)`)
- Compass adjusts display orientation to match real-world heading
- BLE signal is only accepted from other `pulseX` devices to avoid interference

---

## 🧰 Setup & Programming

### 1. Wiring

Wire the sensors to the ESP32 using I2C:
- **BME280 & QMC5883L** → `SDA` to GPIO21, `SCL` to GPIO22  
- **MAX7219** → `DIN` to GPIO11, `CLK` to GPIO13, `CS` to GPIO10

### 2. Flash the Firmware

Upload the `PulseLink ESP32 Code` using the Arduino IDE. Make sure:
- Your board is set to “ESP32 Dev Module”
- You’ve installed the necessary libraries:
  - `Adafruit BME280`
  - `Adafruit Sensor`
  - `QMC5883LCompass`
  - `LedControl`
  - `NimBLE-Arduino`

### 3. Adjust Device Identity

Edit the line:
```cpp
const char* myID = "pulseA";
