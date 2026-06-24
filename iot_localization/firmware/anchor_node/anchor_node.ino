/*
 * ============================================================
 * Indoor Localization System — Anchor Node Firmware
 * ESP32 BLE RSSI Scanner
 *
 * Final Year Project — Telecommunications Engineering
 * ============================================================
 *
 * Instructions:
 *   1. Set ANCHOR_ID below (1, 2, or 3)
 *   2. Set TARGET_TAG_NAME to match your tag's advertised name
 *   3. Flash to ESP32 using Arduino IDE
 *   4. Open Serial Monitor at 115200 baud
 *
 * Output format (JSON over Serial):
 *   {"anchor":1,"rssi":-65,"timestamp":12345}
 */

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <Arduino.h>

// ---- CONFIGURE THESE ----
#define ANCHOR_ID        1          // Change to 2 or 3 for other anchors
#define TARGET_TAG_NAME  "TAG_01"   // Must match tag firmware
#define SCAN_DURATION_S  1          // BLE scan window (seconds)
#define RSSI_SAMPLES     5          // Average over N samples per report
// -------------------------

BLEScan* pBLEScan;
int rssiBuffer[RSSI_SAMPLES];
int bufferIndex = 0;
bool bufferFull = false;

class AdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (String(advertisedDevice.getName().c_str()) == TARGET_TAG_NAME) {
      int rssi = advertisedDevice.getRSSI();

      // Store in rolling buffer
      rssiBuffer[bufferIndex] = rssi;
      bufferIndex = (bufferIndex + 1) % RSSI_SAMPLES;
      if (bufferIndex == 0) bufferFull = true;

      // Compute average
      int count = bufferFull ? RSSI_SAMPLES : bufferIndex;
      long sum = 0;
      for (int i = 0; i < count; i++) sum += rssiBuffer[i];
      int avgRssi = sum / count;

      // Output JSON
      Serial.printf("{\"anchor\":%d,\"rssi\":%d,\"raw\":%d,\"timestamp\":%lu}\n",
                    ANCHOR_ID, avgRssi, rssi, millis());
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.printf("\n[Anchor %d] Initializing BLE scan...\n", ANCHOR_ID);

  BLEDevice::init("Anchor_" + String(ANCHOR_ID));
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);  // Active scan: requests scan response
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  Serial.printf("[Anchor %d] Ready. Scanning for '%s'...\n", ANCHOR_ID, TARGET_TAG_NAME);
}

void loop() {
  BLEScanResults results = pBLEScan->start(SCAN_DURATION_S, false);
  pBLEScan->clearResults();
}
