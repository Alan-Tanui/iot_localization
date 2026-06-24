/*
 * ============================================================
 * Indoor Localization System — Tag Node Firmware
 * ESP32 BLE Advertiser (the device being tracked)
 *
 * Final Year Project — Telecommunications Engineering
 * ============================================================
 *
 * This node simply advertises itself via BLE.
 * The anchor nodes scan for this advertisement and measure RSSI.
 *
 * Flash this to the ESP32 that will be carried/tracked.
 */

#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <Arduino.h>

#define TAG_NAME        "TAG_01"   // Must match anchor firmware
#define TX_POWER        ESP_PWR_LVL_P9  // Max TX power for range

BLEAdvertising* pAdvertising;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[Tag] Starting BLE advertising...");

  BLEDevice::init(TAG_NAME);
  BLEDevice::setPower(TX_POWER);

  pAdvertising = BLEDevice::getAdvertising();

  // Set advertising data
  BLEAdvertisementData advData;
  advData.setName(TAG_NAME);
  advData.setFlags(0x06);  // General discoverable, BR/EDR not supported
  pAdvertising->setAdvertisementData(advData);

  pAdvertising->setMinInterval(100);  // Advertise every 100ms
  pAdvertising->setMaxInterval(200);

  pAdvertising->start();
  Serial.printf("[Tag] Advertising as '%s'. TX power: max.\n", TAG_NAME);
}

void loop() {
  // Tag just keeps advertising — nothing else to do
  delay(1000);
  Serial.println("[Tag] Advertising...");
}
