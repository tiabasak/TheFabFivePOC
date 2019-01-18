#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "ble_service.h"
#include "motor.h"
#include "utils.h"

iRoboCarBLEServer ble_server_;

void setup() {
  Serial.begin(115200);
  pinMode(5, OUTPUT);
  ble_server_.Init();
}

void loop() {
  digitalWrite(5, HIGH);
  delay(250);
  digitalWrite(5, LOW);
  delay(250);
}
