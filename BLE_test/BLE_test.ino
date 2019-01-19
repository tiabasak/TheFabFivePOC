#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "ble_service.h"
#include "motor.h"
#include "utils.h"
#include "heartbeat_led.h"
#include "motor_controller.h"

iRoboCarBLEServer ble_server_;
HeartBeatLed heartbeat_led_;
MotorController motor_controller_;

void setup() {
  Serial.begin(115200);
  ble_server_.Init();
  motor_controller_.Init();
  heartbeat_led_.Init();
}

void loop() {}
