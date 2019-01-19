#pragma once

#include "generic_task.h"
#include "utils.h"

#ifdef ESP32_THING
#define HEARTBEAT_LED_PIN 5
#else
#define HEARTBEAT_LED_PIN 1
#endif

class HeartBeatLed : public GenericTask {
  public:
    HeartBeatLed() : GenericTask("LED Heartbeat") {
      pinMode(led_pin_, OUTPUT);
    }

  private:
    void TaskFunc() final {
      while (true) {
        led_state_ = !led_state_;
        digitalWrite(led_pin_, led_state_);
        DEBUG_LOG("Led Task Function");
        vTaskDelay(250 / portTICK_PERIOD_MS);
      }
    }

    int led_pin_ = HEARTBEAT_LED_PIN;
    int led_state_ = LOW;
};
