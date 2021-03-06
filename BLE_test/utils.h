#pragma once

#define ESP32_THING

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define DEBUG_LOG(...) println(__VA_ARGS__)

size_t println() {
  return 0;
}

template<typename T, typename ... TailType>
size_t println(T&& head, TailType&& ...tail) {
  size_t r = 0;
  r += Serial.println(head);
  r += println((tail)...);
  return r;
}

#define clamp(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
