#pragma once

#define DEBUG_LOG(...) println(__VA_ARGS__)

size_t println(){
  return 0;
}

 template<typename T, typename ... TailType>
size_t println(T&& head, TailType&& ...tail){
  size_t r = 0;
  r+=Serial.println(head);
  r+=println((tail)...);
  return r;
}
