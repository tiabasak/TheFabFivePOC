#pragma once

#include <string>

class GenericTask {
  public:
    GenericTask(const std::string& name, int stack_size = configMINIMAL_STACK_SIZE, int priority = 1) :
      name_(name), stack_size_(stack_size), priority_(priority) {}

    void Init() {
      xTaskCreatePinnedToCore(TaskWrapper, name_.c_str(), stack_size_, reinterpret_cast<void*>(this),
                              priority_, NULL, ARDUINO_RUNNING_CORE);
    }

  protected:
    virtual void TaskFunc() = 0;

  private:
    static void TaskWrapper(void* param) {
      reinterpret_cast<GenericTask*>(param)->TaskFunc();
    }

    const int priority_;
    const int stack_size_;
    const std::string name_;
};
