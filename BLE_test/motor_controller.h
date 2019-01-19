#pragma once

#include <array>

#include "motor.h"
#include "utils.h"
#include "generic_task.h"

class MotorController : public GenericTask {
  public:

    enum Motors { MOTOR1, MOTOR2, NUM_MOTORS };

    MotorController() : GenericTask("Motor Control") {}

  private:
    void TaskFunc() final {
      while (true) {
        
        
      }
    }
  
    std::array<Motor, NUM_MOTORS> motors_{};
};
