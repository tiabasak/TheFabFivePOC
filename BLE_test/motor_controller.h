#pragma once

#include <array>

#include "motor.h"
#include "utils.h"
#include "generic_task.h"
#include "pid.h"

class MotorController : public GenericTask {
  public:

    enum Motors { MOTOR_FIRST, MOTOR1 = MOTOR_FIRST, MOTOR2, NUM_MOTORS };

    MotorController() : GenericTask("Motor Control") {}

    const PID& pid(Motors motor) const {
      return pid_[motor];
    }

    PID& pid(Motors motor) {
      return pid_[motor];
    }

    void SetSpeed(float speed_ms) {
      // Feed speed into pid 
       ForEachMotor([&](Motors motor) {
         pid_[motor].SetTarget(speed_ms);
        /// pid_[motor].
       });
    } 

  private:
    void TaskFunc() final {
      while (true) {
        vTaskDelay(1000);
      }
    }

    template<typename Func>
    void ForEachMotor(const Func& func) {
      for (int motor_index = MOTOR_FIRST; motor_index < NUM_MOTORS; ++motor_index) {
        func(static_cast<Motors>(motor_index));  
      }  
    }
  
    std::array<Motor, NUM_MOTORS> motors_{};
    std::array<PID, NUM_MOTORS> pid_{};
    
};
