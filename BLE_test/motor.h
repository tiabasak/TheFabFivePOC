#pragma once

#include <array>
#include <cmath>

#include "generic_task.h"

class Motor {
  public:
    struct PWMPin {
      int pin;
      int channel;
    };

    enum ControlInputs {CONTROL1, CONTROL2, NUM_CONTROL_INPUTS };

    struct Hardware {
      PWMPin controls[NUM_CONTROL_INPUTS]; // PWM control signals
      int disable; // Pull to GND to enable motor.
      int sleep; // used as a reset.
      int current_sense; // ADC input
      int fault; // Active low signal. Indicates faults.
    };

    enum Direction { FORWARD, REVERSE, STOP };

    Motor() {}
    Motor(const Hardware& hardware) : hardware_(hardware) {
      // Init PWM pins. Set to 0 DC.
      ForEachControlInput([this] (const PWMPin & pwm_pin) {
        pinMode(pwm_pin.pin, OUTPUT);
        ledcSetup(pwm_pin.channel, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
        ledcAttachPin(pwm_pin.pin, pwm_pin.channel);
        ledcWrite(pwm_pin.pin, 0);
      });

      adcAttachPin(hardware_.current_sense);
      pinMode(hardware_.fault, INPUT);
      pinMode(hardware.disable, OUTPUT);
      digitalWrite(hardware.disable, HIGH);
    }

    void SetDutyCycle(float duty_cycle) {
      // Transform PWM representation from (-1.0, 1.0) -> [ (0, DUTY_CYCLE_SCALE), DIR ]
      if (-0.0001f < duty_cycle && duty_cycle < 0.0001f) {
        direction_ = STOP;
      } else if (duty_cycle > 0.0f) {
        direction_ = FORWARD;
      } else {
        direction_ = REVERSE;
      }
      duty_cycle_ = fabsf(duty_cycle);

      // This function uses member vars to update hardware outputs.
      UpdateMotorOutputs();
    }

    void SetEnable(bool enable) {
      if (enabled_ != enable) {
        enabled_ = enable;
        digitalWrite(hardware_.disable, enable ? LOW : HIGH);
        SetDutyCycle(0.0f);
      }
    }

    float GetDutyCycle() const {
      return duty_cycle_;
    }

    bool GetEnable() const {
      return enabled_;
    }

    float GetCurrent() const {
      return analogRead(hardware_.current_sense);
    }

    bool GetFault() const {
      return (digitalRead(hardware_.fault) == LOW);
    }
    
  private:
    static constexpr int PWM_RESOLUTION_BITS = 8;
    static constexpr int PWM_FREQUENCY_HZ = 30000;
    static constexpr int DUTY_CYCLE_SCALE = (1 << PWM_RESOLUTION_BITS) - 1;

    void UpdateMotorOutputs() {
      const int inverted_scaled_duty_cycle = DUTY_CYCLE_SCALE * (1.0f - duty_cycle_);

      switch (direction_) {
        case FORWARD:
          SetMotors(DUTY_CYCLE_SCALE, inverted_scaled_duty_cycle);
          break;
        case REVERSE:
          SetMotors(inverted_scaled_duty_cycle, DUTY_CYCLE_SCALE);
          break;
        case STOP:
          SetMotors(DUTY_CYCLE_SCALE, DUTY_CYCLE_SCALE);
          break;
      }
    }

    void SetMotors(int duty_cycle1, int duty_cycle2) {
      ledcWrite(hardware_.controls[CONTROL1].pin, duty_cycle1);
      ledcWrite(hardware_.controls[CONTROL2].pin, duty_cycle2);
    };

    template<typename F>
    const void ForEachControlInput(const F& func) {
      for (int control_index = 0; control_index < NUM_CONTROL_INPUTS; ++control_index) {
        func(hardware_.controls[control_index]);
      }
    }

    Direction direction_;
    float duty_cycle_ = 0.0f; // controlled via in1/in2
    bool enabled_ = false; // controlled via disable pin
    bool fault_ = false;
    Hardware hardware_;
};

constexpr int Motor::PWM_RESOLUTION_BITS;
constexpr int Motor::PWM_FREQUENCY_HZ;
constexpr int Motor::DUTY_CYCLE_SCALE;
