#pragma once

#include <array>
#include <cmath>

#define M1_EN1_PIN
#define M1_EN2_PIN
#define M1_PWM_PIN
#define M1_PWM_CHANNEL

#define M1_EN1_PIN
#define M1_EN2_PIN
#define M1_PWM_PIN
#define M1_PWM_CHANNEL

class Motor {
  public:
    enum class Motors {
      MOTOR_ONE,
      MOTOR_TWO
    };

    struct Config {
      int in_pinw[2] = { -1};
      int pwm_channels[2] = { -1};
      int freq = 30000;
      int resolution = 8;
    };

    Motor(const Config& config_in) : config_(config_in) {
      DUTY_CYCLE_SCALE = (1 << config_.resolution) - 1;

      pinMode(config_.en1_pin, OUTPUT);
      pinMode(config_.en2_pin, OUTPUT);
      pinMode(config_.pwm_pin, OUTPUT);

      digitalWrite(config_.en1_pin, LOW);
      digitalWrite(config_.en2_pin, LOW);

      ledcSetup(config_.pwm_channel, config_.freq, config_.resolution);
      ledcAttachPin(config_.pwm_pin, config_.pwm_channel);
      ledcWrite(config_.pwm_pin, 0);

    }

    void SetDutyCycle(float duty_cycle) {
      duty_cycle_ = fabsf(duty_cycle);
      const int scaled_duty_cycle = DUTY_CYCLE_SCALE * duty_cycle_;
      ledcWrite(0, 0);
    }
    
    float GetDutyCycle() const {
      return duty_cycle_;
    }
      
    void Halt() {
      
    }


  private:
    float duty_cycle_ = 0.0f;
    bool halted_ = false;
    Config config_;
    int DUTY_CYCLE_SCALE = 0;
};

void Motor::SetDutyCycle(float duty_cycle) {


}
