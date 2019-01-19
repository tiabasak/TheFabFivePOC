#pragma once

#include "utils.h"

class PID {
  public:
    struct Parameters {
      float p_gain;
      float d_gain;
      float i_gain;
      float i_limit;
    };

    void SetParams(const Parameters& params) {
      params_ = params;
    }

    void Reset() {
      i_error_ = 0.0f;
      de_dt_ = 0.0f;
    }

    void SetTarget(float target) {
      target_ = target;
    }

    void AddMeasurement(float new_measurement, int new_measurement_time) {
      const int time_delta_ms = new_measurement_time - last_measurement_time_;
      const float time_delta = time_delta_ms / 1000.0f;

      const float old_error = target_ - last_measurement_;
      const float new_error = target_ - new_measurement;
      const float average_error = (new_error + old_error) / 2.0f;

      i_error_ += (average_error * time_delta);
      i_error_ = clamp(i_error_, -params_.i_limit, params_.i_limit);

      if (time_delta_ms > 0) {
        de_dt_ = (new_error - old_error) / time_delta;
      } else {
        de_dt_ = 0.0f;
      }

      last_measurement_ = new_measurement;
      last_measurement_time_ = new_measurement_time;
    }

    float GetOutput() {
      const float p_term = params_.p_gain * (target_ - last_measurement_);
      const float i_term = params_.i_gain * i_error_;
      const float d_term = params_.d_gain * de_dt_;
      return p_term + i_term + d_term;
    }

  private:
    Parameters params_;
    float target_;
    float last_measurement_;
    int last_measurement_time_;
    float i_error_;
    float de_dt_;
};
