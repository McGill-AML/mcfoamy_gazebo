#include "gazebo_example/controller.h"

namespace controllers
{

double saturate(const double value, const double min, const double max)
{
  return std::max(std::min(value, max), min);
}
  
PID::PID(const double kp, const double kd, const double ki):
  kp_(kp)
  , kd_(kd)
  , ki_(ki)
  , output_min_(std::numeric_limits<double>::min())
  , output_max_(std::numeric_limits<double>::max())
  , integral_min_(std::numeric_limits<double>::min())
  , integral_max_(std::numeric_limits<double>::max())
  , integral_error_(0)
{}

double PID::output(const State desired_state, const State measured_state, const double dt)
{
  const State error_state = desired_state - measured_state;
  integral_error_ = saturate(integral_error_ + (error_state.x * dt), integral_min_, integral_max_);
  
  double output = (kp_ * error_state.x)  + (kd_ * error_state.dx) + (ki_ * integral_error_);
  return saturate(output, output_min_, output_max_);
}

void PID::set_ouput_saturation(double min, double max)
{
  output_min_ = min;
  output_max_ = max;
}

void PID::set_integral_saturation(double min, double max)
{
  output_min_ = min;
  output_max_ = max;
}

void PID::reset_integral(const double value)
{
  integral_error_ = value;
}

}
